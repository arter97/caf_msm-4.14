/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

#include "smb235x-charger.h"

#define FLOAT_VOLTAGE_BASE_MV	7200
#define FLOAT_VOLTAGE_STEP_MV	20
#define CURRENT_STEP_MA		50
#define MICRO_TO_MILLI		1000
#define DELAY_WORK_TIME_MS	10000

#define CDP_CURRENT_UA		1500000
#define DCP_CURRENT_UA		1500000
#define HVDCP_CURRENT_UA	3000000
#define SDP_500_MA		500000
#define BASED_VOLTAGE_UV	5000000
#define QC3_DEFAULT_VOLTAGE_UV	9000000
#define QC3_VOLTAGE_STEPS_UV	200000
#define VOLTAGE_FORCE_5V_UV	5000000
#define VOLTAGE_FORCE_9V_UV	9000000
#define VOLTAGE_FORCE_12V_UV	12000000

struct smb235x_irq_data {
	void *parent_data;
	const char *name;
};

struct smb235x_irq_info {
	struct smb235x_irq_data *irq_data;
	const char *name;
	const irq_handler_t handler;
	const bool wake;
	int irq;
};

struct smb235x_dt_props {
	int trickle_charge_current_ua;
	int max_pre_charge_current_ua;
	int pre_charge_current_ua;
	int fast_charge_current_ua;
	int max_fcc_ua;
	int max_fv_uv;
	int termination_current_ua;
	int auto_recharge_soc;
	int float_option;
	int chg_inhibit_thr_uv;
	const char *tcpm_psy_name;
};

struct smb235x_chg_chip {
	struct regmap *regmap;
	struct device *dev;
	struct smb235x_dt_props dt;
	struct power_supply *batt_psy;
	struct power_supply *usb_psy;
	struct work_struct status_change_work;
	struct delayed_work smb235x_update_work;
	struct power_supply *bms_psy;
	struct power_supply *tcpm_psy;
	struct mutex hvdcp_update_voltage_lock;
	struct notifier_block nb;

	int trickle_charge_current_ua;
	int max_pre_charge_current_ua;
	int pre_charge_current_ua;
	int termination_current_ua;
	int float_volt_uv;
	int fastchg_curr_ua;
	int max_fcc_ua;
	int sdp_icl_ua;
	int charger_type;
	enum power_supply_usb_type usb_type;
	int auto_recharge_soc;
	int hvdcp_pulse_count_max;
	int hvdcp3_voltage_uv;
	int based_hvdcp_voltage_uv;
	bool pd_active;
	char tcpm_full_psy_name[64];
};

static int smb235x_set_icl_sw(struct smb235x_chg_chip *chip, int icl_ma)
{
	int rc = 0;
	u8 stat = icl_ma / CURRENT_STEP_MA;

	rc = regmap_update_bits(chip->regmap,
			USBIN_LOAD_CFG_REG,
			ICL_OVERRIDE_AFTER_APSD_BIT,
			ICL_OVERRIDE_AFTER_APSD_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to enable SW icl control rc = %d\n", rc);
		return rc;
	}

	rc = regmap_update_bits(chip->regmap, USB_CMD_ICL_OVERRIDE_REG,
			ICL_OVERRIDE_BIT, ICL_OVERRIDE_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to enable ICL_OVERRIDE rc = %d\n", rc);
		return rc;
	}

	rc = regmap_write(chip->regmap, USBIN_CURRENT_LIMIT_CFG_REG, stat);
	if (rc < 0)
		dev_err(chip->dev, "Failed to set USBIN_CURRENT_LIMIT_CFG_REG rc = %d\n", rc);

	return rc;
}

static int smb235x_enable_charge(struct smb235x_chg_chip *chip, bool enable)
{
	int rc = 0;
	u8 val = enable ? CHARGING_ENABLE_BIT : 0;

	rc = regmap_update_bits(chip->regmap, CHARGING_ENABLE_CMD_REG,
			CHARGING_ENABLE_BIT, val);
	if (rc < 0)
		dev_err(chip->dev, "Failed to set CHARGING_ENABLE_BIT rc = %d\n", rc);

	return rc;
}

static int smb235x_rerun_apsd(struct smb235x_chg_chip *chip)
{
	int rc;

	rc = regmap_update_bits(chip->regmap, USB_CMD_APSD_REG,
		USB_APSD_RERUN_BIT, USB_APSD_RERUN_BIT);
	if (rc < 0)
		dev_err(chip->dev, "Failed to rerun apsd rc = %d\n", rc);

	return rc;
}

static int smb235x_enable_apsd(struct smb235x_chg_chip *chip)
{
	int rc;
	u32 stat = 0;

	chip->hvdcp3_voltage_uv = QC3_DEFAULT_VOLTAGE_UV;
	chip->based_hvdcp_voltage_uv = BASED_VOLTAGE_UV;

	rc = regmap_read(chip->regmap,
		USB_HVDCP_PULSE_COUNT_MAX, &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to read USB_HVDCP_PULSE_COUNT_MAX rc = %d\n", rc);
		return rc;
	}

	chip->hvdcp_pulse_count_max = stat & HVDCP_PULSE_COUNT_MAX_QC3P0_BIT;

	rc = regmap_update_bits(chip->regmap, USBIN_OPTIONS_1_CFG_REG,
			USBIN_HVDCP_AUTH_ALG_EN_BIT |
			USBIN_HVDCP_AUTONOMOUS_MODE_EN_BIT |
			USBIN_APSD_ENABLE_BIT |
			USBIN_HVDCP_EN_BIT,
			USBIN_HVDCP_AUTH_ALG_EN_BIT |
			!USBIN_HVDCP_AUTONOMOUS_MODE_EN_BIT |
			USBIN_APSD_ENABLE_BIT |
			USBIN_HVDCP_EN_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to set USBIN_APSD_ENABLE_BIT rc = %d\n", rc);
		return rc;
	}

	rc = smb235x_rerun_apsd(chip);
	if (rc < 0)
		dev_err(chip->dev, "Failed to rerun apsd rc = %d\n", rc);

	return rc;
}

static int smb235x_get_prop_from_bms(struct smb235x_chg_chip *chip,
		enum power_supply_property prop,
		union power_supply_propval *val)
{
	if (!chip->bms_psy) {
		chip->bms_psy = power_supply_get_by_name("bms");
		if (!chip->bms_psy) {
			dev_dbg(chip->dev, "bms driver not enable\n");
			return -EINVAL;
		}
	}

	return power_supply_get_property(chip->bms_psy, prop, val);
}

static int smb235x_get_prop_from_tcpm(struct smb235x_chg_chip *chip,
	enum power_supply_property prop, union power_supply_propval *val)
{
	if (!chip->tcpm_psy) {
		chip->tcpm_psy = power_supply_get_by_name(chip->tcpm_full_psy_name);
		if (!chip->tcpm_psy) {
			dev_dbg(chip->dev, "tcpm driver not enable\n");
			return -EINVAL;
		}
	}

	return power_supply_get_property(chip->tcpm_psy, prop, val);
}

static bool smb235x_get_usb_online(struct smb235x_chg_chip *chip)
{
	int rc;
	u32 stat = 0;

	rc = regmap_read(chip->regmap, DCDC_POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to read DCDC_POWER_PATH_STATUS_REG rc = %d\n", rc);
		return false;
	}

	return ((stat & USE_USBIN_BIT) && (stat & VALID_INPUT_POWER_SOURCE_STS_BIT));
}

static int smb235x_get_chg_type(struct smb235x_chg_chip *chip)
{
	union power_supply_propval pval;
	int rc = 0;
	u32 stat = 0;
	bool usb_online = false;

	usb_online = smb235x_get_usb_online(chip);
	if (!usb_online) {
		chip->charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
		chip->usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		return 0;
	}

	rc = smb235x_get_prop_from_tcpm(chip, POWER_SUPPLY_PROP_USB_TYPE, &pval);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to get POWER_SUPPLY_PROP_USB_TYPE from tcpm rc = %d\n", rc);
	} else {
		if (pval.intval != POWER_SUPPLY_USB_TYPE_C) {
			chip->charger_type = pval.intval;
			chip->usb_type = pval.intval;
			return 0;
		}
	}

	rc = regmap_read(chip->regmap, USB_APSP_RESULT_STATUS_REG, &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to read USB_APSP_RESULT_STATUS_REG rc = %d\n", rc);
		return rc;
	}

	stat &= APSD_RESULT_STATUS_MASK;

	if (stat & FLOAT_CHARGER_BIT) {
		chip->charger_type = POWER_SUPPLY_TYPE_USB_FLOAT;
		chip->usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	}
	if (stat & DCP_CHARGER_BIT) {
		chip->charger_type = POWER_SUPPLY_TYPE_USB_DCP;
		chip->usb_type = POWER_SUPPLY_USB_TYPE_DCP;
	}
	if (stat & OCP_CHARGER_BIT) {
		chip->charger_type = POWER_SUPPLY_TYPE_USB_DCP;
		chip->usb_type = POWER_SUPPLY_USB_TYPE_DCP;
	}
	if (stat & CDP_CHARGER_BIT) {
		chip->charger_type = POWER_SUPPLY_TYPE_USB_CDP;
		chip->usb_type = POWER_SUPPLY_USB_TYPE_CDP;
	}
	if (stat & SDP_CHARGER_BIT) {
		chip->charger_type = POWER_SUPPLY_TYPE_USB;
		chip->usb_type = POWER_SUPPLY_USB_TYPE_SDP;
	}
	if (stat & QC_3P0_BIT) {
		chip->charger_type = POWER_SUPPLY_TYPE_USB_HVDCP_3;
		chip->usb_type = POWER_SUPPLY_USB_TYPE_DCP;
	}
	if (stat & QC_2P0_BIT) {
		chip->charger_type = POWER_SUPPLY_TYPE_USB_HVDCP;
		chip->usb_type = POWER_SUPPLY_USB_TYPE_DCP;
	}

	return 0;
}

static irqreturn_t smb235x_chg_state_change_irq_handler(int irq, void *data)
{
	struct smb235x_irq_data *irq_data = data;
	struct smb235x_chg_chip *chip = irq_data->parent_data;
	int rc = 0;
	u32 stat = 0;

	dev_dbg(chip->dev, "IRQ: %s\n", irq_data->name);

	rc = regmap_read(chip->regmap, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n", rc);
		return IRQ_NONE;
	}

	stat = stat & BATTERY_CHARGER_STATUS_MASK;

	dev_dbg(chip->dev, "battery charger status is %d\n", stat);

	power_supply_changed(chip->batt_psy);

	return IRQ_HANDLED;
}

static irqreturn_t smb235x_charge_err_irq_handler(int irq, void *data)
{
	struct smb235x_irq_data *irq_data = data;
	struct smb235x_chg_chip *chip = irq_data->parent_data;
	u32 stat = 0;
	int rc = 0;

	dev_dbg(chip->dev, "IRQ: %s\n", irq_data->name);

	rc = regmap_read(chip->regmap, BATTERY_CHARGER_STATUS_2_REG, &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to read BATTERY_CHARGER_STATUS_2_REG rc = %d\n", rc);
		return IRQ_NONE;
	}

	if (stat & CHARGER_ERROR_STATUS_SFT_EXPIRE_BIT)
		dev_info(chip->dev, "Charge error due to CHARGER_ERROR_STATUS_SFT_EXPIRE\n");
	if (stat & CHARGER_ERROR_STATUS_BAT_OV_BIT)
		dev_info(chip->dev, "Charge error due to CHARGER_ERROR_STATUS_BAT_OV\n");
	if (stat & CHARGER_ERROR_STATUS_BAT_TERM_MISSING_BIT)
		dev_info(chip->dev, "Charge error due to CHARGER_ERROR_STATUS_BAT_TERM_MISSING\n");

	return IRQ_HANDLED;
}

static irqreturn_t smb235x_default_irq_handler(int irq, void *data)
{
	struct smb235x_irq_data *irq_data = data;
	struct smb235x_chg_chip *chip = irq_data->parent_data;

	dev_dbg(chip->dev, "IRQ: %s\n", irq_data->name);

	return IRQ_HANDLED;
}

static irqreturn_t smb235x_batt_temp_changed_irq_handler(int irq, void *data)
{
	struct smb235x_irq_data *irq_data = data;
	struct smb235x_chg_chip *chip = irq_data->parent_data;

	dev_dbg(chip->dev, "IRQ: %s\n", irq_data->name);
	power_supply_changed(chip->batt_psy);

	return IRQ_HANDLED;
}

static irqreturn_t smb235x_batt_psy_changed_irq_handler(int irq, void *data)
{
	struct smb235x_irq_data *irq_data = data;
	struct smb235x_chg_chip *chip = irq_data->parent_data;

	dev_dbg(chip->dev, "IRQ: %s\n", irq_data->name);
	power_supply_changed(chip->batt_psy);

	return IRQ_HANDLED;
}

static irqreturn_t smb235x_usbin_uv_irq_handler(int irq, void *data)
{
	struct smb235x_irq_data *irq_data = data;
	struct smb235x_chg_chip *chip = irq_data->parent_data;

	dev_dbg(chip->dev, "IRQ: %s\n", irq_data->name);

	return IRQ_HANDLED;
}

static irqreturn_t smb235x_usbin_ov_irq_handler(int irq, void *data)
{
	struct smb235x_irq_data *irq_data = data;
	struct smb235x_chg_chip *chip = irq_data->parent_data;

	dev_dbg(chip->dev, "IRQ: %s\n", irq_data->name);

	return IRQ_HANDLED;
}

static irqreturn_t smb235x_usb_plugin_irq_handler(int irq, void *data)
{
	struct smb235x_irq_data *irq_data = data;
	struct smb235x_chg_chip *chip = irq_data->parent_data;
	int rc;
	u32 stat = 0;
	bool vbus_rising;

	dev_dbg(chip->dev, "IRQ: %s\n", irq_data->name);

	rc = regmap_read(chip->regmap, USB_INT_RT_STS_OFFSET_REG, &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to read USB_INT_RT_STS_OFFSET_REG rc = %d\n", rc);
		return IRQ_NONE;
	}

	vbus_rising = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);

	if (vbus_rising) {
		rc = smb235x_enable_charge(chip, true);
		if (rc < 0) {
			dev_err(chip->dev, "Failed to enable charge rc = %d\n", rc);
			return IRQ_HANDLED;
		}
	} else {
		rc = smb235x_enable_charge(chip, false);
		if (rc < 0) {
			dev_err(chip->dev, "Failed to disable charge rc = %d\n", rc);
			return IRQ_HANDLED;
		}
	}

	power_supply_changed(chip->usb_psy);
	dev_dbg(chip->dev, "IRQ: usbin-plugin %s\n",
			vbus_rising ? "attached" : "detached");

	return IRQ_HANDLED;
}

static int smb235x_set_hvdcp3_voltage(struct smb235x_chg_chip *chip, int voltage_uv)
{
	int rc, hvdcp_pulse_count;

	if (voltage_uv < BASED_VOLTAGE_UV)
		return 0;

	mutex_lock(&chip->hvdcp_update_voltage_lock);

	if (voltage_uv > chip->based_hvdcp_voltage_uv) {
		hvdcp_pulse_count = (voltage_uv - chip->based_hvdcp_voltage_uv) / QC3_VOLTAGE_STEPS_UV;
		if (hvdcp_pulse_count > chip->hvdcp_pulse_count_max)
			hvdcp_pulse_count = chip->hvdcp_pulse_count_max;

		while (hvdcp_pulse_count--) {
			rc = regmap_update_bits(chip->regmap, USB_CMD_HVDCP_2_REG,
				SINGLE_INCREMENT_BIT, SINGLE_INCREMENT_BIT);
			if (rc < 0) {
				dev_err(chip->dev, "Failed to set USB_CMD_HVDCP_2_REG to increase the voltage rc = %d\n",
						rc);
				mutex_unlock(&chip->hvdcp_update_voltage_lock);
				return rc;
			}
			usleep_range(500, 1000);
		}
	} else {
		hvdcp_pulse_count = (chip->hvdcp3_voltage_uv - voltage_uv) / QC3_VOLTAGE_STEPS_UV;

		while (hvdcp_pulse_count--) {
			rc = regmap_update_bits(chip->regmap, USB_CMD_HVDCP_2_REG,
				SINGLE_DECREMENT_BIT, SINGLE_DECREMENT_BIT);
			if (rc < 0) {
				dev_err(chip->dev, "Failed to set USB_CMD_HVDCP_2_REG to increase the voltage rc = %d\n",
						rc);
				mutex_unlock(&chip->hvdcp_update_voltage_lock);
				return rc;
			}
			usleep_range(500, 1000);
		}
	}

	chip->hvdcp3_voltage_uv = voltage_uv;
	chip->based_hvdcp_voltage_uv = voltage_uv;

	mutex_unlock(&chip->hvdcp_update_voltage_lock);

	return 0;
}

static void smb235x_handle_apsd_done(struct smb235x_chg_chip *chip, bool done)
{
	union power_supply_propval pval;
	int rc, icl_ma;

	if (!done)
		return;

	rc = smb235x_get_chg_type(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to get the charger typer rc = %d\n", rc);
		return;
	}

	if (chip->charger_type != POWER_SUPPLY_TYPE_USB_HVDCP_3)
		chip->based_hvdcp_voltage_uv = BASED_VOLTAGE_UV;

	switch (chip->charger_type) {
	case POWER_SUPPLY_TYPE_USB:
		if (chip->sdp_icl_ua)
			icl_ma = chip->sdp_icl_ua / MICRO_TO_MILLI;
		else
			icl_ma = SDP_500_MA / MICRO_TO_MILLI;
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		icl_ma = CDP_CURRENT_UA / MICRO_TO_MILLI;
		break;
	case POWER_SUPPLY_TYPE_USB_DCP:
		icl_ma = DCP_CURRENT_UA / MICRO_TO_MILLI;
		break;
	case POWER_SUPPLY_TYPE_USB_FLOAT:
		icl_ma = SDP_500_MA / MICRO_TO_MILLI;
		break;
	case POWER_SUPPLY_TYPE_USB_HVDCP:
		icl_ma = HVDCP_CURRENT_UA / MICRO_TO_MILLI;
		rc = regmap_update_bits(chip->regmap, USB_CMD_HVDCP_2_REG, FORCE_9V_BIT, FORCE_9V_BIT);
		if (rc < 0) {
			dev_err(chip->dev, "Failed to update USB_CMD_HVDCP_2_REG rc = %d\n", rc);
			return;
		}
		break;
	case POWER_SUPPLY_TYPE_USB_HVDCP_3:
		icl_ma = HVDCP_CURRENT_UA / MICRO_TO_MILLI;
		rc = smb235x_set_hvdcp3_voltage(chip, chip->hvdcp3_voltage_uv);
		if (rc < 0)
			dev_err(chip->dev, "Failed to set the hvdcp3 voltage rc = %d\n", rc);
		break;
	default:
		icl_ma = SDP_500_MA / MICRO_TO_MILLI;
		break;
	}

	if (chip->pd_active) {
                rc = smb235x_get_prop_from_tcpm(chip,
                                POWER_SUPPLY_PROP_CURRENT_MAX, &pval);
                if (rc < 0) {
                        dev_err(chip->dev, "Failed to get icl from tcpm rc = %d\n", rc);
                        return;
                }

                icl_ma = pval.intval / MICRO_TO_MILLI;
	}

	rc = smb235x_set_icl_sw(chip, icl_ma);
	if (rc < 0)
		dev_err(chip->dev, "Failed to set current of type %d rc = %d\n",
			chip->charger_type, rc);

}

static irqreturn_t smb235x_usb_source_change_irq_handler(int irq, void *data)
{
	struct smb235x_irq_data *irq_data = data;
	struct smb235x_chg_chip *chip = irq_data->parent_data;
	int rc;
	u32 stat = 0;

	dev_dbg(chip->dev, "IRQ: %s\n", irq_data->name);

	rc = regmap_read(chip->regmap, USB_APSD_STATUS_REG, &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to read USB_APSD_STATUS_REG rc = %d\n", rc);
		return IRQ_NONE;
	}

	dev_dbg(chip->dev, "APSD_STATUS = 0x%02x\n", stat);

	smb235x_handle_apsd_done(chip, (bool)(stat & APSD_DTC_STATUS_DONE_BIT));

	power_supply_changed(chip->usb_psy);

	return IRQ_HANDLED;
}

static irqreturn_t smb235x_wdog_bark_irq_handler(int irq, void *data)
{
	struct smb235x_irq_data *irq_data = data;
	struct smb235x_chg_chip *chip = irq_data->parent_data;
	int rc;

	dev_dbg(chip->dev, "IRQ: %s\n", irq_data->name);

	rc = regmap_write(chip->regmap, MISC_BARK_BITE_WDOG_PET_REG,
			BARK_BITE_WDOG_PET_BIT);
	if (rc < 0)
		dev_err(chip->dev, "Failed to reset BARK watchdog rc = %d\n", rc);

	return IRQ_HANDLED;
}

static irqreturn_t smb235x_aicl_done_irq_handler(int irq, void *data)
{
	struct smb235x_irq_data *irq_data = data;
	struct smb235x_chg_chip *chip = irq_data->parent_data;
	int rc;
	u32 stat = 0;

	dev_dbg(chip->dev, "IRQ: %s\n", irq_data->name);

	rc = regmap_read(chip->regmap, DCDC_AICL_ICL_STATUS_REG, &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to read aicl status rc = %d\n", rc);
		return IRQ_NONE;
	}

	dev_dbg(chip->dev, "aicl reault is %dma\n", stat * CURRENT_STEP_MA);

	return IRQ_HANDLED;
}

static struct smb235x_irq_info smb235x_irqs[] = {
	/* CHGR IRQ */
	[CHRG_ERROR_IRQ] = {
		.name = "chgr-error",
		.handler = smb235x_charge_err_irq_handler,
	},

	[CHGR_STATE_CHANGE_IRQ] = {
		.name = "chgr-state-change",
		.handler = smb235x_chg_state_change_irq_handler,
		.wake = true,
	},

	/* DCDC IRQ */
	[OTG_FAIL_IRQ] = {
		.name = "otg-fail",
		.handler = smb235x_default_irq_handler,
	},

	[INPUT_CURRENT_LIMITING_IRQ] = {
		.name = "input-current-limit",
		.handler = smb235x_default_irq_handler,
	},

	/* BATIF IRQ */
	[BAT_TEMP_IRQ] = {
		.name = "batt-temp",
		.handler = smb235x_batt_temp_changed_irq_handler,
		.wake = true,
	},

	[BAT_OV_IRQ] = {
		.name = "batt-ov",
		.handler = smb235x_batt_psy_changed_irq_handler,
	},

	[BAT_LOW_IRQ] = {
		.name = "batt-low",
		.handler = smb235x_batt_psy_changed_irq_handler,
	},

	[BAT_THERM_OR_ID_MISSING_IRQ] = {
		.name = "batt-therm-or-id-missing",
		.handler = smb235x_batt_psy_changed_irq_handler,
	},

	[BAT_TERMINAL_MISSING_IRQ] = {
		.name = "batt-terminal-missing",
		.handler = smb235x_batt_psy_changed_irq_handler,
	},

	/* USBIN IRQ */
	[USBIN_COLLAPSE_IRQ] = {
		.name = "usbin-collapse",
		.handler = smb235x_default_irq_handler,
	},

	[USBIN_VASHDN_IRQ] = {
		.name = "usbin-vashdn",
		.handler = smb235x_default_irq_handler,
	},

	[USBIN_UV_IRQ] = {
		.name	= "usbin-uv",
		.handler = smb235x_usbin_uv_irq_handler,
		.wake = true,
	},

	[USBIN_OV_IRQ] = {
		.name = "usbin-ov",
		.handler = smb235x_usbin_ov_irq_handler,
	},

	[USBIN_PLUGIN_IRQ] = {
		.name = "usbin-plugin",
		.handler = smb235x_usb_plugin_irq_handler,
		.wake = true,
	},

	[USBIN_SRC_CHANGE_IRQ] = {
		.name = "usbin-src-change",
		.handler = smb235x_usb_source_change_irq_handler,
		.wake = true,
	},

	[USBIN_ICL_CHANGE_IRQ] = {
		.name = "usbin-icl-change",
		.handler = smb235x_default_irq_handler,
		.wake = true,
	},

	/* MISC IRQ */
	[AICL_DONE_IRQ] = {
		.name = "aicl-done",
		.handler = smb235x_aicl_done_irq_handler,
	},

	[TEMP_CHANGE_IRQ] = {
		.name = "temp-change",
		.handler = smb235x_default_irq_handler,
	},

	[WDOG_BARK_IRQ] = {
		.name = "wdog-bark",
		.handler = smb235x_wdog_bark_irq_handler,
		.wake = true,
	}
};

static int smb235x_parse_dt(struct smb235x_chg_chip *chip)
{
	struct device_node *node = chip->dev->of_node;
	int rc;

	chip->trickle_charge_current_ua = 50000;
	chip->pre_charge_current_ua = 750000;
	chip->max_pre_charge_current_ua = 1000000;
	chip->fastchg_curr_ua = 3250000;
	chip->max_fcc_ua = 3250000;
	chip->termination_current_ua = -325000;
	chip->float_volt_uv = 8800000;
	chip->auto_recharge_soc = 98;
	chip->dt.tcpm_psy_name = devm_kzalloc(chip->dev, 48, GFP_KERNEL);
	if (!chip->dt.tcpm_psy_name)
		return -ENOMEM;

	chip->dt.max_fv_uv = -EINVAL;
	rc = of_property_read_u32(node, "qcom,fv-max-uv", &chip->dt.max_fv_uv);
	if (!rc)
		chip->float_volt_uv = chip->dt.max_fv_uv;

	chip->dt.max_fcc_ua = -EINVAL;
	rc = of_property_read_u32(node, "qcom,fcc-max-ua", &chip->dt.max_fcc_ua);
	if (!rc)
		chip->max_fcc_ua = chip->dt.max_fcc_ua;

	chip->dt.fast_charge_current_ua = -EINVAL;
	rc = of_property_read_u32(node, "qcom,fast-charge-current-ua",
			&chip->dt.fast_charge_current_ua);
	if (!rc)
		chip->fastchg_curr_ua = chip->dt.fast_charge_current_ua;

	chip->dt.trickle_charge_current_ua = -EINVAL;
	rc = of_property_read_u32(node, "qcom,trickle-current-ua",
			&chip->dt.trickle_charge_current_ua);
	if (!rc)
		chip->trickle_charge_current_ua =
			chip->dt.trickle_charge_current_ua;

	chip->dt.pre_charge_current_ua = -EINVAL;
	rc = of_property_read_u32(node, "qcom,precharge-current-ua",
			&chip->dt.pre_charge_current_ua);
	if (!rc)
		chip->pre_charge_current_ua =
			chip->dt.pre_charge_current_ua;

	chip->dt.max_pre_charge_current_ua = -EINVAL;
	rc = of_property_read_u32(node, "qcom,max-precharge-current-ua",
			&chip->dt.max_pre_charge_current_ua);
	if (!rc)
		chip->max_pre_charge_current_ua =
			chip->dt.max_pre_charge_current_ua;

	chip->dt.termination_current_ua = -EINVAL;
	rc = of_property_read_u32(node, "qcom,termination-current-ma",
			&chip->dt.termination_current_ua);
	if (!rc)
		chip->termination_current_ua = chip->dt.termination_current_ua;

	chip->dt.auto_recharge_soc = -EINVAL;
	rc = of_property_read_u32(node, "qcom,auto-recharge-soc",
			&chip->dt.auto_recharge_soc);
	if (!rc)
		chip->auto_recharge_soc = chip->dt.auto_recharge_soc;

	rc = of_property_read_u32(node, "qcom,float-option", &chip->dt.float_option);
	if (rc < 0)
		chip->dt.float_option = -EINVAL;

	rc = of_property_read_u32(node, "qcom,chg-inhibit-threshold-mv",
			&chip->dt.chg_inhibit_thr_uv);
	if (rc < 0)
		chip->dt.chg_inhibit_thr_uv = -EINVAL;

	rc = of_property_read_string(node, "tcpm-psy-name",
			&chip->dt.tcpm_psy_name);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to get the tcpm-power-supply-node rc=%d\n", rc);
		return rc;
	}

	dev_dbg(chip->dev, "complete smb2352 parse dt rc = %d\n", rc);

	return 0;
}

static int smb235x_config_chg_current_voltage(struct smb235x_chg_chip *chip)
{
	int rc;
	u8 reg_val = 0;

	reg_val = (chip->fastchg_curr_ua / MICRO_TO_MILLI / CURRENT_STEP_MA) + 1;
	rc = regmap_write(chip->regmap,
			CHGR_FAST_CHARGE_CURRENT_CFG_REG, reg_val);
	if (rc < 0) {
		dev_err(chip->dev, "Faile to write fast charge current rc = %d\n", rc);
		return rc;
	}

	reg_val = (chip->max_fcc_ua / MICRO_TO_MILLI / CURRENT_STEP_MA) + 1;
	rc = regmap_write(chip->regmap,
			CHGR_MAX_FAST_CHARGE_CURRENT_CFG_REG, reg_val);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to write max fast charge current rc = %d\n", rc);
		return rc;
	}

	reg_val = (chip->float_volt_uv / MICRO_TO_MILLI - FLOAT_VOLTAGE_BASE_MV)
		/ FLOAT_VOLTAGE_STEP_MV;
	rc = regmap_write(chip->regmap, CHGR_FLOAT_VOLTAGE_CFG_REG, reg_val);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to write float voltage rc = %d\n", rc);
		return rc;
	}

	reg_val = chip->trickle_charge_current_ua / MICRO_TO_MILLI / CURRENT_STEP_MA;
	rc = regmap_write(chip->regmap,
			CHGR_TRICKLE_CHARGE_CURRENT_CFG_REG, reg_val);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to write trickle charge current rc = %d\n", rc);
		return rc;
	}

	reg_val = chip->pre_charge_current_ua / MICRO_TO_MILLI / CURRENT_STEP_MA;
	rc = regmap_write(chip->regmap,
			CHGR_PRE_CHARGE_CURRENT_CFG_REG, reg_val);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to write pre charge current rc = %d\n", rc);
		return rc;
	}

	reg_val = chip->max_pre_charge_current_ua / MICRO_TO_MILLI / CURRENT_STEP_MA;
	rc = regmap_write(chip->regmap,
			CHGR_MAX_PRE_CHARGER_CURRENT_CFG_REG, reg_val);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to write max pre charge current rc = %d\n", rc);
		return rc;
	}

	return rc;
}

static int smb235x_config_aicl(struct smb235x_chg_chip *chip)
{
	u8 mask, val;
	int rc;

	mask = USBIN_AICL_PERIODIC_RERUN_EN_BIT | USBIN_AICL_EN_BIT;
	val = mask;

	rc = regmap_update_bits(chip->regmap,
			USBIN_AICL_OPTIONS_CFG_REG, mask, val);
	if (rc < 0)
		dev_err(chip->dev, "Failed to set aicl rc = %d\n", rc);

	return rc;
}

static int smb235x_enable_watchdog(struct smb235x_chg_chip *chip)
{
	int rc;
	u8 val = BARK_WDOG_INT_EN_BIT | WDOG_TIMER_EN_ON_PLUGIN_BIT;

	rc = regmap_update_bits(chip->regmap, MISC_WD_CFG_REG,
			BARK_WDOG_INT_EN_BIT
			| WDOG_TIMER_EN_ON_PLUGIN_BIT, val);
	if (rc < 0)
		dev_err(chip->dev, "Failed to set MISC_WD_CFG_REG rc = %d\n", rc);

	return rc;
}

static int smb235x_config_charge_termination(struct smb235x_chg_chip *chip)
{
	int reg_val = 0;
	int rc;

	reg_val = chip->termination_current_ua / MICRO_TO_MILLI
			/ CURRENT_STEP_MA;

	rc = regmap_write(chip->regmap,
			CHGR_CHARGE_CURRENT_TERMINATION_CFG_REG, reg_val);
	if (rc < 0)
		dev_err(chip->dev, "Failed to config ITERM threshold HIGH rc = %d\n", rc);

	return rc;
}

static int smb235x_config_recharge(struct smb235x_chg_chip *chip)
{
	int rc;

	rc = regmap_update_bits(chip->regmap, CHGR_CFG2_REG,
			SOC_BASED_RECHG_BIT, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to config VBAT-recharge CHG_CFG2_REG rc = %d\n", rc);
		return rc;
	}

	rc = regmap_write(chip->regmap, CHGR_RCHG_SOC_THRESHOLD_CFG_REG,
			chip->dt.auto_recharge_soc);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to config recharger SOC\n rc = %d\n", rc);
		return rc;
	}

	return rc;
}

static int smb235x_config_float_charge(struct smb235x_chg_chip *chip)
{
	u8 val;
	int rc = 0;

	if (chip->dt.float_option == -EINVAL)
		return rc;

	switch (chip->dt.float_option) {
	case FLOAT_SDP:
		val = FORCE_FLOAT_SDP_CFG_BIT;
		break;
	case DISABLE_CHARGING:
		val = SUSPEND_FLOAT_CFG_BIT;
		break;
	case SUSPEND_INPUT:
		val = FLOAT_DIS_CHGING_CFG_BIT;
		break;
	default:
		val = 0;
		return rc;
	}

	rc = regmap_update_bits(chip->regmap, USBIN_OPTIONS_2_CFG_REG,
		FLOAT_OPTIONS_MASK, val);
	if (rc < 0)
		dev_err(chip->dev, "Failed to set float charge rc = %d\n", rc);

	return rc;
}

static int smb235x_config_inhibit(struct smb235x_chg_chip *chip)
{
	int rc = 0;
	u8 val = 0;
	int chg_inhibit_thr_mv;

	if (chip->dt.chg_inhibit_thr_uv == -EINVAL)
		return rc;

	rc = regmap_update_bits(chip->regmap, CHGR_CFG2_REG, CHARGER_INHIBIT_BIT,
	(chip->dt.chg_inhibit_thr_uv ? CHARGER_INHIBIT_BIT : 0));
	if (rc < 0) {
		dev_err(chip->dev, "Failed to config inhibit mode rc = %d\n", rc);
		return rc;
	}

	chg_inhibit_thr_mv = chip->dt.chg_inhibit_thr_uv / MICRO_TO_MILLI;

	switch (chg_inhibit_thr_mv) {
	case 100:
		val = INHIBIT_ANALOG_VFLT_MINUS_100MV;
		break;
	case 200:
		val = INHIBIT_ANALOG_VFLT_MINUS_200MV;
		break;
	case 400:
		val = INHIBIT_ANALOG_VFLT_MINUS_400MV;
		break;
	case 600:
		val = INHIBIT_ANALOG_VFLT_MINUS_600MV;
		break;
	default:
		dev_err(chip->dev, "Invalid inhibit threshold value\n");
		return -EINVAL;
	}

	rc = regmap_update_bits(chip->regmap, CHARGE_INHIBIT_THRESHOLD_CFG_REG,
			CHARGE_INHIBIT_THRESHOLD_MASK, val);
	if (rc < 0)
		dev_err(chip->dev, "Failed to config the charge inhibit threshold rc = %d\n", rc);

	return rc;
}

static int smb235x_chg_init(struct smb235x_chg_chip *chip)
{
	int rc;

	rc = smb235x_config_chg_current_voltage(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to config the charging current and voltage rc = %d\n", rc);
		return rc;
	}

	rc = smb235x_enable_apsd(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to enable APSD rc= %d\n", rc);
		return rc;
	}

	rc = smb235x_config_aicl(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to config aicl rc= %d\n", rc);
		return rc;
	}

	rc = smb235x_enable_watchdog(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to enable watchdog rc = %d\n", rc);
		return rc;
	}

	rc = smb235x_config_charge_termination(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to config charge termination rc = %d\n", rc);
		return rc;
	}

	rc = smb235x_config_recharge(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to config recharger\n");
		return rc;
	}

	rc = smb235x_config_float_charge(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to config float charge\n");
		return rc;
	}

	rc = smb235x_config_inhibit(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to config inhibit rc = %d\n", rc);
		return rc;
	}

	rc = smb235x_enable_charge(chip, true);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to enable charge rc = %d\n", rc);
		return rc;
	}

	return rc;
}

static int smb235x_request_interrupts(struct smb235x_chg_chip *chip)
{
	struct device_node *node = chip->dev->of_node;
	struct smb235x_irq_data *irq_data;
	int rc, i;
	int irq = 0;

	for (i = 0; i < ARRAY_SIZE(smb235x_irqs); i++) {
		if (!smb235x_irqs[i].handler)
			return 0;

		irq_data = devm_kzalloc(chip->dev, sizeof(*irq_data), GFP_KERNEL);
		if (!irq_data)
			return -ENOMEM;

		irq_data->parent_data = chip;
		irq_data->name = smb235x_irqs[i].name;

		irq = of_irq_get_byname(node, smb235x_irqs[i].name);
		if (irq <= 0) {
			dev_err(chip->dev, "Couldn't get irq %s byname\n", smb235x_irqs[i].name);
			return irq;
		}

		rc = devm_request_threaded_irq(chip->dev, irq, NULL,
			smb235x_irqs[i].handler,
			IRQF_ONESHOT, irq_data->name, irq_data);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't request irq %d\n", irq);
			return rc;
		}

		smb235x_irqs[i].irq = irq;
		smb235x_irqs[i].irq_data = irq_data;

		if (smb235x_irqs[i].wake)
			enable_irq_wake(irq);
	}

	return 0;
}

static int smb235x_get_prop_usb_present(struct smb235x_chg_chip *chip,
			union power_supply_propval *val)
{
	int rc;
	u32 stat = 0;

	rc = regmap_read(chip->regmap, USB_INT_RT_STS_OFFSET_REG, &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to read USB_INT_RT_STS_OFFSET_REG rc = %d\n", rc);
		return rc;
	}

	val->intval = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);
	return 0;
}

static int smb235x_get_prop_usb_icl(struct smb235x_chg_chip *chip,
			union power_supply_propval *val)
{
	int rc;
	u32 stat = 0;
	u32 override_stat;
	bool usb_online;

	usb_online = smb235x_get_usb_online(chip);
	if (!usb_online) {
		val->intval = 0;
		return 0;
	}

	rc = regmap_read(chip->regmap, USB_CMD_ICL_OVERRIDE_REG, &override_stat);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to read USB_CMD_ICL_OVERRIDE_REG rc = %d\n", rc);
		return rc;
	}

	override_stat &= ICL_OVERRIDE_BIT;

	if (override_stat == 1) {
		rc = regmap_read(chip->regmap, USBIN_CURRENT_LIMIT_CFG_REG, &stat);
		if (rc < 0) {
			dev_err(chip->dev, "Failed to read USB_CMD_ICL_OVERRIDE_REG rc = %d\n", rc);
			return rc;
		}

		val->intval = stat * CURRENT_STEP_MA;
	} else {
		rc = regmap_read(chip->regmap, DCDC_ICL_MAX_STATUS_REG, &stat);
		if (rc < 0) {
			dev_err(chip->dev, "Failed to read DCDC_ICL_MAX_STATUS_REG rc = %d\n", rc);
			return rc;
		}

		val->intval = stat * CURRENT_STEP_MA;
	}

	val->intval *= MICRO_TO_MILLI;

	return rc;
}

static int smb235x_get_hvdcp2_voltage(struct smb235x_chg_chip *chip)
{
	int rc;
	u32 stat = 0;
	int voltage_uv;

	rc = regmap_read(chip->regmap, USB_QC_CHANGE_STATUS_REG, &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to read USB_QC_CHANGE_STATUS_REG rc = %d\n", rc);
		return rc;
	}

	if (stat & QC_12V_BIT)
		voltage_uv = VOLTAGE_FORCE_12V_UV;
	else if (stat & QC_9V_BIT)
		voltage_uv = VOLTAGE_FORCE_9V_UV;
	else if (stat & QC_5V_BTI)
		voltage_uv = VOLTAGE_FORCE_5V_UV;
	else
		voltage_uv = VOLTAGE_FORCE_5V_UV;

	return voltage_uv;
}

static int smb235x_get_prop_usb_voltage(struct smb235x_chg_chip *chip,
		union power_supply_propval *val)
{
	int rc = 0;
	bool usb_online;

	usb_online = smb235x_get_usb_online(chip);
	if (!usb_online) {
		val->intval = 0;
		return rc;
	}

	switch (chip->charger_type) {
	case POWER_SUPPLY_TYPE_USB_FLOAT:
	case POWER_SUPPLY_TYPE_USB_DCP:
	case POWER_SUPPLY_TYPE_USB_CDP:
	case POWER_SUPPLY_TYPE_USB:
		val->intval = VOLTAGE_FORCE_5V_UV;
		break;
	case POWER_SUPPLY_TYPE_USB_HVDCP_3:
		val->intval = chip->hvdcp3_voltage_uv;
		break;
	case POWER_SUPPLY_TYPE_USB_HVDCP:
		val->intval = smb235x_get_hvdcp2_voltage(chip);
		break;
	default:
		val->intval = VOLTAGE_FORCE_5V_UV;
		break;
	}

	if (chip->pd_active) {
		rc = smb235x_get_prop_from_tcpm(chip,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, val);
		if (rc < 0)
			dev_err(chip->dev, "Failed to get maximum voltage from tcpm rc = %d\n", rc);
	}

	return rc;
}

static int smb235x_usb_get_prop(struct power_supply *psy,
			enum power_supply_property prop,
			union power_supply_propval *pval)
{
	struct smb235x_chg_chip *chip = power_supply_get_drvdata(psy);
	int rc = 0;
	bool usb_online;

	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		rc = smb235x_get_prop_usb_present(chip, pval);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		pval->intval = smb235x_get_usb_online(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = smb235x_get_prop_usb_icl(chip, pval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		rc = smb235x_get_prop_usb_voltage(chip, pval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = smb235x_get_prop_usb_voltage(chip, pval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		usb_online = smb235x_get_usb_online(chip);
		if (!usb_online) {
			pval->intval = 0;
			return 0;
		}
		rc = smb235x_get_prop_from_tcpm(chip, prop, pval);
		break;
	case POWER_SUPPLY_PROP_SDP_CURRENT_MAX:
		if (chip->sdp_icl_ua)
			pval->intval = chip->sdp_icl_ua;
		else
			pval->intval = SDP_500_MA;
		break;
	case POWER_SUPPLY_PROP_REAL_TYPE:
		rc = smb235x_get_chg_type(chip);
		if (!rc)
			pval->intval = chip->charger_type;
		else
			pval->intval = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		rc = smb235x_get_chg_type(chip);
		if (!rc)
			pval->intval = chip->usb_type;
		else
			pval->intval = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
	default:
		dev_err(chip->dev, "get prop %d is not supported in usb\n", prop);
		return -EINVAL;
	}

	if (rc < 0) {
		dev_err(chip->dev, "Failed to get prop %d rc = %d\n", prop, rc);
		return -ENODATA;
	}

	return rc;
}

static int smb235x_set_prop_usb_input_current_limit
		(struct smb235x_chg_chip *chip,
		const union power_supply_propval *val)
{
	int rc;
	u8 icl_ma = val->intval / MICRO_TO_MILLI;

	rc = smb235x_set_icl_sw(chip, icl_ma);
	if (rc < 0)
		dev_err(chip->dev, "Failed to set icl rc = %d\n", rc);

	return rc;
}

static int smb235x_set_prop_usb_voltage_now(struct smb235x_chg_chip *chip,
		const union power_supply_propval *pval)
{
	int rc;

	switch (chip->charger_type) {
	case POWER_SUPPLY_TYPE_USB_HVDCP_3:
		rc = smb235x_set_hvdcp3_voltage(chip, pval->intval);
		break;
	default:
		dev_dbg(chip->dev, "Nosupport set voltage now\n");
		break;
	}

	return 0;
}

static int smb235x_usb_set_prop(struct power_supply *psy,
				enum power_supply_property prop,
				const union power_supply_propval *pval)
{
	struct smb235x_chg_chip *chip = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = smb235x_set_prop_usb_input_current_limit(chip, pval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		rc = smb235x_set_prop_usb_voltage_now(chip, pval);
		break;
	case POWER_SUPPLY_PROP_SDP_CURRENT_MAX:
		chip->sdp_icl_ua = pval->intval;
		rc = smb235x_set_prop_usb_input_current_limit(chip, pval);
		break;
	default:
		dev_err(chip->dev, "Set prop %d is not supported in usb psy\n", prop);
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int smb235x_usb_prop_is_writeable(struct power_supply *psy,
			enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_SDP_CURRENT_MAX:
		return 1;
	default:
		break;
	}

	return 0;
}

static enum power_supply_property smb235x_usb_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_SDP_CURRENT_MAX,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_REAL_TYPE,
};

static enum power_supply_usb_type usb_psy_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
	POWER_SUPPLY_USB_TYPE_PD_PPS,
};

static const struct power_supply_desc usb_psy_desc = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = usb_psy_usb_types,
	.num_usb_types = ARRAY_SIZE(usb_psy_usb_types),
	.properties = smb235x_usb_props,
	.num_properties = ARRAY_SIZE(smb235x_usb_props),
	.get_property = smb235x_usb_get_prop,
	.set_property = smb235x_usb_set_prop,
	.property_is_writeable = smb235x_usb_prop_is_writeable,
};

static int smb235x_init_usb_psy(struct smb235x_chg_chip *chip)
{
	struct power_supply_config usb_cfg = {};

	usb_cfg.drv_data = chip;
	usb_cfg.of_node = chip->dev->of_node;

	chip->usb_psy = devm_power_supply_register(chip->dev, &usb_psy_desc, &usb_cfg);
	if (IS_ERR(chip->usb_psy)) {
		dev_err(chip->dev, "Couldn't register USB power supply rc = %d\n",
				PTR_ERR(chip->usb_psy));
		return PTR_ERR(chip->usb_psy);
	}

	return 0;
}

static int smb235x_get_prop_batt_present(struct smb235x_chg_chip *chip,
		union power_supply_propval *val)
{
	int rc;
	u32 stat = 0;

	rc = regmap_read(chip->regmap, BATIF_INT_RT_STS_OFFSET_REG, &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to read BATIF_INT_RT_STS_OFFSET_REG rc = %d\n", rc);
		return rc;
	}

	val->intval = !(stat & (BAT_TERMINAL_MISSING_RT_STS_BIT |
				BAT_THERM_OR_ID_MISSING_RT_STS_BIT));

	return rc;
}

static int smb235x_get_prop_batt_charge_type(struct smb235x_chg_chip *chip,
		union power_supply_propval *val)
{
	int rc;
	u32 stat = 0;

	rc = regmap_read(chip->regmap, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to read BATTERY_CHARGER_STATUS_1_REG rc = %d\n", rc);
		return rc;
	}

	switch (stat & BATTERY_CHARGER_STATUS_MASK) {
	case TRICKLE_CHARGE:
	/* fallthrough */
	case PRE_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	case FULLON_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	case TAPER_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_TAPER;
		break;
	default:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
	}

	return rc;
}

static int smb235x_get_prop_batt_health(struct smb235x_chg_chip *chip,
		union power_supply_propval *val)
{
	int rc;
	u32 stat = 0;

	rc = regmap_read(chip->regmap, BATTERY_CHARGER_STATUS_7_REG, &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to read BATTERY_CHARGER_STATUS_7_REG rc = %d\n", rc);
		return rc;
	}

	if (stat & BAT_TEMP_STATUS_TOO_COLD_BIT)
		val->intval = POWER_SUPPLY_HEALTH_COLD;
	else if (stat & BAT_TEMP_STATUS_TOO_HOT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (stat & BAT_TEMP_STATUS_COLD_SOFT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_COOL;
	else if (stat & BAT_TEMP_STATUS_HOT_SOFT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_WARM;
	else
		val->intval = POWER_SUPPLY_HEALTH_GOOD;

	return rc;
}

static int smb235x_get_prop_battery_status(struct smb235x_chg_chip *chip,
		union power_supply_propval *pval)
{
	int rc;
	u32 stat = 0;
	bool usb_online;

	usb_online = smb235x_get_usb_online(chip);

	rc = regmap_read(chip->regmap, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to read BATTERY_CHARGER_STATUS_1_REG rc = %d\n", rc);
		return rc;
	}

	stat &= BATTERY_CHARGER_STATUS_MASK;

	if (!usb_online) {
		switch (stat) {
		case TERMINATE_CHARGE:
		/* fallthrough */
		case INHIBIT_CHARGE:
			pval->intval = POWER_SUPPLY_STATUS_FULL;
			return rc;
		default:
			pval->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			return rc;
		}
	}

	switch (stat) {
	case TRICKLE_CHARGE:
	/* fallthrough */
	case PRE_CHARGE:
	/* fallthrough */
	case FULLON_CHARGE:
	/* fallthrough */
	case TAPER_CHARGE:
	/* fallthrough */
		pval->intval = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case TERMINATE_CHARGE:
		pval->intval = POWER_SUPPLY_STATUS_FULL;
		break;
	case INHIBIT_CHARGE:
	/* fallthrough */
	case PAUSE_CHARGE:
	/* fallthrough */
	case DISABLE_CHARGE:
		pval->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	default:
		pval->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		return rc;
	}

	return rc;
}

static int smb235x_batt_get_prop(struct power_supply *psy,
		enum power_supply_property prop,
		union power_supply_propval *pval)
{
	struct smb235x_chg_chip *chip = power_supply_get_drvdata(psy);
	int rc = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		rc = smb235x_get_prop_batt_present(chip, pval);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		rc = smb235x_get_prop_battery_status(chip, pval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		rc = smb235x_get_prop_batt_charge_type(chip, pval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		rc = smb235x_get_prop_from_bms(chip, prop, pval);
		if (rc < 0)
			rc = smb235x_get_prop_batt_health(chip, pval);
		break;
	case POWER_SUPPLY_PROP_TEMP:
	/* fallthrough */
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	/* fallthrough */
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	/* fallthrough */
	case POWER_SUPPLY_PROP_CAPACITY:
		rc = smb235x_get_prop_from_bms(chip, prop, pval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		pval->intval = chip->float_volt_uv;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		pval->intval = chip->fastchg_curr_ua;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		rc = smb235x_get_prop_from_bms(chip, prop, pval);
		if (rc < 0)
			pval->intval = chip->fastchg_curr_ua;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		rc = smb235x_get_prop_from_bms(chip, prop, pval);
		if (rc < 0)
			pval->intval = chip->float_volt_uv;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		pval->intval = chip->termination_current_ua;
		break;
	default:
		break;
	}

	if (rc < 0) {
		dev_err(chip->dev, "Failed to get prop %d rc = %d", prop, rc);
		return -ENODATA;
	}

	return 0;
}

static int smb235x_set_fv(struct smb235x_chg_chip *chip, int vfloat_uv)
{
	int rc;
	u8 reg_val = 0;

	reg_val = (vfloat_uv / MICRO_TO_MILLI - FLOAT_VOLTAGE_BASE_MV) /
		FLOAT_VOLTAGE_STEP_MV;
	rc = regmap_write(chip->regmap, CHGR_FLOAT_VOLTAGE_CFG_REG, reg_val);
	if (rc < 0)
		dev_err(chip->dev, "Failed to write float voltage rc = %d\n", rc);

	return rc;
}

static int smb235x_set_fcc(struct smb235x_chg_chip *chip, int fcc_ua)
{
	int rc;
	u8 reg_val = 0;

	reg_val = (fcc_ua / MICRO_TO_MILLI / CURRENT_STEP_MA) + 1;
	rc = regmap_write(chip->regmap, CHGR_FAST_CHARGE_CURRENT_CFG_REG,
			reg_val);
	if (rc < 0)
		dev_err(chip->dev, "Faile to write fast charge current rc = %d\n", rc);

	return rc;
}

static int smb235x_batt_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *pval)
{
	struct smb235x_chg_chip *chip = power_supply_get_drvdata(psy);
	union power_supply_propval bms_val;
	bool bms_prop_exist = false;
	int rc;

	rc = smb235x_get_prop_from_bms(chip, prop, &bms_val);
	if (rc < 0)
		dev_err(chip->dev, "Failed to get prop %d from bms\n", rc);
	else
		bms_prop_exist = true;

	switch (prop) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		if (bms_prop_exist)
			chip->float_volt_uv = bms_val.intval;
		else
			chip->float_volt_uv = pval->intval;

		rc = smb235x_set_fv(chip, chip->float_volt_uv);
		if (rc < 0)
			dev_err(chip->dev, "Failed to set FV rc = %d\n", rc);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		if (bms_prop_exist)
			chip->fastchg_curr_ua = bms_val.intval;
		else
			chip->fastchg_curr_ua = pval->intval;

		rc = smb235x_set_fcc(chip, chip->fastchg_curr_ua);
		if (rc < 0)
			dev_err(chip->dev, "Failed to set FCC rc = %d\n", rc);
		break;
	default:
		rc = -EINVAL;
	}

	return rc;
}

static int smb235x_batt_prop_is_writeable(struct power_supply *psy,
			enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		return 1;
	default:
		break;
	}

	return 0;
}

static enum power_supply_property smb235x_batt_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
};

static const struct power_supply_desc batt_psy_desc = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = smb235x_batt_props,
	.num_properties = ARRAY_SIZE(smb235x_batt_props),
	.get_property = smb235x_batt_get_prop,
	.set_property = smb235x_batt_set_prop,
	.property_is_writeable = smb235x_batt_prop_is_writeable,
};

static int smb235x_init_battery_psy(struct smb235x_chg_chip *chip)
{
	struct power_supply_config batt_cfg = {};

	batt_cfg.drv_data = chip;
	batt_cfg.of_node = chip->dev->of_node;

	chip->batt_psy = devm_power_supply_register(chip->dev,
			&batt_psy_desc, &batt_cfg);
	if (IS_ERR(chip->batt_psy)) {
		dev_err(chip->dev, "Couldn't register battery power supply\n");
		return PTR_ERR(chip->batt_psy);
	}

	return 0;
}

static void smb235x_tcpm_update_icl(struct smb235x_chg_chip *chip)
{
	union power_supply_propval pval;
	int rc, icl_ma;

	rc = smb235x_get_prop_from_tcpm(chip,
			POWER_SUPPLY_PROP_USB_TYPE, &pval);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to get POWER_SUPPLY_PROP_USB_TYPE from tcpm rc = %d\n",
				rc);
		return;
	} else {
		if (pval.intval != POWER_SUPPLY_USB_TYPE_C) {
			chip->pd_active = true;
		} else {
			chip->pd_active = false;
			return;
		}
	}

	rc = smb235x_get_prop_from_tcpm(chip,
			POWER_SUPPLY_PROP_CURRENT_MAX, &pval);
	icl_ma = pval.intval / MICRO_TO_MILLI;

	rc = smb235x_set_icl_sw(chip, icl_ma);
	if (rc < 0)
		dev_err(chip->dev, "Failed to set current of type %d rc = %d\n",
				chip->charger_type, rc);
}

static void status_change_work(struct work_struct *work)
{
	struct smb235x_chg_chip *chip = container_of(work,
			struct smb235x_chg_chip, status_change_work);
	smb235x_tcpm_update_icl(chip);
}

static int smb235x_tcpm_notifier_cb(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct power_supply *psy = data;
	struct smb235x_chg_chip *chip = container_of(nb,
			struct smb235x_chg_chip, nb);

	if (event != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	if (work_pending(&chip->status_change_work))
		return NOTIFY_OK;

	if (strcmp(psy->desc->name, chip->tcpm_full_psy_name) == 0)
		schedule_work(&chip->status_change_work);

	return NOTIFY_OK;
}

static int smb235x_init_psy(struct smb235x_chg_chip *chip)
{
	int rc;

	strlcpy(chip->tcpm_full_psy_name, "tcpm-source-psy-",
			sizeof(chip->tcpm_full_psy_name));

	rc = smb235x_init_usb_psy(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to initialize the usb power supply\n");
		return rc;
	}

	rc = smb235x_init_battery_psy(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to initialize the battery power supply\n");
		return rc;
	}

	chip->bms_psy = power_supply_get_by_name("bms");
	if (!chip->bms_psy) {
		dev_dbg(chip->dev, "bms driver not enable\n");
		return 0;
	}

	scnprintf(chip->tcpm_full_psy_name, sizeof(chip->tcpm_full_psy_name),
			"%s%s", chip->tcpm_full_psy_name,
			chip->dt.tcpm_psy_name);
	chip->tcpm_psy = power_supply_get_by_name(chip->tcpm_full_psy_name);
	if (!chip->tcpm_psy) {
		dev_dbg(chip->dev, "tcpm driver not enable\n");
		return 0;
	}

	chip->nb.notifier_call = smb235x_tcpm_notifier_cb;
	rc = power_supply_reg_notifier(&chip->nb);
	if (rc < 0)
		dev_err(chip->dev, "Failed to register the psy notifier rc = %d\n", rc);

	return rc;
}

static void smb235x_update_fv_fcc(struct smb235x_chg_chip *chip)
{
	union power_supply_propval pval;
	int rc;

	rc = smb235x_get_prop_from_bms(chip,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	if (rc < 0) {
		return;
	} else {
		if (chip->fastchg_curr_ua != pval.intval) {
			rc = smb235x_set_fcc(chip, pval.intval);
			if (rc < 0)
				return;
			chip->fastchg_curr_ua = pval.intval;
		}
	}

	rc = smb235x_get_prop_from_bms(chip,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX, &pval);
	if (rc < 0) {
		return;
	} else {
		if (chip->float_volt_uv != pval.intval) {
			rc = smb235x_set_fv(chip, pval.intval);
			if (rc < 0)
				return;
			chip->float_volt_uv = pval.intval;
		}
	}
}

static void smb235x_update_soc(struct smb235x_chg_chip *chip)
{
	union power_supply_propval pval;
	int rc;
	u8 soc = 0;

	rc = smb235x_get_prop_from_bms(chip,
			POWER_SUPPLY_PROP_CAPACITY, &pval);
	if (rc < 0)
		return;

	soc = DIV_ROUND_CLOSEST(pval.intval * 255, 100);

	rc = regmap_write(chip->regmap,
			CHGR_STEP_CHG_SOC_VBATT_V_REG, soc);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to update SOC rc = %d\n", rc);
		return;
	}

	rc = regmap_update_bits(chip->regmap,
			CHGR_STEP_CHG_SOC_VBATT_V_UPDATE_REG,
			CHGR_STEP_SOC_VBATT_V_UPDATE_BIT,
			CHGR_STEP_SOC_VBATT_V_UPDATE_BIT);
	if (rc < 0)
		dev_err(chip->dev, "Failed to update soc or vbatt update register rc = %d\n", rc);
}

static void smb235x_update_work(struct work_struct *work)
{
	struct smb235x_chg_chip *chip = container_of(work,
			struct smb235x_chg_chip, smb235x_update_work.work);

	smb235x_update_soc(chip);
	smb235x_update_fv_fcc(chip);

	schedule_delayed_work(&chip->smb235x_update_work, msecs_to_jiffies(DELAY_WORK_TIME_MS));
}

static void smb235x_free_interrupts(struct smb235x_chg_chip *chip)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(smb235x_irqs); i++) {
		if (smb235x_irqs[i].irq > 0) {
			if (smb235x_irqs[i].wake)
				disable_irq_wake(smb235x_irqs[i].irq);
			disable_irq(smb235x_irqs[i].irq);
		}
	}
}

static void smb235x_set_initial_status(struct smb235x_chg_chip *chip)
{
	struct smb235x_irq_data irq_data = {chip, "usbin-src-change"};

	smb235x_get_chg_type(chip);
	smb235x_usb_source_change_irq_handler(0, &irq_data);

	schedule_delayed_work(&chip->smb235x_update_work, msecs_to_jiffies(20));
}

static int smb235x_probe(struct platform_device *pdev)
{
	struct smb235x_chg_chip *chip;
	int rc;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;

	chip->regmap = dev_get_regmap(chip->dev->parent, NULL);
	if (!chip->regmap) {
		dev_err(chip->dev, "parent regmap is missing\n");
		return -EINVAL;
	}

	INIT_DELAYED_WORK(&chip->smb235x_update_work, smb235x_update_work);
	INIT_WORK(&chip->status_change_work, status_change_work);
	mutex_init(&chip->hvdcp_update_voltage_lock);

	rc = smb235x_parse_dt(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to parse the devicetree, rc=%d\n", rc);
		return rc;
	}

	rc = smb235x_chg_init(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to initialize the charger HW setting, rc=%d\n", rc);
		return rc;
	}

	rc = smb235x_init_psy(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to initialize the power supply, rc=%d\n", rc);
		return rc;
	}

	rc = smb235x_request_interrupts(chip);
	if (rc < 0) {
		dev_err(chip->dev, "Failed to request the interrupt, rc=%d\n", rc);
		return rc;
	}

	smb235x_set_initial_status(chip);

	platform_set_drvdata(pdev, chip);

	dev_dbg(chip->dev, "smb235x charger driver probe successfully\n");

	return rc;
}

static int smb235x_remove(struct platform_device *pdev)
{
	struct smb235x_chg_chip *chip = platform_get_drvdata(pdev);

	smb235x_free_interrupts(chip);
	cancel_delayed_work_sync(&chip->smb235x_update_work);
	if (chip->bms_psy)
		power_supply_put(chip->bms_psy);
	if (chip->tcpm_psy)
		power_supply_put(chip->tcpm_psy);

	return 0;
}

static const struct of_device_id match_table[] = {
	{
		.compatible = "qcom,smb235x-charger",
	},
	{} /* sentinel */
};

static struct platform_driver smb235x_driver = {
	.driver = {
		.name = "qcom_smb235x",
		.of_match_table = match_table,
	},
	.probe = smb235x_probe,
	.remove = smb235x_remove,
};

module_platform_driver(smb235x_driver);

MODULE_DESCRIPTION("Qualcomm PMIC smb235x charger driver");
MODULE_LICENSE("GPL");
