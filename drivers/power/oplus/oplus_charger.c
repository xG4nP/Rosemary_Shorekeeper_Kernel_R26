// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/of_gpio.h>
#include <linux/kthread.h>
#include <linux/version.h>
#include <linux/reboot.h>
#include <linux/module.h>
#include <linux/sched/clock.h>
#include <soc/oplus/system/oplus_project.h>

#ifdef CONFIG_OPLUS_CHARGER_MTK

//#include <mtk_boot_common.h>
#include <mt-plat/mtk_boot.h>
#include <linux/gpio.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0))
#include <uapi/linux/sched/types.h>
#endif
#else /* CONFIG_OPLUS_CHARGER_MTK */
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/of.h>

#include <linux/bitops.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/spmi.h>
#include <linux/printk.h>
#include <linux/ratelimit.h>
#include <linux/debugfs.h>
#include <linux/leds.h>
#include <linux/rtc.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0))
#include <linux/qpnp/qpnp-adc.h>
#else
#include <uapi/linux/sched/types.h>
#endif
#include <linux/batterydata-lib.h>
#include <linux/of_batterydata.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0))
#include <linux/msm_bcl.h>
#endif
#include <linux/ktime.h>
#include <linux/kernel.h>
#endif

#include "oplus_charger.h"
#include "oplus_gauge.h"
#include "oplus_vooc.h"
#include "oplus_short.h"
#include "oplus_adapter.h"
#include "charger_ic/oplus_short_ic.h"
#include "oplus_debug_info.h"
#ifndef CONFIG_OPLUS_CHARGER_MTK
#ifndef WPC_NEW_INTERFACE
#include "oplus_wireless.h"
#include "wireless_ic/oplus_chargepump.h"	//for WPC
#else
#include "oplus_wireless.h"
#endif
#endif
static struct oplus_chg_chip *g_charger_chip = NULL;

#define MAX_UI_DECIMAL_TIME 24
#define UPDATE_TIME 1

#define OPLUS_CHG_UPDATE_INTERVAL_SEC 		5
/* first run after init 10s */
#define OPLUS_CHG_UPDATE_INIT_DELAY	round_jiffies_relative(msecs_to_jiffies(500))
/* update cycle 5s */
#define OPLUS_CHG_UPDATE_INTERVAL	round_jiffies_relative(msecs_to_jiffies(OPLUS_CHG_UPDATE_INTERVAL_SEC*1000))


#define OPLUS_CHG_DEFAULT_CHARGING_CURRENT	512

int enable_charger_log = 0;
int charger_abnormal_log = 0;
int tbatt_pwroff_enable = 1;
extern bool oplus_is_power_off_charging(struct oplus_vooc_chip *chip);
bool check_fastchg_quit = false;

/* wenbin.liu@SW.Bsp.Driver, 2016/03/01  Add for log tag*/
#define charger_xlog_printk(num, fmt, ...) \
		do { \
			if (enable_charger_log >= (int)num) { \
				pr_debug(KERN_NOTICE pr_fmt("[OPLUS_CHG][%s]"fmt), __func__, ##__VA_ARGS__); \
			} \
		} while (0)

void oplus_chg_turn_off_charging(struct oplus_chg_chip *chip);
void oplus_chg_turn_on_charging(struct oplus_chg_chip *chip);

static void oplus_chg_smooth_to_soc(struct oplus_chg_chip *chip);
static void oplus_chg_variables_init(struct oplus_chg_chip *chip);
static void oplus_chg_update_work(struct work_struct *work);
static void oplus_chg_reset_adapter_work(struct work_struct *work);
static void oppo_fastchg_check_work(struct work_struct *work);
static void oplus_chg_protection_check(struct oplus_chg_chip *chip);
static void oplus_chg_get_battery_data(struct oplus_chg_chip *chip);
static void oplus_chg_check_tbatt_status(struct oplus_chg_chip *chip);
static void oplus_chg_check_tbatt_normal_status(struct oplus_chg_chip *chip);
static void oplus_chg_get_chargerid_voltage(struct oplus_chg_chip *chip);
void oplus_chg_set_input_current_limit(struct oplus_chg_chip *chip);
static void oplus_chg_set_charging_current(struct oplus_chg_chip *chip);
static void oplus_chg_battery_update_status(struct oplus_chg_chip *chip);
static void oplus_chg_pdqc_to_normal(struct oplus_chg_chip *chip);
static void oplus_get_smooth_soc_switch(struct oplus_chg_chip *chip);
static void oplus_chg_pd_config(struct oplus_chg_chip *chip);
static void oplus_chg_qc_config(struct oplus_chg_chip *chip);
#ifdef  CONFIG_FB
static int fb_notifier_callback(struct notifier_block *nb, unsigned long event, void *data);
#endif
void oplus_chg_ui_soc_decimal_init(void);
void oplus_chg_ui_soc_decimal_deinit(void);

static void oplus_chg_show_ui_soc_decimal(struct work_struct *work);


static int chgr_dbg_vchg = 0;
module_param(chgr_dbg_vchg, int, 0644);
MODULE_PARM_DESC(chgr_dbg_vchg, "debug charger voltage");

static int chgr_dbg_total_time = 0;
module_param(chgr_dbg_total_time, int, 0644);
MODULE_PARM_DESC(chgr_dbg_total_time, "debug charger total time");

/****************************************/
static int reset_mcu_delay = 0;
static bool suspend_charger = false;
static bool vbatt_higherthan_4180mv = false;
static bool vbatt_lowerthan_3300mv = false;

enum power_supply_property oplus_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_OTG_SWITCH,
	POWER_SUPPLY_PROP_OTG_ONLINE,
};

enum power_supply_property oplus_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
#ifdef CONFIG_OPLUS_FAST2NORMAL_CHG
	POWER_SUPPLY_PROP_FAST2NORMAL_CHG,
#endif
};

enum power_supply_property oplus_batt_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_AUTHENTICATE,
	POWER_SUPPLY_PROP_CHARGE_TIMEOUT,
	POWER_SUPPLY_PROP_CHARGE_TECHNOLOGY,
	POWER_SUPPLY_PROP_FAST_CHARGE,
	POWER_SUPPLY_PROP_MMI_CHARGING_ENABLE,	/*add for MMI_CHG_TEST*/
#ifdef CONFIG_OPLUS_CHARGER_MTK
	POWER_SUPPLY_PROP_STOP_CHARGING_ENABLE,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CURRENT_MAX,
#endif
#ifndef CONFIG_OPLUS_SDM670_CHARGER
	POWER_SUPPLY_PROP_CHARGE_FULL,
#endif
	POWER_SUPPLY_PROP_BATTERY_FCC,
	POWER_SUPPLY_PROP_BATTERY_SOH,
	POWER_SUPPLY_PROP_BATTERY_CC,
	POWER_SUPPLY_PROP_BATTERY_RM,
	POWER_SUPPLY_PROP_BATTERY_NOTIFY_CODE,
#ifdef CONFIG_OPLUS_SMART_CHARGER_SUPPORT
	POWER_SUPPLY_PROP_COOL_DOWN,
#endif
	POWER_SUPPLY_PROP_ADAPTER_FW_UPDATE,
	POWER_SUPPLY_PROP_VOOCCHG_ING,
#ifdef CONFIG_OPLUS_CHECK_CHARGERID_VOLT
	POWER_SUPPLY_PROP_CHARGERID_VOLT,
#endif
#ifdef CONFIG_OPLUS_SHIP_MODE_SUPPORT
	POWER_SUPPLY_PROP_SHIP_MODE,
#endif
#ifdef CONFIG_OPLUS_CALL_MODE_SUPPORT
	POWER_SUPPLY_PROP_CALL_MODE,
#endif
#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
#ifdef CONFIG_OPLUS_SHORT_USERSPACE
	POWER_SUPPLY_PROP_SHORT_C_LIMIT_CHG,
	POWER_SUPPLY_PROP_SHORT_C_LIMIT_RECHG,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED,
#else
	POWER_SUPPLY_PROP_SHORT_C_BATT_UPDATE_CHANGE,
	POWER_SUPPLY_PROP_SHORT_C_BATT_IN_IDLE,
	POWER_SUPPLY_PROP_SHORT_C_BATT_CV_STATUS,
#endif /*CONFIG_OPLUS_SHORT_USERSPACE*/
#endif
#ifdef CONFIG_OPLUS_SHORT_HW_CHECK
	POWER_SUPPLY_PROP_SHORT_C_HW_FEATURE,
	POWER_SUPPLY_PROP_SHORT_C_HW_STATUS,
#endif
#ifdef CONFIG_OPLUS_SHORT_IC_CHECK
	POWER_SUPPLY_PROP_SHORT_C_IC_OTP_STATUS,
	POWER_SUPPLY_PROP_SHORT_C_IC_VOLT_THRESH,
	POWER_SUPPLY_PROP_SHORT_C_IC_OTP_VALUE,
#endif
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
};

#ifdef CONFIG_OPLUS_CHARGER_MTK
int oplus_usb_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
		{
	int ret = 0;

	//struct oplus_chg_chip *chip = container_of(psy->desc, struct oplus_chg_chip, usb_psd);
	struct oplus_chg_chip *chip = g_charger_chip;

	if (chip->charger_exist) {
		if ((chip->charger_type == POWER_SUPPLY_TYPE_USB
				|| chip->charger_type == POWER_SUPPLY_TYPE_USB_CDP)
				&& chip->stop_chg == 1) {
			chip->usb_online = true;
			chip->usb_psd.type = POWER_SUPPLY_TYPE_USB;
		}
	} else {
		chip->usb_online = false;
	}

	switch (psp) {
		case POWER_SUPPLY_PROP_CURRENT_MAX:
			val->intval = 500000;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MAX:
			val->intval = 5000000;
			break;
		case POWER_SUPPLY_PROP_ONLINE:
			val->intval = chip->usb_online;
			break;
		case POWER_SUPPLY_PROP_OTG_SWITCH:
			val->intval = chip->otg_switch;
			break;
		case POWER_SUPPLY_PROP_OTG_ONLINE:
			val->intval = chip->otg_online;
			break;
		default:
			pr_err("get prop %d is not supported in usb\n", psp);
			ret = -EINVAL;
			break;
	}
	return ret;
}

int oplus_usb_property_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int ret = 0;
	switch (psp) {
		case POWER_SUPPLY_PROP_OTG_SWITCH:
			return 1;
		default:
			pr_err("writeable prop %d is not supported in usb\n", psp);
			ret = -EINVAL;
			break;
	}
	return 0;
}

void __attribute__((weak)) oplus_set_otg_switch_status(bool value)
{
	return;
}

int oplus_usb_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	int ret = 0;
	//struct oplus_chg_chip *chip = container_of(psy->desc, struct oplus_chg_chip, usb_psd);
	struct oplus_chg_chip *chip = g_charger_chip;

	switch (psp) {
		case POWER_SUPPLY_PROP_OTG_SWITCH:
			if (val->intval == 1) {
				chip->otg_switch = true;
				oplus_set_otg_switch_status(true);
			} else {
				chip->otg_switch = false;
				chip->otg_online = false;
				oplus_set_otg_switch_status(false);
			}
			charger_xlog_printk(CHG_LOG_CRTI, "otg_switch: %d\n", chip->otg_switch);
			break;
		default:
			pr_err("set prop %d is not supported in usb\n", psp);
			ret = -EINVAL;
			break;
	}
	return ret;
}

static void usb_update(struct oplus_chg_chip *chip)
{
	if (chip->charger_exist) {
		/*if (chip->charger_type==STANDARD_HOST || chip->charger_type==CHARGING_HOST) {*/
		if (chip->charger_type == POWER_SUPPLY_TYPE_USB
				|| chip->charger_type == POWER_SUPPLY_TYPE_USB_CDP) {
			chip->usb_online = true;
			chip->usb_psd.type = POWER_SUPPLY_TYPE_USB;
		}
	} else {
		chip->usb_online = false;
	}
	power_supply_changed(chip->usb_psy);
}
#endif
static void fastchgquit_check(void)
{
	struct oplus_chg_chip *chip = g_charger_chip;
	if (!chip) {
		chg_err("g_oppo_chip is NULL\n");
		return;
	}

	if((!chip->charger_exist) && chip->ac_online && (chip->chg_ops->get_charger_volt() < 2500) && check_fastchg_quit){
		chg_err("charger_exist null, turn off fastchg\n");
		//if(get_reset_gpio_value()) {
			chg_err("pull down mcu_en\n");
			oplus_vooc_turn_off_fastchg();
		//}
	}
	check_fastchg_quit = false;
}

int oplus_ac_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int ret = 0;
	//struct oplus_chg_chip *chip = container_of(psy->desc, struct oplus_chg_chip, ac_psd);
	struct oplus_chg_chip *chip = g_charger_chip;

	if (chip->charger_exist) {
		if ((chip->charger_type == POWER_SUPPLY_TYPE_USB_DCP || suspend_charger)
				|| (oplus_vooc_get_fastchg_started() == true)
				|| (oplus_vooc_get_fastchg_to_normal() == true)
				|| (oplus_vooc_get_fastchg_to_warm() == true)
				|| (oplus_vooc_get_fastchg_dummy_started() == true)
				|| (oplus_vooc_get_adapter_update_status() == ADAPTER_FW_NEED_UPDATE)
				|| (oplus_vooc_get_btb_temp_over() == true)) {
			chip->ac_online = true;
		} else {
			chip->ac_online = false;
		}
	} else {
		if ((oplus_vooc_get_fastchg_started() == true)
				|| (oplus_vooc_get_fastchg_to_normal() == true)
				|| (oplus_vooc_get_fastchg_to_warm() == true)
				|| (oplus_vooc_get_fastchg_dummy_started() == true)
				|| (oplus_vooc_get_adapter_update_status() == ADAPTER_FW_NEED_UPDATE)
				|| (oplus_vooc_get_btb_temp_over() == true)
				|| chip->mmi_fastchg == 0) {
			chip->ac_online = true;
		} else {
			chip->ac_online = false;

		}
	}
	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			val->intval = chip->ac_online;
			break;
#ifdef CONFIG_OPLUS_FAST2NORMAL_CHG
		case POWER_SUPPLY_PROP_FAST2NORMAL_CHG:
			if (oplus_vooc_get_fastchg_to_normal() == true
					|| oplus_vooc_get_fastchg_to_warm() == true
					|| oplus_vooc_get_btb_temp_over() == true
					|| oplus_vooc_get_fastchg_low_temp_full() == true) {
				val->intval = 1;
			} else {
				val->intval = 0;
			}
			break;
#endif
		default:
			pr_err("get prop %d is not supported in ac\n", psp);
			ret = -EINVAL;
			break;
	}
	if (chip->ac_online) {
		charger_xlog_printk(CHG_LOG_CRTI, "chg_exist:%d, ac_online:%d\n",
				chip->charger_exist, chip->ac_online);
	}
	if((!chip->charger_exist) && chip->ac_online ){
		if (!check_fastchg_quit){
			check_fastchg_quit = true;
			schedule_delayed_work(&chip->fastcheck_work, OPLUS_CHG_UPDATE_INTERVAL);
		} else {
			check_fastchg_quit = false;
		}
	}
	return ret;
}


int oplus_battery_property_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int rc = 0;

	switch (psp) {
		case POWER_SUPPLY_PROP_MMI_CHARGING_ENABLE:
			rc = 1;
			break;
#ifdef CONFIG_OPLUS_SMOOTH_SOC
		case POWER_SUPPLY_PROP_SMOOTH_SWITCH:
			rc = 1;
			break;
#endif
#ifdef CONFIG_OPLUS_SMART_CHARGER_SUPPORT
		case POWER_SUPPLY_PROP_COOL_DOWN:
			rc = 1;
			break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			if (g_charger_chip && g_charger_chip->smart_charging_screenoff) {
				rc = 1;
			} else {
				rc = 0;
			}
			break;
#endif
#ifdef CONFIG_OPLUS_CHARGER_MTK
		case POWER_SUPPLY_PROP_STOP_CHARGING_ENABLE:
			rc = 1;
			break;
#endif
#ifdef CONFIG_OPLUS_SHIP_MODE_SUPPORT
		case POWER_SUPPLY_PROP_SHIP_MODE:
			rc = 1;
			break;
#endif
#ifdef CONFIG_OPLUS_CALL_MODE_SUPPORT
		case POWER_SUPPLY_PROP_CALL_MODE:
			rc = 1;
			break;
#endif
#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
#ifdef CONFIG_OPLUS_SHORT_USERSPACE
		case POWER_SUPPLY_PROP_SHORT_C_LIMIT_CHG:
		case POWER_SUPPLY_PROP_SHORT_C_LIMIT_RECHG:
#else
		case POWER_SUPPLY_PROP_SHORT_C_BATT_UPDATE_CHANGE:
		case POWER_SUPPLY_PROP_SHORT_C_BATT_IN_IDLE:
#endif /*CONFIG_OPLUS_SHORT_USERSPACE*/
			rc = 1;
			break;
#endif
#ifdef CONFIG_OPLUS_SHORT_HW_CHECK
		case POWER_SUPPLY_PROP_SHORT_C_HW_FEATURE:
			rc = 1;
			break;
#endif
#ifdef CONFIG_OPLUS_SHORT_IC_CHECK
		case POWER_SUPPLY_PROP_SHORT_C_IC_VOLT_THRESH:
			rc = 1;
			break;
#endif
#ifdef CONFIG_OPLUS_CHIP_SOC_NODE
		case POWER_SUPPLY_PROP_CHIP_SOC:
			rc = 1;
			break;
#endif
//		pr_err("writeable prop %d is not supported in batt\n", psp);
		default:
			rc = 0;
			break;
	}
	return rc;
}

int oplus_battery_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	int ret = 0;
	//struct oplus_chg_chip *chip = container_of(psy->desc, struct oplus_chg_chip, battery_psd);
	struct oplus_chg_chip *chip = g_charger_chip;

	switch (psp) {
		case POWER_SUPPLY_PROP_MMI_CHARGING_ENABLE:
			charger_xlog_printk(CHG_LOG_CRTI, "set mmi_chg = [%d].\n", val->intval);
			if (val->intval == 0) {
				if(chip->unwakelock_chg == 1) {
					ret = -EINVAL;
					charger_xlog_printk(CHG_LOG_CRTI,
							"unwakelock testing , this test not allowed.\n");
				} else {
					chip->prop_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
					chip->mmi_chg = 0;
					oplus_chg_turn_off_charging(chip);
					if (oplus_vooc_get_fastchg_started() == true) {
						oplus_vooc_turn_off_fastchg();
						chip->mmi_fastchg = 0;
					}
				}
			} else {
				if(chip->unwakelock_chg == 1) {
					ret = -EINVAL;
					charger_xlog_printk(CHG_LOG_CRTI,
							"unwakelock testing , this test not allowed.\n");
				} else {
					chip->mmi_chg = 1;
					chip->prop_status = POWER_SUPPLY_STATUS_CHARGING;
					if (chip->mmi_fastchg == 0) {
						oplus_chg_clear_chargerid_info();
					}
					chip->mmi_fastchg = 1;
					oplus_chg_turn_on_charging(chip);
				}
			}
			break;
#ifdef CONFIG_OPLUS_SMOOTH_SOC
		case POWER_SUPPLY_PROP_SMOOTH_SWITCH:
			chip->smooth_switch = val->intval;
			break;
#endif
#ifdef CONFIG_OPLUS_SMART_CHARGER_SUPPORT
		case POWER_SUPPLY_PROP_COOL_DOWN:
			oplus_smart_charge_by_cool_down(chip, val->intval);
			break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			if (chip->smart_charging_screenoff) {
				oplus_smart_charge_by_shell_temp(chip, val->intval);
				break;
			} else {
				ret = -EINVAL;
				break;
			}
#endif
#ifdef CONFIG_OPLUS_CHARGER_MTK
		case POWER_SUPPLY_PROP_STOP_CHARGING_ENABLE:
			charger_xlog_printk(CHG_LOG_CRTI, "set stop_chg = [%d].\n", val->intval);
			if (val->intval == 0) {
				chip->stop_chg = 0;
			} else {
				chip->stop_chg = 1;
			}
		break;
#endif
#ifdef CONFIG_OPLUS_SHIP_MODE_SUPPORT
		case POWER_SUPPLY_PROP_SHIP_MODE:
			chip->enable_shipmode = val->intval;
			oplus_gauge_update_soc_smooth_parameter();
			break;
#endif
#ifdef CONFIG_OPLUS_CALL_MODE_SUPPORT
		case POWER_SUPPLY_PROP_CALL_MODE:
			chip->calling_on = val->intval;
			break;
#endif
#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
#ifdef CONFIG_OPLUS_SHORT_USERSPACE
		case POWER_SUPPLY_PROP_SHORT_C_LIMIT_CHG:
			printk(KERN_ERR "[OPLUS_CHG] [short_c_bat] set limit chg[%d]\n", !!val->intval);
			chip->short_c_batt.limit_chg = !!val->intval;
			//for userspace logic
			if (!!val->intval == 0){
				chip->short_c_batt.is_switch_on = 0;
			}
			break;
		case POWER_SUPPLY_PROP_SHORT_C_LIMIT_RECHG:
			printk(KERN_ERR "[OPLUS_CHG] [short_c_bat] set limit rechg[%d]\n", !!val->intval);
			chip->short_c_batt.limit_rechg = !!val->intval;
			break;
#else
		case POWER_SUPPLY_PROP_SHORT_C_BATT_UPDATE_CHANGE:
			printk(KERN_ERR "[OPLUS_CHG] [short_c_batt]: set update change[%d]\n", val->intval);
			oplus_short_c_batt_update_change(chip, val->intval);
			chip->short_c_batt.update_change = val->intval;
			break;

		case POWER_SUPPLY_PROP_SHORT_C_BATT_IN_IDLE:
			printk(KERN_ERR "[OPLUS_CHG] [short_c_batt]: set in idle[%d]\n", !!val->intval);
			chip->short_c_batt.in_idle = !!val->intval;
			break;
#endif /*CONFIG_OPLUS_SHORT_USERSPACE*/
#endif /* CONFIG_OPLUS_SHORT_C_BATT_CHECK */
#ifdef CONFIG_OPLUS_SHORT_HW_CHECK
		case POWER_SUPPLY_PROP_SHORT_C_HW_FEATURE:
			printk(KERN_ERR "[OPLUS_CHG] [short_c_hw_check]: set is_feature_hw_on [%d]\n", val->intval);
			chip->short_c_batt.is_feature_hw_on = val->intval;
			break;
#endif /* CONFIG_OPLUS_SHORT_C_BATT_CHECK */
#ifdef CONFIG_OPLUS_SHORT_IC_CHECK
		case POWER_SUPPLY_PROP_SHORT_C_IC_VOLT_THRESH:
			if (chip) {
				chip->short_c_batt.ic_volt_threshold = val->intval;
				oplus_short_ic_set_volt_threshold(chip);
				//pr_err("%s:[OPLUS_CHG][oplus_short_ic],ic_volt_threshold val->intval[%d]\n", __FUNCTION__, val->intval);
			}
			break;
#endif
		default:
			//pr_err("set prop %d is not supported in batt\n", psp);
			ret = -EINVAL;
			break;
	}
	return ret;
}


#define OPLUS_MIDAS_CHG_DEBUG 0
#ifdef OPLUS_MIDAS_CHG_DEBUG
#define	midas_debug(fmt, args...)	\
	pr_debug("[OPLUS_MIDAS_CHG_DEBUG]" fmt, ##args)
#else
#define	midas_debug(fmt, args...)
#endif /* OPLUS_MIDAS_CHG_DEBUG */

static struct oplus_midas_chg {
	int cali_passed_chg;
	int passed_chg;
	int accu_delta;

	int prev_chg_stat;	/* 1--charger-on, 0 -- otherwise */

	unsigned int reset_counts;
} midas_chg;

static void oplus_midas_chg_info(const char *name)
{
	midas_debug("%s: passedchg=%d, realpassedchg=%d,"
		"accu_delta=%d, prev_chg_stat=%d, reset_counts=%u\n",
		name, midas_chg.cali_passed_chg, midas_chg.passed_chg,
		midas_chg.accu_delta, midas_chg.prev_chg_stat, midas_chg.reset_counts);
}

/* TODO: how to determine passedchg is reset precisely ? */
#define	__abs(a, b) ((a > b) ? (a - b) : (b - a))
#define ZEROTH	5
#define COMSUMETH 10
static bool oplus_midas_passedchg_reset(int prev, int val)
{
	if(__abs(val, prev) > COMSUMETH) {
		return true;
	} else if (__abs(val, 0) > ZEROTH) {
		return false;
	}

	if(prev < 0 && val - prev >= ZEROTH) {
			return true;
	}
	if(prev >= 0 && val < prev) {
			return true;
	}
	return false;
}

static void oplus_midas_chg_data_init(void)
{
	int val, ret;
	midas_chg.accu_delta = 0;
	midas_chg.reset_counts = 0;
	if(oplus_vooc_get_allow_reading() == true) {
		ret = oplus_gauge_get_passedchg(&val);
		if (ret) {
			pr_err("%s: get passedchg error %d\n", __FUNCTION__, val);
			midas_chg.cali_passed_chg = midas_chg.passed_chg = 0;
		} else {
			midas_chg.cali_passed_chg = midas_chg.passed_chg = val;
		}
	} else {
		pr_err("%s: not allow reading", __
