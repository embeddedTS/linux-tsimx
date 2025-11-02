// SPDX-License-Identifier: GPL-2.0-only
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/regmap.h>
#include <linux/mfd/core.h>
#include <linux/reboot.h>
#include <linux/mfd/ts_wizard.h>

/* We need a static device to support this for shutdown/reboot hooks */
static struct device *ts_rstc_device;
static atomic_t ts_restart_nb_refcnt = ATOMIC_INIT(0);

static ssize_t reboot_reason_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ts_wizard *wizard = dev_get_drvdata(dev);
	uint32_t reason;
	int len, err;

	err = regmap_read(wizard->regmap, WIZARD_REBOOT_REASON, &reason);
	if (err < 0)
		dev_err(dev, "error reading reg %u", WIZARD_REBOOT_REASON);

	switch (reason) {
	case REBOOT_REASON_POR:
		len = sprintf(buf, "POR\n");
		break;
	case REBOOT_REASON_CPU_WDT:
		len = sprintf(buf, "CPU WDT\n");
		break;
	case REBOOT_REASON_SOFTWARE_REBOOT:
		len = sprintf(buf, "Software Reboot\n");
		break;
	case REBOOT_REASON_BROWNOUT:
		len = sprintf(buf, "Brownout\n");
		break;
	case REBOOT_REASON_RTC_ALARM_REBOOT:
		len = sprintf(buf, "RTC Alarm Reboot\n");
		break;
	case REBOOT_REASON_WAKE_FROM_PWR_CYCLE:
		len = sprintf(buf, "Wake from PWR Cycle\n");
		break;
	case REBOOT_REASON_WAKE_FROM_WAKE_SIGNAL:
		len = sprintf(buf, "Wake from WAKE_EN\n");
		break;
	case REBOOT_REASON_WAKE_FROM_RTC_ALARM:
		len = sprintf(buf, "Wake from RTC Alarm\n");
		break;
	case REBOOT_REASON_WAKE_FROM_USB_VBUS:
		len = sprintf(buf, "Wake from USB VBUS\n");
		break;
	default:
		len = sprintf(buf, "Unknown\n");
		break;
	}

	return len;
}
static DEVICE_ATTR_RO(reboot_reason);

static struct attribute *ts_wizard_sysfs_entries[] = {
	&dev_attr_reboot_reason.attr,
	NULL,
};

static struct attribute_group ts_wizard_attr_group = {
	.attrs	= ts_wizard_sysfs_entries,
};

static int ts_wizard_restart(struct sys_off_data *data)
{
	struct ts_wizard *wizard = data->cb_data;
	int err = -ENOENT;

	if (wizard) {
		err = regmap_write(wizard->regmap, WIZARD_CMDS, I2C_REBOOT);
		if (!err)
			mdelay(1000);
	}

	dev_emerg(ts_rstc_device, "reset controller could not cause a reset!");

	return NOTIFY_DONE;
}

static int ts_wizard_do_poweroff(struct sys_off_data *data)
{
	struct ts_wizard *wizard = data->cb_data;
	int err = -ENOENT;

	if (wizard) {
		err = regmap_write(wizard->regmap, WIZARD_CMDS, I2C_HALT);
		if (!err)
			mdelay(1000);
	}

	dev_emerg(ts_rstc_device, "Unable to call halt (%d)", err);
	return NOTIFY_DONE;
}

static int ts_wizard_rstc_probe(struct platform_device *pdev)
{
	struct ts_wizard *wizard = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	int err = 0;

	dev_set_drvdata(dev, wizard);
	if (atomic_inc_return(&ts_restart_nb_refcnt) == 1) {
		ts_rstc_device = dev;
		err = devm_register_sys_off_handler(dev,
						    SYS_OFF_MODE_POWER_OFF_PREPARE,
						    SYS_OFF_PRIO_DEFAULT,
						    ts_wizard_do_poweroff,
							wizard);
		if (err) {
			dev_err(dev, "cannot register sys off handler (err=%d)\n", err);
			return err;
		}

		err = devm_register_restart_handler(dev, ts_wizard_restart, wizard);
		if (err) {
			dev_err(dev, "cannot register restart handler (err=%d)\n", err);
			atomic_dec(&ts_restart_nb_refcnt);
			return err;
		}
	} else {
		err = EEXIST;
		dev_err(dev, "rstc already registered");
	}

	err = sysfs_create_group(&dev->kobj, &ts_wizard_attr_group);
	if (err)
		dev_warn(dev, "error creating sysfs entries\n");

	dev_info(dev, "Using wizard for reset controller");

	return 0;
}

static struct platform_driver tswizard_rstc_driver = {
	.driver = {
		.name = "tswizard-reset",
	},
	.probe = ts_wizard_rstc_probe,
};

module_platform_driver(tswizard_rstc_driver);

MODULE_DESCRIPTION("embeddedTS wizard reset controller driver");
MODULE_AUTHOR("Mark Featherston <mark@embeddedts.com>");
MODULE_LICENSE("GPL");
