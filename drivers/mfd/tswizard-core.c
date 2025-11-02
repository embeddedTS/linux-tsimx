// SPDX-License-Identifier: GPL-2.0-only

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/regmap.h>
#include <linux/mfd/core.h>
#include <linux/of_device.h>
#include <linux/mfd/ts_wizard.h>

#define MODEL_TS_7250_V3 0x7250
#define MODEL_TS_9370    0x9370
#define MODEL_TS_9390    0x9390
#define MODEL_TS_4300    0x4300

static struct mfd_cell ts7250v3_devs[] = {
	{
		.name = "tswizard-reset",
		.of_compatible = "technologic,wizard-reset",
		.id = -1,
	},
	{
		.name = "tswizard-temp",
		.of_compatible = "technologic,wizard-temp",
		.id = -1,
	},
	{
		.name = "tswizard-adc",
		.of_compatible = "technologic,wizard-adc",
		.id = -1,
	}
};

static struct mfd_cell ts9370_devs[] = {
	{
		.name = "wizard-irq",
		.of_compatible = "technologic,wizard-irq",
		.id = -1,
	},
	{
		.name = "tswizard-reset",
		.of_compatible = "technologic,wizard-reset",
		.id = -1,
	},
	{
		.name = "wizard-silo",
		.of_compatible = "technologic,wizard-silo",
		.id = -1,
	},
	{
		.name = "tswizard-temp",
		.of_compatible = "technologic,wizard-temp",
		.id = -1,
	},
	{
		.name = "tswizard-adc",
		.of_compatible = "technologic,wizard-adc",
		.id = -1,
	},
};

const struct regmap_config ts_wizard_i2c_regmap = {
	.reg_bits = 16,
	.val_bits = 16,
	.can_multi_write = true,
	.reg_format_endian = REGMAP_ENDIAN_LITTLE,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.cache_type = REGCACHE_NONE,
};
EXPORT_SYMBOL_GPL(ts_wizard_i2c_regmap);

static ssize_t vbus_present_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct ts_wizard *wizard = dev_get_drvdata(dev);
	unsigned int reg;
	int ret;

	ret = regmap_read(wizard->regmap, WIZARD_INPUTS, &reg);
	if (ret)
		return ret;
	ret = sprintf(buf, "%d\n", !!(reg & INPUTS_USB_VBUS));
	return ret;
}
static DEVICE_ATTR_RO(vbus_present);

static ssize_t wake_en_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct ts_wizard *wizard = dev_get_drvdata(dev);
	unsigned int ctrl_reg = 0;
	bool en;
	int ret;

	ret = kstrtobool(buf, &en);
	if (ret)
		return ret;

	if (en)
		ctrl_reg |= FLG_WAKE_EN;

	ret = regmap_update_bits(wizard->regmap, WIZARD_FLAGS,
				 FLG_WAKE_EN,
				 ctrl_reg);

	return ret ? ret : count;
}

static ssize_t wake_en_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct ts_wizard *wizard = dev_get_drvdata(dev);
	unsigned int reg;
	int ret;

	ret = regmap_read(wizard->regmap, WIZARD_FLAGS, &reg);
	if (ret)
		return ret;
	ret = sprintf(buf, "%d\n", !!(reg & FLG_WAKE_EN));
	return ret;
}
static DEVICE_ATTR_RW(wake_en);

static ssize_t console_cfg_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct ts_wizard *wizard = dev_get_drvdata(dev);
	unsigned int ctrl_reg;
	int ret;

	if (sysfs_streq(buf, "auto"))
		ctrl_reg = 0;
	else if (sysfs_streq(buf, "always-usb"))
		ctrl_reg = FLG_FORCE_USB_CON;
	else
		return -EINVAL;

	ret = regmap_update_bits(wizard->regmap, WIZARD_FLAGS, FLG_FORCE_USB_CON,
				 ctrl_reg);

	return ret ? ret : count;
}

static ssize_t console_cfg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ts_wizard *wizard = dev_get_drvdata(dev);
	unsigned int reg;
	int ret;

	ret = regmap_read(wizard->regmap, WIZARD_FLAGS, &reg);
	if (ret)
		return ret;

	if (reg & FLG_FORCE_USB_CON)
		ret = sprintf(buf, "auto [always-usb]\n");
	else
		ret = sprintf(buf, "[auto] always-usb\n");

	return ret;
}
static DEVICE_ATTR_RW(console_cfg);

static struct attribute *ts7250v3_sysfs_entries[] = {
	&dev_attr_vbus_present.attr,
	&dev_attr_wake_en.attr,
	&dev_attr_console_cfg.attr,
	NULL,
};

static struct attribute_group ts7250v3_attr_group = {
	.attrs	= ts7250v3_sysfs_entries,
};

static struct attribute *ts9370_sysfs_entries[] = {
	&dev_attr_vbus_present.attr,
	//&dev_attr_wake_en.attr,
	NULL,
};

static struct attribute_group ts9370_attr_group = {
	.attrs	= ts9370_sysfs_entries,
};

static int ts_wizard_i2c_probe(struct i2c_client *client)
{
	struct ts_wizard *wizard;
	struct device *dev = &client->dev;
	int err = 0, i;
	uint32_t model, revision;

	wizard = devm_kzalloc(dev, sizeof(struct ts_wizard),
			     GFP_KERNEL);
	if (!wizard)
		return -ENOMEM;

	dev_set_drvdata(dev, wizard);

	wizard->regmap = devm_regmap_init_i2c(client, &ts_wizard_i2c_regmap);
	if (IS_ERR(wizard->regmap)) {
		err = PTR_ERR(wizard->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", err);
		return err;
	}

	err = regmap_read(wizard->regmap, WIZARD_MODEL, &model);
	if (err < 0)
		dev_err(dev, "error reading reg %u", WIZARD_MODEL);
	err = regmap_read(wizard->regmap, WIZARD_REV_INFO, &revision);
	if (err < 0)
		dev_err(dev, "error reading reg %u", WIZARD_REV_INFO);
	dev_info(&client->dev, "Model %04X rev %d%s\n",
		 model,
		 revision & 0x7fff,
		 revision & 0x8000 ? " (DIRTY)" : "");

	switch (model) {
	case MODEL_TS_7250_V3:
		err = sysfs_create_group(&dev->kobj, &ts7250v3_attr_group);
		if (err)
			dev_warn(dev, "error creating sysfs entries for the ts7250v3\n");
		break;

		/* Set up and register the platform devices. */
		for (i = 0; i < ARRAY_SIZE(ts7250v3_devs); i++) {
			ts7250v3_devs[i].platform_data = wizard;
			ts7250v3_devs[i].pdata_size = sizeof(struct ts_wizard);
		}

		return mfd_add_devices(dev, 0, ts7250v3_devs,
				ARRAY_SIZE(ts7250v3_devs), NULL, 0, NULL);
	case MODEL_TS_9370:
	case MODEL_TS_9390:
	case MODEL_TS_4300:
		err = sysfs_create_group(&dev->kobj, &ts9370_attr_group);
		if (err)
			dev_warn(dev, "error creating sysfs entries for the ts%04x\n", model);

		/* Set up and register the platform devices. */
		for (i = 0; i < ARRAY_SIZE(ts9370_devs); i++) {
			ts9370_devs[i].platform_data = wizard;
			ts9370_devs[i].pdata_size = sizeof(struct ts_wizard);
		}

		return mfd_add_devices(dev, 0, ts9370_devs,
				ARRAY_SIZE(ts9370_devs), NULL, 0, NULL);

		break;
	default:
		dev_warn(dev, "tswizard-core: unknown model: %04X\n", model);
		break;
	}
	return 0;
}

static const struct i2c_device_id ts_wizard_i2c_id[] = {
	{ "wizard", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, ts_wizard_i2c_id);

static const struct of_device_id ts_wizard_i2c_of_match[] = {
	{ .compatible = "technologic,wizard", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ts_wizard_i2c_of_match);

static struct i2c_driver ts_wizard_i2c_driver = {
	.driver = {
		.name = "tswizard-core",
		.of_match_table = of_match_ptr(ts_wizard_i2c_of_match),
	},
	.probe = ts_wizard_i2c_probe,
	.id_table = ts_wizard_i2c_id,
};
module_i2c_driver(ts_wizard_i2c_driver);

MODULE_AUTHOR("Mark Featherston <mark@embeddedts.com>");
MODULE_DESCRIPTION("MFD driver for embeddedTS Supervisory microcontroller");
MODULE_LICENSE("GPL v2");
