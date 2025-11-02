// SPDX-License-Identifier: GPL-2.0-only
/*
 *  embeddedTS Wizard MFD I2C controller
 */

#include <linux/irqchip.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/fwnode.h>
#include <linux/mfd/ts_wizard.h>

#define MAX_IRQS	16

#define IRQ_STATUS      0
#define IRQ_ACK         1
#define IRQ_MASK_SET    2
#define IRQ_MASK_CLR    3
#define IRQ_MASK_RW     4

static const struct regmap_irq wizard_irqs[MAX_IRQS] = {
	REGMAP_IRQ_REG(0, 0, BIT(0)),
	REGMAP_IRQ_REG(1, 0, BIT(1)),
	REGMAP_IRQ_REG(2, 0, BIT(2)),
	REGMAP_IRQ_REG(3, 0, BIT(3)),
	REGMAP_IRQ_REG(4, 0, BIT(4)),
	REGMAP_IRQ_REG(5, 0, BIT(5)),
	REGMAP_IRQ_REG(6, 0, BIT(6)),
	REGMAP_IRQ_REG(7, 0, BIT(7)),
	REGMAP_IRQ_REG(8, 0, BIT(8)),
	REGMAP_IRQ_REG(9, 0, BIT(9)),
	REGMAP_IRQ_REG(10, 0, BIT(10)),
	REGMAP_IRQ_REG(11, 0, BIT(11)),
	REGMAP_IRQ_REG(12, 0, BIT(12)),
	REGMAP_IRQ_REG(13, 0, BIT(13)),
	REGMAP_IRQ_REG(14, 0, BIT(14)),
	REGMAP_IRQ_REG(15, 0, BIT(15)),
};

static const struct regmap_irq_chip wizard_regmap_ic = {
	.name           = "wizard-irq",
	.irqs           = wizard_irqs,
	.num_irqs       = ARRAY_SIZE(wizard_irqs),
	.num_regs       = 1,
	.irq_reg_stride = 1,

	.status_base    = WIZARD_IRQCHIP_BASE + IRQ_STATUS,
	.ack_base       = WIZARD_IRQCHIP_BASE + IRQ_ACK,
	.mask_base      = WIZARD_IRQCHIP_BASE + IRQ_MASK_RW,
};

static int wizard_irq_probe(struct platform_device *pdev)
{
	struct ts_wizard *wizard = dev_get_drvdata(pdev->dev.parent);
	struct device *dev   = &pdev->dev;
	struct regmap_irq_chip_data *ricd;
	int parent_irq;

	if (!wizard || !wizard->regmap)
		return -ENODEV;

	parent_irq = platform_get_irq(pdev, 0);
	if (parent_irq < 0)
		return parent_irq;

	return devm_regmap_add_irq_chip_fwnode(dev,
					       dev_fwnode(dev),
					       wizard->regmap,
					       parent_irq,
					       0,
					       0,
					       &wizard_regmap_ic,
					       &ricd);
}

static const struct of_device_id wizard_of_match[] = {
	{ .compatible = "technologic,wizard-irq", },
	{ }
};
MODULE_DEVICE_TABLE(of, wizard_of_match);

static struct platform_driver wizard_driver = {
	.driver = {
		.name   = "wizard-irq",
		.of_match_table = wizard_of_match,
	},
	.probe	= wizard_irq_probe,
};
module_platform_driver(wizard_driver);

MODULE_DESCRIPTION("embeddedTS wizard IRQ controller");
MODULE_AUTHOR("Mark Featherston <mark@embeddedts.com>");
MODULE_LICENSE("GPL");
