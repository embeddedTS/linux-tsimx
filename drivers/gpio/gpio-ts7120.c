/*
 * Digital I/O driver for Technologic Systems TS-7120
 *
 * Copyright (C) 2019 Technologic Systems
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether expressed or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License version 2 for more details.
 */

#include <linux/gpio/driver.h>
#include <linux/of_device.h>
#include <linux/spinlock.h>
#include <linux/module.h>

#define TS7120_NR_DIO	 	12
#define TS7120_DIO_BASE  	160
#define SYSCON_ADDRESS		0x50004000
#define SYSCON_SIZE 			0x50
#define OUTPUT_SET_REG 0x10
#define OUTPUT_GET_REG 0x10
#define OUTPUT_ENABLE_SET_REG 0x12
#define OUTPUT_CLR_REG 0x14
#define OUTPUT_ENABLE_CLR_REG 0x16

struct TS7120_gpio_priv {
	void __iomem  *syscon;
	struct gpio_chip gpio_chip;
	spinlock_t lock;
	unsigned int direction[4];      /* enough for all 118 DIOs, 1=in, 0=out */
	unsigned int ovalue[4];
};


/*
	DIO controlled by the FPGA on the TS-7120:

		Header HD20...
		Pin 2 DIO_2
		Pin 4 DIO_4
		Pin 5 DIO_5
		.
		.
		Pin 15 DIO_15

	There's no DIO_0, DIO_1 or DIO_3 on this header, so that makes 13 total.
	This driver will accept only those DIO numbers that have a pin on the header.

	DIO is controlled by four 16-bit registers in the FPGA Syscon:

		Offset 0x10:  Data Set (write) or Pin State (read)
		Offset 0x12:  Output Enable Set
		Offset 0x14:  Data Clear
		Offset 0x16:  Output Enable Clear


*/

#define IS_VALID_OFFSET(x) ((x) < 12)

static inline struct TS7120_gpio_priv *to_gpio_TS7120(struct gpio_chip *chip)
{
	return container_of(chip, struct TS7120_gpio_priv, gpio_chip);
}

static int TS7120_gpio_get_direction(struct gpio_chip *chip,
					  unsigned int offset)
{
	struct TS7120_gpio_priv *priv = to_gpio_TS7120(chip);

	if (priv == NULL) {
		printk("%s %d, priv is NULL!\n", __func__, __LINE__);
		return -1;
	}

	if (priv->syscon == NULL) {
		  printk("%s %d, priv->syscon is NULL!\n", __func__, __LINE__);
		  return -1;
	}

	if (!IS_VALID_OFFSET(offset))
		return -EINVAL;

	return !!(priv->direction[offset / 32] & (1 << offset % 32));

}

static int TS7120_gpio_direction_input(struct gpio_chip *chip,
						 unsigned int offset)
{
	struct TS7120_gpio_priv *priv = to_gpio_TS7120(chip);
//	unsigned int reg; //, bit;
	unsigned long flags;

	if (priv == NULL) {
		printk("%s %d, priv is NULL!\n", __func__, __LINE__);
		return -1;
	}

	if (priv->syscon == NULL) {
		  printk("%s %d, priv->syscon is NULL!\n", __func__, __LINE__);
		  return -1;
	}

	if (!IS_VALID_OFFSET(offset))
		return -EINVAL;

	spin_lock_irqsave(&priv->lock, flags);

	priv->direction[offset / 32] |= (1 << offset % 32);

	writew((1 << offset), priv->syscon + OUTPUT_ENABLE_CLR_REG);

	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int TS7120_gpio_direction_output(struct gpio_chip *chip,
					unsigned int offset, int value)
{
	struct TS7120_gpio_priv *priv = to_gpio_TS7120(chip);
	unsigned long flags;
	int ret =0;

	if (!IS_VALID_OFFSET(offset)) {
		printk("offset %d is invalid\n", offset);
			 return -EINVAL;
	}

	if (priv == NULL) {
		printk("%s %d, priv is NULL!\n", __func__, __LINE__);
		return -1;
	}

	if (priv->syscon == NULL) {
		  printk("%s %d, priv->syscon is NULL!\n", __func__, __LINE__);
		  return -1;
	}

	spin_lock_irqsave(&priv->lock, flags);

	writew((1 << offset), priv->syscon + OUTPUT_ENABLE_SET_REG);

	if (value)
		writew((1 << offset), priv->syscon + OUTPUT_SET_REG);
	else
		writew((1 << offset), priv->syscon + OUTPUT_CLR_REG);

	priv->direction[offset / 32] &= ~(1 << offset % 32);
	spin_unlock_irqrestore(&priv->lock, flags);

	return ret;
}

static int TS7120_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct TS7120_gpio_priv *priv = to_gpio_TS7120(chip);
	uint16_t reg;

	if (!IS_VALID_OFFSET(offset))
		return -EINVAL;

	if (priv == NULL) {
		printk("%s %d, priv is NULL!\n", __func__, __LINE__);
		return -1;
	}

	if (priv->syscon == NULL) {
		  printk("%s %d, priv->syscon is NULL!\n", __func__, __LINE__);
		  return -1;
	}

	reg = readw(priv->syscon + 0x10);

	return !!(reg & (1 << offset));

}

static void TS7120_gpio_set(struct gpio_chip *chip, unsigned int offset,
				 int value)
{
	struct TS7120_gpio_priv *priv = to_gpio_TS7120(chip);
	unsigned long flags;

	if (priv == NULL) {
		printk("%s %d, priv is NULL!\n", __func__, __LINE__);
		return;
	}
	if (priv->syscon == NULL) {
		  printk("%s %d, priv->syscon is NULL!\n", __func__, __LINE__);
		  return;
	}

	if (!IS_VALID_OFFSET(offset))
		return;

	if ((priv->direction[offset / 32] & (1 << offset % 32))) {
		printk("DIO #%d is not an output\n", priv->gpio_chip.base + offset);
		return;
	}

	spin_lock_irqsave(&priv->lock, flags);

	if (value)
		writew((1 << offset), priv->syscon + OUTPUT_SET_REG);
	else
		writew((1 << offset), priv->syscon + OUTPUT_CLR_REG);

	spin_unlock_irqrestore(&priv->lock, flags);

}


static const struct gpio_chip template_chip = {
	.label			= "TS7120-gpio",
	.owner			= THIS_MODULE,
	.get_direction		= TS7120_gpio_get_direction,
	.direction_input	= TS7120_gpio_direction_input,
	.direction_output	= TS7120_gpio_direction_output,
	.get			= TS7120_gpio_get,
	.set			= TS7120_gpio_set,
	.base			= -1,
	.can_sleep		= false,
};

static const struct of_device_id TS7120_gpio_of_match_table[] = {
	{
		.compatible = "technologic,TS7120-gpio",
	},

	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, TS7120_gpio_of_match_table);

static int TS7120_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	struct TS7120_gpio_priv *priv;
	u32 ngpio, reg;
	int base;
	int ret;
	void __iomem  *membase;

	match = of_match_device(TS7120_gpio_of_match_table, dev);
	if (!match)
		return -EINVAL;

	if (of_property_read_u32(dev->of_node, "ngpios", &ngpio))
		ngpio = TS7120_NR_DIO;

	if (of_property_read_u32(dev->of_node, "base", &base))
		base = TS7120_DIO_BASE;

	membase = devm_ioremap(dev, SYSCON_ADDRESS, SYSCON_SIZE);

	if (IS_ERR(membase)) {
		pr_err("Could not map resource\n");
		return -ENOMEM;;
	}


	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->syscon = membase;

	reg = readl(priv->syscon);
	printk("FPGA: 0x%08X\n", reg);

	memset(priv->direction, 0xFF, sizeof(priv->direction));
	memset(priv->ovalue, 0, sizeof(priv->ovalue));
	/* Set all the DIO to inputs */

	writew(0xffff, priv->syscon + OUTPUT_ENABLE_CLR_REG);

	spin_lock_init(&priv->lock);
	priv->gpio_chip = template_chip;
	priv->gpio_chip.label = "TS7120-gpio";
	priv->gpio_chip.ngpio = ngpio;
	priv->gpio_chip.base = base;
	pdev->dev.platform_data = &priv;

	platform_set_drvdata(pdev, priv);

	ret = gpiochip_add(&priv->gpio_chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to register gpiochip\n");
		return ret;
	}

	return 0;
}

static int TS7120_gpio_remove(struct platform_device *pdev)
{
	struct TS7120_gpio_priv *priv = platform_get_drvdata(pdev);
	if (priv)
		gpiochip_remove(&priv->gpio_chip);
	return 0;
}

static struct platform_driver TS7120_gpio_driver = {
	.driver = {
		.name = "TS7120-gpio",
		.of_match_table = of_match_ptr(TS7120_gpio_of_match_table),
	},
	.probe = TS7120_gpio_probe,
	.remove = TS7120_gpio_remove,
};
module_platform_driver(TS7120_gpio_driver);

MODULE_AUTHOR("Technologic Systems");
MODULE_DESCRIPTION("GPIO interface for Technologic Systems TS-7120 DIO");
MODULE_LICENSE("GPL");
