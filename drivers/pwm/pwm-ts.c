// SPDX-License-Identifier: GPL-2.0
/*
 * PWM for embeddedTS TS-7250-V3, TS-7120, et al.
 * Copyright (C) 2021-2022 Technologic Systems, Inc. dba embeddedTS
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/slab.h>

/*
 * These register addresses are a regmap mapping. The actual registers are different
 * depending on if the host system has a 16-bit data bus or 32-bit.
 */
#define REG_CONFIG		0
/* Enable PWM Output */
#define REG_CONFIG_ENABLED		(1 << 0)
/* 0 = idle high, active low.  1 = idle low, active high */
#define REG_CONFIG_INVERSED		(1 << 1)
#define REG_PERIOD		1
#define REG_DUTY		2
#define REG_SHIFT		3

#define CYCLE_MASK		0x3ff
#define SHIFT_MAX		12

struct ts_pwm {
	struct regmap *map;
	struct clk *clk;
};

static inline struct ts_pwm *to_ts_pwm(struct pwm_chip *chip)
{
	return pwmchip_get_drvdata(chip);
}

static int ts_pwm_calc(struct pwm_chip *chip,
		       unsigned int duty,
		       unsigned int period)
{
	struct ts_pwm *ts = to_ts_pwm(chip);
	unsigned long clk_rate = clk_get_rate(ts->clk);
	unsigned long long cycle;
	unsigned int  cnt, duty_cnt;
	u16 duty_reg;
	u8 shift;

	/* Calc shift & period reg */
	for (shift = 0; shift < SHIFT_MAX; shift++) {
		cycle = DIV_ROUND_CLOSEST_ULL(NSEC_PER_SEC,
					      (clk_rate / 100) >> shift);
		cnt = DIV_ROUND_CLOSEST(period * 100, (unsigned int)cycle);
		if (cnt <= CYCLE_MASK)
			break;
	}

	if (cnt > CYCLE_MASK)
		return -EINVAL;

	dev_dbg(&chip->dev, "cycle=%llu shift=%u cnt=%u\n",
		cycle, shift, cnt);

	if (duty == period) {
		duty_reg = cnt;
	} else if (duty == 0) {
		duty_reg = 0;
	} else {
		duty_cnt = DIV_ROUND_CLOSEST(duty * 100, (unsigned int)cycle);
		if (duty_cnt > CYCLE_MASK) {
			dev_err(&chip->dev, "unable to get duty cycle\n");
			return -EINVAL;
		}

		dev_dbg(&chip->dev, "shift=%u cnt=%u duty_cnt=%u\n",
			shift, cnt, duty_cnt);
		duty_reg = cnt - duty_cnt;
	}

	regmap_write(ts->map, REG_PERIOD, cnt);
	regmap_write(ts->map, REG_DUTY, duty_reg);
	regmap_write(ts->map, REG_SHIFT, shift);

	return 0;
}

static int ts_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			const struct pwm_state *state)
{
	struct ts_pwm *ts = to_ts_pwm(chip);
	int err;
	u16 ctrl = 0;

	if (state->polarity != PWM_POLARITY_NORMAL)
		ctrl |= REG_CONFIG_INVERSED;

	if (state->enabled)
		ctrl |= REG_CONFIG_ENABLED;

	err = ts_pwm_calc(chip, state->duty_cycle, state->period);
	if (err < 0)
		return err;

	regmap_write(ts->map, REG_CONFIG, ctrl);

	return 0;
}

static const struct pwm_ops ts_pwm_ops = {
	.apply = ts_pwm_apply,
};

static const struct regmap_config tspwm_32bit_regmap_config = {
	.reg_bits = 16,
	.val_bits = 32,
	.reg_shift = REGMAP_UPSHIFT(2),
	.max_register = 0x3F,
};

static const struct regmap_config tspwm_16bit_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = REGMAP_UPSHIFT(1),
	.max_register = 0x3F,
};

static int ts_pwm_probe(struct platform_device *pdev)
{
	const struct regmap_config *regmap_cfg;
	struct pwm_chip *chip;
	void __iomem *base;
	struct ts_pwm *ts;
	int err;

	chip = devm_pwmchip_alloc(&pdev->dev, 1, sizeof(*ts));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	ts = pwmchip_get_drvdata(chip);

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	ts->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(ts->clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		return PTR_ERR(ts->clk);
	}

	regmap_cfg = device_get_match_data(&pdev->dev);
	ts->map = devm_regmap_init_mmio(&pdev->dev, base, regmap_cfg);
	if (IS_ERR(ts->map))
		return PTR_ERR(ts->map);


	chip->ops = &ts_pwm_ops;

	pm_runtime_enable(&pdev->dev);

	err = devm_pwmchip_add(&pdev->dev, chip);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to register PWM chip: %d\n", err);
		return err;
	}

	return 0;
}

static const struct of_device_id ts_pwm_matches[] = {
	{ .compatible = "technologic,pwm", .data = &tspwm_16bit_regmap_config},
	{ .compatible = "technologic,pwm32", .data = &tspwm_32bit_regmap_config},
	{},
};
MODULE_DEVICE_TABLE(of, ts_pwm_matches);

static struct platform_driver ts_pwm_driver = {
	.driver = {
		.name = "ts-pwm",
		.of_match_table = ts_pwm_matches,
	},
	.probe = ts_pwm_probe,
};
module_platform_driver(ts_pwm_driver);

MODULE_ALIAS("platform:ts-pwm");
MODULE_DESCRIPTION("embeddedTS PS");
MODULE_AUTHOR("Mark Featherston <mark@embeddedTS.com>");
MODULE_LICENSE("GPL");
