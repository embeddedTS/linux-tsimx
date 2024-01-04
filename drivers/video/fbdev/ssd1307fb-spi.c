/*
 * Driver for the Solomon SSD1307 OLED controller
 *
 * Copyright 2012 Free Electrons
 *
 * Licensed under the GPLv2 or later.
 *
 * Modified by embeddedTS for SPI (original is i2c)
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/fb.h>
#include <linux/uaccess.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pwm.h>
#include <linux/delay.h>

#define SSD1307FB_DATA			0x40
#define SSD1307FB_COMMAND		0x80

#define SSD1307FB_SET_ADDRESS_MODE	0x20
#define SSD1307FB_SET_ADDRESS_MODE_HORIZONTAL	(0x00)
#define SSD1307FB_SET_ADDRESS_MODE_VERTICAL	(0x01)
#define SSD1307FB_SET_ADDRESS_MODE_PAGE		(0x02)
#define SSD1307FB_SET_COL_RANGE		0x21
#define SSD1307FB_SET_PAGE_RANGE	0x22
#define SSD1307FB_CONTRAST		0x81
#define	SSD1307FB_CHARGE_PUMP		0x8d
#define SSD1307FB_SEG_REMAP_ON		0xa1
#define SSD1307FB_DISPLAY_OFF		0xae
#define SSD1307FB_SET_MULTIPLEX_RATIO	0xa8
#define SSD1307FB_DISPLAY_ON		0xaf
#define SSD1307FB_START_PAGE_ADDRESS	0xb0
#define SSD1307FB_SET_DISPLAY_OFFSET	0xd3
#define	SSD1307FB_SET_CLOCK_FREQ	0xd5
#define	SSD1307FB_SET_PRECHARGE_PERIOD	0xd9
#define	SSD1307FB_SET_COM_PINS_CONFIG	0xda
#define	SSD1307FB_SET_VCOMH		0xdb

struct ssd1307fb_par;

struct ssd1307fb_ops {
	int (*init)(struct ssd1307fb_par *);
	int (*remove)(struct ssd1307fb_par *);
};

struct ssd1307fb_par {
	struct spi_device	*spi;
	u32 height;
	struct fb_info *info;
	struct ssd1307fb_ops *ops;
	u32 page_offset;
	struct pwm_device *pwm;
	u32 pwm_period;
	int reset;		/* gpio for the RESET# line */
	int datcmd;		/* gpio for the DATA/CMD# line */
	u32 width;
};


static struct fb_fix_screeninfo ssd1307fb_fix = {
	.id		= "Solomon SSD1307",
	.type		= FB_TYPE_PACKED_PIXELS,
	.visual		= FB_VISUAL_MONO10,	/* Monochr. 1=White 0=Black */
	.xpanstep	= 0,
	.ypanstep	= 0,
	.ywrapstep	= 0,
	.accel		= FB_ACCEL_NONE,
	.line_length = 16,
};

static struct fb_var_screeninfo ssd1307fb_var = {
	.bits_per_pixel	= 1,
};


static int ssd1307fb_write_array(struct ssd1307fb_par *par,
				 u8 *array, u32 len)
{
	struct spi_device *spi = par->spi;

	struct spi_message message;
	struct spi_transfer msg_buf;
	int ret = 0;

	gpio_set_value(par->datcmd, 1);

	memset(&msg_buf, 0, sizeof(msg_buf));
	spi_message_init(&message);

	msg_buf.bits_per_word = 8;
	msg_buf.tx_buf = array;
	msg_buf.len = len;
	spi_message_add_tail(&msg_buf, &message);

	ret = spi_sync(spi, &message);

	return ret;
}

static inline int ssd1307fb_write_cmd(struct ssd1307fb_par *par, u8 cmd)
{
	struct spi_device *spi = par->spi;

	struct spi_message message;
	struct spi_transfer msg_buf;
	int ret;

	gpio_set_value(par->datcmd, 0);
	udelay(1);

	memset(&msg_buf, 0, sizeof(msg_buf));
	spi_message_init(&message);

	msg_buf.bits_per_word = 8;
	msg_buf.tx_buf = &cmd;
	msg_buf.len = 1;
	spi_message_add_tail(&msg_buf, &message);
	ret = spi_sync(spi, &message);

	gpio_set_value(par->datcmd, 1);
	return ret;
}

static void ssd1307fb_update_display(struct ssd1307fb_par *par)
{
	u8 *array;
	u8 *vmem = par->info->screen_base;
	int i, j, k;

	array = kzalloc(par->width * par->height / 8, GFP_KERNEL);
	if (!array)
		return;

	/*
	 * The screen is divided in pages, each having a height of 8
	 * pixels, and the width of the screen. When sending a byte of
	 * data to the controller, it gives the 8 bits for the current
	 * column. I.e, the first byte are the 8 bits of the first
	 * column, then the 8 bits for the second column, etc.
	 *
	 *
	 * Representation of the screen, assuming it is 5 bits
	 * wide. Each letter-number combination is a bit that controls
	 * one pixel.
	 *
	 * A0 A1 A2 A3 A4
	 * B0 B1 B2 B3 B4
	 * C0 C1 C2 C3 C4
	 * D0 D1 D2 D3 D4
	 * E0 E1 E2 E3 E4
	 * F0 F1 F2 F3 F4
	 * G0 G1 G2 G3 G4
	 * H0 H1 H2 H3 H4
	 *
	 * If you want to update this screen, you need to send 5 bytes:
	 *  (1) A0 B0 C0 D0 E0 F0 G0 H0
	 *  (2) A1 B1 C1 D1 E1 F1 G1 H1
	 *  (3) A2 B2 C2 D2 E2 F2 G2 H2
	 *  (4) A3 B3 C3 D3 E3 F3 G3 H3
	 *  (5) A4 B4 C4 D4 E4 F4 G4 H4
	 */

	for (i = 0; i < (par->height / 8); i++) {
		for (j = 0; j < par->width; j++) {
			u32 array_idx = i * par->width + j;
			array[array_idx] = 0;
			for (k = 0; k < 8; k++) {
				u32 page_length = par->width * i;
				u32 index = page_length + (par->width * k + j) / 8;
				u8 byte = *(vmem + index);
				u8 bit = byte & (1 << (j % 8));
				bit = bit >> (j % 8);
				array[array_idx] |= bit << k;
			}
		}
	}

	ssd1307fb_write_cmd(par, SSD1307FB_SET_PAGE_RANGE);
	ssd1307fb_write_cmd(par, 0x0);
	ssd1307fb_write_cmd(par, par->page_offset + (par->height / 8) - 1);
	ssd1307fb_write_array(par, array, par->width * par->height / 8);
	kfree(array);
}



static ssize_t ssd1307fb_write(struct fb_info *info, const char __user *buf,
		size_t count, loff_t *ppos)
{
	struct ssd1307fb_par *par = info->par;
	unsigned long total_size;
	unsigned long p = *ppos;
	u8 __iomem *dst;

	total_size = info->fix.smem_len;

	if (p > total_size)
		return -EINVAL;

	if (count + p > total_size)
		count = total_size - p;

	if (!count)
		return -EINVAL;

	dst = (void __force *) (info->screen_base + p);

	if (copy_from_user(dst, buf, count))
		return -EFAULT;

	ssd1307fb_update_display(par);

	*ppos += count;

	return count;
}

static void ssd1307fb_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
	struct ssd1307fb_par *par = info->par;
	sys_fillrect(info, rect);
	ssd1307fb_update_display(par);
}

static void ssd1307fb_copyarea(struct fb_info *info, const struct fb_copyarea *area)
{
	struct ssd1307fb_par *par = info->par;
	sys_copyarea(info, area);
	ssd1307fb_update_display(par);
}

static void ssd1307fb_imageblit(struct fb_info *info, const struct fb_image *image)
{
	struct ssd1307fb_par *par = info->par;
	sys_imageblit(info, image);
	ssd1307fb_update_display(par);
}

static struct fb_ops ssd1307fb_ops = {
	.owner		= THIS_MODULE,
	.fb_read	= fb_sys_read,
	.fb_write	= ssd1307fb_write,
	.fb_fillrect	= ssd1307fb_fillrect,
	.fb_copyarea	= ssd1307fb_copyarea,
	.fb_imageblit	= ssd1307fb_imageblit,

};

static void ssd1307fb_deferred_io(struct fb_info *info,
				struct list_head *pagelist)
{
	ssd1307fb_update_display(info->par);
}

static struct fb_deferred_io ssd1307fb_defio = {
	.delay		= HZ,
	.deferred_io	= ssd1307fb_deferred_io,
};

static int ssd1307fb_ssd1307_init(struct ssd1307fb_par *par)
{
	int ret;

	par->pwm = pwm_get(&par->spi->dev, NULL);
	if (IS_ERR(par->pwm)) {
		dev_err(&par->spi->dev, "Could not get PWM from device tree!\n");
		return PTR_ERR(par->pwm);
	}

	par->pwm_period = pwm_get_period(par->pwm);
	/* Enable the PWM */
	pwm_config(par->pwm, par->pwm_period / 2, par->pwm_period);
	pwm_enable(par->pwm);

	dev_dbg(&par->spi->dev, "Using PWM%d with a %dns period.\n",
		par->pwm->pwm, par->pwm_period);

	/* Map column 127 of the OLED to segment 0 */
	ret = ssd1307fb_write_cmd(par, SSD1307FB_SEG_REMAP_ON);
	if (ret < 0)
		return ret;

	/* Turn on the display */
	ret = ssd1307fb_write_cmd(par, SSD1307FB_DISPLAY_ON);
	if (ret < 0)
		return ret;

	return 0;
}

static int ssd1307fb_ssd1307_remove(struct ssd1307fb_par *par)
{
	pwm_disable(par->pwm);
	pwm_put(par->pwm);
	return 0;
}

static struct ssd1307fb_ops ssd1307fb_ssd1307_ops = {
	.init	= ssd1307fb_ssd1307_init,
	.remove	= ssd1307fb_ssd1307_remove,
};

static int ssd1307fb_ssd1306_init(struct ssd1307fb_par *par)
{
	int ret;

	/* Set initial contrast */
	ret = ssd1307fb_write_cmd(par, SSD1307FB_CONTRAST);
	if (ret < 0)
		return ret;

	ret = ssd1307fb_write_cmd(par, 0x7f);
	if (ret < 0)
		return ret;

	/* Set COM direction */
	ret = ssd1307fb_write_cmd(par, 0xc8);
	if (ret < 0)
		return ret;

	/* Set segment re-map */
	ret = ssd1307fb_write_cmd(par, SSD1307FB_SEG_REMAP_ON);
	if (ret < 0)
		return ret;

	/* Set multiplex ratio value */
	ret = ssd1307fb_write_cmd(par, SSD1307FB_SET_MULTIPLEX_RATIO);
	if (ret < 0)
		return ret;

	ret = ssd1307fb_write_cmd(par, par->height - 1);
	if (ret < 0)
		return ret;

	/* set display offset value */
	ret = ssd1307fb_write_cmd(par, SSD1307FB_SET_DISPLAY_OFFSET);
	if (ret < 0)
		return ret;

	ret = ssd1307fb_write_cmd(par, par->page_offset);
	if (ret < 0)
		return ret;

	/* Set clock frequency */
	ret = ssd1307fb_write_cmd(par, SSD1307FB_SET_CLOCK_FREQ);
	if (ret < 0)
		return ret;

	ret = ssd1307fb_write_cmd(par, 0xf0);
	if (ret < 0)
		return ret;

	/* Set precharge period in number of ticks from the internal clock */
	ret = ssd1307fb_write_cmd(par, SSD1307FB_SET_PRECHARGE_PERIOD);
	if (ret < 0)
		return ret;

	ret = ssd1307fb_write_cmd(par, 0x22);
	if (ret < 0)
		return ret;

	/* Set COM pins configuration */
	ret = ssd1307fb_write_cmd(par, SSD1307FB_SET_COM_PINS_CONFIG);
	if (ret < 0)
		return ret;

	ret = ssd1307fb_write_cmd(par, 0x12);
	if (ret < 0)
		return ret;

	/* Set VCOMH */
	ret = ssd1307fb_write_cmd(par, SSD1307FB_SET_VCOMH);
	if (ret < 0)
		return ret;

	ret = ssd1307fb_write_cmd(par, 0x49);
	if (ret < 0)
		return ret;

	/* Turn on the DC-DC Charge Pump */
	ret = ssd1307fb_write_cmd(par, SSD1307FB_CHARGE_PUMP);
	if (ret < 0)
		return ret;

	ret = ssd1307fb_write_cmd(par, 0x14);
	if (ret < 0)
		return ret;

	/* Switch to horizontal addressing mode */
	ret = ssd1307fb_write_cmd(par, SSD1307FB_SET_ADDRESS_MODE);
	if (ret < 0)
		return ret;

	ret = ssd1307fb_write_cmd(par,
				  SSD1307FB_SET_ADDRESS_MODE_HORIZONTAL);
	if (ret < 0)
		return ret;

	ret = ssd1307fb_write_cmd(par, SSD1307FB_SET_COL_RANGE);
	if (ret < 0)
		return ret;

	ret = ssd1307fb_write_cmd(par, 0x0);
	if (ret < 0)
		return ret;

	ret = ssd1307fb_write_cmd(par, par->width - 1);
	if (ret < 0)
		return ret;

	ret = ssd1307fb_write_cmd(par, SSD1307FB_SET_PAGE_RANGE);
	if (ret < 0)
		return ret;

	ret = ssd1307fb_write_cmd(par, 0x0);
	if (ret < 0)
		return ret;

	ret = ssd1307fb_write_cmd(par,
				  par->page_offset + (par->height / 8) - 1);
	if (ret < 0)
		return ret;

	/* Turn on the display */
	ret = ssd1307fb_write_cmd(par, SSD1307FB_DISPLAY_ON);
	if (ret < 0)
		return ret;

	return 0;
}

static struct ssd1307fb_ops ssd1307fb_ssd1306_ops = {
	.init	= ssd1307fb_ssd1306_init,
};

static const struct of_device_id ssd1307fb_of_match[] = {
	{
		.compatible = "solomon,ssd1306fb-spi",
		.data = (void *)&ssd1307fb_ssd1306_ops,
	},
	{
		.compatible = "solomon,ssd1307fb-spi",
		.data = (void *)&ssd1307fb_ssd1307_ops,
	},
	{},
};
MODULE_DEVICE_TABLE(of, ssd1307fb_of_match);

static int ssd1307fb_probe(struct spi_device *spi)
{
	struct fb_info *info;
	struct device_node *node = spi->dev.of_node;
	u32 vmem_size;
	struct ssd1307fb_par *par;
	u8 *vmem;
	int ret;

	if (!node) {
		dev_err(&spi->dev, "No device tree data found!\n");
		return -EINVAL;
	}

	info = framebuffer_alloc(sizeof(struct ssd1307fb_par), &spi->dev);
	if (!info) {
		dev_err(&spi->dev, "Couldn't allocate framebuffer.\n");
		return -ENOMEM;
	}

	par = info->par;
	par->info = info;
	par->spi = spi;

	par->ops = (struct ssd1307fb_ops *)of_match_device(ssd1307fb_of_match,
								&spi->dev)->data;

	par->reset = of_get_named_gpio(spi->dev.of_node,
					 "reset-gpios", 0);
	if (!gpio_is_valid(par->reset)) {
		ret = -EINVAL;
		goto fb_alloc_error;
	}

	par->datcmd = of_get_named_gpio(spi->dev.of_node,
					 "datcmd-gpios", 0);
	if (!gpio_is_valid(par->datcmd)) {
		ret = -EINVAL;
		goto fb_alloc_error;
	}

	if (of_property_read_u32(node, "solomon,width", &par->width))
		par->width = 96;

	if (of_property_read_u32(node, "solomon,height", &par->height))
		par->height = 16;

	if (of_property_read_u32(node, "solomon,page-offset", &par->page_offset))
		par->page_offset = 0;

	vmem_size = par->width * par->height / 8;

	vmem = devm_kzalloc(&spi->dev, vmem_size, GFP_KERNEL);
	if (!vmem) {
		dev_err(&spi->dev, "Couldn't allocate graphical memory.\n");
		ret = -ENOMEM;
		goto fb_alloc_error;
	}

	info->fbops = &ssd1307fb_ops;
	info->fix = ssd1307fb_fix;
	info->fix.line_length = par->width / 8;
	info->fbdefio = &ssd1307fb_defio;

	info->var = ssd1307fb_var;
	info->var.xres = par->width;
	info->var.xres_virtual = par->width;
	info->var.yres = par->height;
	info->var.yres_virtual = par->height;

	info->var.red.length = 1;
	info->var.red.offset = 0;
	info->var.green.length = 1;
	info->var.green.offset = 0;
	info->var.blue.length = 1;
	info->var.blue.offset = 0;

	info->screen_base = (u8 __force __iomem *)vmem;
	info->fix.smem_start = (unsigned long)vmem;
	info->fix.smem_len = vmem_size;

	fb_deferred_io_init(info);

	ret = devm_gpio_request_one(&spi->dev, par->reset,
					 GPIOF_OUT_INIT_HIGH,
					 "oled-reset");
	if (ret) {
		dev_err(&spi->dev,
			"failed to request reset gpio %d: %d\n",
			par->reset, ret);
		goto reset_oled_error;
	}

	ret = devm_gpio_request_one(&spi->dev, par->datcmd,
					 GPIOF_OUT_INIT_HIGH,
					 "oled-datcmd");
	if (ret) {
		dev_err(&spi->dev,
			"failed to request datcmd gpio %d: %d\n",
			par->datcmd, ret);
		goto reset_oled_error;
	}

	spi_set_drvdata(spi, info);

	/* Reset the screen */
	gpio_set_value(par->reset, 0);
	udelay(400);
	gpio_set_value(par->reset, 1);
	udelay(400);

	if (par->ops->init) {
		ret = par->ops->init(par);
		if (ret)
			goto reset_oled_error;
	}

	ret = register_framebuffer(info);
	if (ret) {
		dev_err(&spi->dev, "Couldn't register the framebuffer\n");
		goto panel_init_error;
	}

	dev_info(&spi->dev, "fb%d: %s framebuffer device registered, using %d bytes of video memory\n", info->node, info->fix.id, vmem_size);

	return 0;

panel_init_error:
	if (par->ops->remove)
		par->ops->remove(par);
reset_oled_error:
	fb_deferred_io_cleanup(info);
fb_alloc_error:
	framebuffer_release(info);
	return ret;
}

static int ssd1307fb_remove(struct spi_device *spi)
{
	struct fb_info *info = spi_get_drvdata(spi);
	struct ssd1307fb_par *par = info->par;

	unregister_framebuffer(info);
	if (par->ops->remove)
		par->ops->remove(par);
	fb_deferred_io_cleanup(info);
	framebuffer_release(info);

	return 0;
}

static const struct spi_device_id ssd1307fb_spi_id[] = {
	{ "ssd1306fb", 0 },
	{ "ssd1307fb", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, ssd1307fb_spi_id);

static struct spi_driver ssd1307fb_driver = {
	.probe = ssd1307fb_probe,
	.remove = ssd1307fb_remove,
	.id_table = ssd1307fb_spi_id,
	.driver = {
		.name = "ssd1307fb-spi",
		.of_match_table = ssd1307fb_of_match,
		.owner = THIS_MODULE,
	},
};

module_spi_driver(ssd1307fb_driver);

MODULE_DESCRIPTION("FB driver for the Solomon SSD1307 OLED controller");
MODULE_AUTHOR("Maxime Ripard <maxime.ripard@free-electrons.com>");
MODULE_AUTHOR("embeddedTS Inc.");
MODULE_LICENSE("GPL v2");
