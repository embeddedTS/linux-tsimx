#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#include <linux/module.h>
#include <linux/kallsyms.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/blkdev.h>
#include <linux/dma-mapping.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/hdreg.h>
#include <linux/proc_fs.h>
#include <linux/reboot.h>

#define DEFAULT_POLL_RATE	1
static int poll_rate = -1;
module_param(poll_rate, int, 0644);
MODULE_PARM_DESC(poll_rate,
		 "Rate in seconds to poll for SD card.  Defaults to 1\n");

#define DEFAULT_POLLING_DISABLE 0
static int disable_poll = -1;
module_param(disable_poll, int, 0644);
MODULE_PARM_DESC(disable_poll,
		 "Set to non-zero to only check for SD once on startup\n");

#define DRIVER_NAME "tssdcard"

/*
 * Current use of this driver only includes TS-7120
 */
#define MAX_SDS	1

#define SDWAIT(s)	while(! (readw((unsigned short *)((s)->sd_syscon + 0x12)) & (1<<5)))
#define SDPOKE32(s, x, y)	do { writel((y), (volatile void *)((s)->sd_regstart + (x))); SDWAIT(s); } while (0)
#define SDPOKE16(s, x, y)	do { writew((y), (volatile void *)((s)->sd_regstart + 0x10 + (x))); SDWAIT(s); } while (0)
#define SDPOKE8(s, x, y)	do { writeb((y), (volatile void *)((s)->sd_regstart + 0x20 + ((x) << 1))); SDWAIT(s); } while (0)
#define SDPEEK32(s, x)		readl((unsigned int *)((s)->sd_regstart + (x)))
#define SDPEEK16(s, x)		readw((unsigned short *)((s)->sd_regstart + 0x10 + (x)))
#define SDPEEK8(s, x)		readb((unsigned char *)((s)->sd_regstart + 0x20 + ((x) << 1))  )

/* Disable probing for eMMC, we only connect this core here to SD */
#define SD_NOMMC
#define SD_NOAUTOMMC

/* Layer includes low level hardware support */
#include "tssdcore2.c"

static DEFINE_MUTEX(tssdcore_lock);
static struct semaphore sem;
static atomic_t busy;

struct tssdcard_dev {
	struct device *dev;
	struct sdcore tssdcore;
	char *devname;
	sector_t sectors;
	struct gendisk *gd;
	struct request_queue *queue;
	struct bio *bio;
	struct bio *biotail;
	spinlock_t lock;
	atomic_t users;    /* How many users */
	struct task_struct *thread;
	wait_queue_head_t event;
	struct work_struct diskpoll_work;
	struct workqueue_struct *diskpoll_queue;
	struct timer_list cd_timer;
	unsigned long timeout;
	unsigned long lasttimeout;
	int cardpresent;
	int lasterr;
	int major;
};

struct tssdcard_host {
	struct platform_device *pdev;
	struct resource *mem_res;
	void __iomem *base, *syscon;
	int numluns;
	struct tssdcard_dev luns[MAX_SDS];
};


static int tssdcard_probe(struct platform_device *pdev);
static int tssdcard_remove(struct platform_device *pdev);


static void tssdcard_add_bio(struct tssdcard_dev *dev, struct bio *bio)
{
	spin_lock(&dev->lock);
	if (dev->biotail) {
		dev->biotail->bi_next = bio;
		dev->biotail = bio;
	} else{
		dev->bio = dev->biotail = bio;
	}
	spin_unlock(&dev->lock);
}

static struct bio *tssdcard_get_bio(struct tssdcard_dev *dev)
{
	struct bio *bio;

	spin_lock(&dev->lock);
	bio = dev->bio;
	if (bio) {
		if (bio == dev->biotail)
			dev->biotail = NULL;
		dev->bio = bio->bi_next;
		bio->bi_next = NULL;
	}
	spin_unlock(&dev->lock);
	return bio;
}

static void tssdcard_reset_timeout(struct sdcore *sd)
{
	if (sd) {
		struct tssdcard_dev *dev = (struct tssdcard_dev *)sd->os_arg;
		if (dev) {
			/* SD Spec allows 1 second for cards to answer */
			dev->timeout = jiffies + HZ;
		}
	}
}

static int tssdcard_transfer(struct tssdcard_dev *dev, unsigned long sector,
					unsigned long nsect, char *buffer, int rw)
{
	int ret = 0;

	dev_dbg(dev->dev, "%s size:%lld sector:%lu nsect:%lu rw:%d\n",
		__func__, (long long)dev->sectors, sector, nsect, rw);

	tssdcard_reset_timeout(&dev->tssdcore);
	switch (rw) {
	case WRITE:
		ret = sdwrite(&dev->tssdcore, sector, buffer, nsect);
		if (ret && !dev->tssdcore.sd_wprot) {
			if (sdreset(&dev->tssdcore) != 0) {
				//tssdcard_reset_timeout(dev);
				tssdcard_reset_timeout(&dev->tssdcore);
				ret = sdwrite(&dev->tssdcore, sector,
							buffer, nsect);
			}
		}
		break;

	case READ:
	case READA:
		ret = sdread(&dev->tssdcore, sector, buffer, nsect);
		if (ret) {
			if (sdreset(&dev->tssdcore) != 0) {
				//tssdcard_reset_timeout(dev);
				tssdcard_reset_timeout(&dev->tssdcore);
				ret = sdread(&dev->tssdcore, sector,
						  buffer, nsect);
			}
		}
	}

	return ret;
}

static void tssdcard_handle_bio(struct tssdcard_dev *dev, struct bio *bio)
{
	struct bio_vec bvec;
	struct bvec_iter iter;
	sector_t sector, end_sector, n_sectors;
	char *buffer;
	int ret = 0;

	sector = bio->bi_iter.bi_sector;
	end_sector = (bio->bi_iter.bi_sector) + (bio->bi_iter.bi_size >> 9) - 1;

	if ((bio->bi_iter.bi_size % 512) != 0)
		panic("Invalid transfer, bi_size 512 != 0\n");

	bio_for_each_segment(bvec, bio, iter) {
		if ((sector + (bvec.bv_len >> 9)) > end_sector)
			n_sectors = end_sector - sector + 1;
		else
			n_sectors = bvec.bv_len >> 9;
		if (n_sectors == 0)
			continue;

		buffer = kmap(bvec.bv_page) + bvec.bv_offset;
		ret = tssdcard_transfer(dev, sector, n_sectors, buffer,
					bio_data_dir(bio));
		sector += n_sectors;
		kunmap(bvec.bv_page);
	}

	bio_endio(bio, ret);

	if (ret) {
		dev->cardpresent = 0;
		queue_work(dev->diskpoll_queue,
				&dev->diskpoll_work);
	}
}

static void tssdcard_delay(void *data, unsigned int us)
{
	if (us > 50000)
		msleep_interruptible(us/1000);
	else
		udelay(us);
}

static int tssdcard_timeout_relaxed(void *data)
{
	struct sdcore *sd = (struct sdcore *)data;
	struct tssdcard_dev *dev = (struct tssdcard_dev *)sd->os_arg;
	int ret;

	dev->lasttimeout = jiffies;

	if (jiffies_to_msecs(dev->timeout - jiffies) > 50)
		msleep_interruptible(10);

	ret = time_is_before_jiffies(dev->timeout);

	return ret;
}

static int tssdcard_timeout(void *data)
{
	struct sdcore *sd = (struct sdcore *)data;
	struct tssdcard_dev *dev = (struct tssdcard_dev *)sd->os_arg;
	int ret;

	dev->lasttimeout = jiffies;
	ret = time_is_before_jiffies(dev->timeout);

	return ret;
}

static void tssdcard_irqwait(void *data, unsigned int x)
{
	struct tssdcard_dev *dev = (struct tssdcard_dev *)data;
	uint32_t reg;

	do {
#ifdef CONFIG_PREEMPT_NONE
		/* Default marvell kernel config has no preempt, so
		 * to support that:
		 */
		cond_resched();
#endif

		reg = readw((uint16_t *)(dev->tssdcore.sd_syscon + 0x12));
	} while (reg & (1 << 6));
}

static void tssdcard_release(struct gendisk *disk, fmode_t mode)
{
	struct tssdcard_dev *dev = disk->private_data;

	atomic_dec(&dev->users);
	if (atomic_read(&dev->users) == 0) {
		if (dev->thread != NULL) {
			char buffer[512];
			int ret;

			kthread_stop(dev->thread);
			dev->thread = NULL;
			if (dev->sectors) {
				tssdcard_reset_timeout(&dev->tssdcore);
				ret = sdread(&dev->tssdcore, 0, buffer, 1);
			}
		}
	}
}

static int tssdcard_peek_bio(struct tssdcard_dev *dev)
{
	int ret = 0;

	spin_lock(&dev->lock);
	if (dev->bio != NULL)
		ret = 1;
	spin_unlock(&dev->lock);

	return ret;
}

static int tssdcard_thread(void *data)
{
	struct tssdcard_dev *dev = data;
	struct bio *bio;

	while (!kthread_should_stop()) {
		wait_event_interruptible(dev->event,
					 tssdcard_peek_bio(dev) ||
					 kthread_should_stop());

		if (down_interruptible(&sem))
			continue;

		atomic_inc(&busy);
		if (atomic_read(&busy) > 1)
			panic("recursive make_request!\n");

		bio = tssdcard_get_bio(dev);
		if (bio)
			tssdcard_handle_bio(dev, bio);

		atomic_dec(&busy);
		up(&sem);
	}

	return 0;
}

static int tssdcard_open(struct block_device *bdev, fmode_t mode)
{
	struct tssdcard_dev *dev = bdev->bd_disk->private_data;

	if (!atomic_read(&dev->users))
		check_disk_change(bdev);
	atomic_inc(&dev->users);
	if (dev->thread == NULL && atomic_read(&dev->users)) {
		dev->thread = kthread_create(tssdcard_thread,
						  dev, dev->devname);
		if (IS_ERR(dev->thread))
			dev->thread = NULL;
		else
			wake_up_process(dev->thread);
	}

	return 0;
}


static void  tssdcard_make_request(struct request_queue *q, struct bio *bio)
{
	struct tssdcard_dev *dev = q->queuedata;

	tssdcard_add_bio(dev, bio);
	wake_up(&dev->event);

}

static int tssdcard_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
	struct tssdcard_dev *dev = bdev->bd_disk->private_data;

	geo->cylinders = dev->sectors >> 9 / (4 * 16);
	geo->heads = 4;
	geo->sectors = 16;
	return 0;
}

static const struct block_device_operations tssdcard_ops = {
	.owner		= THIS_MODULE,
	.open		= tssdcard_open,
	.release	= tssdcard_release,
	.getgeo		= tssdcard_getgeo,
};

static void tssdcard_alloc_disk(struct tssdcard_dev *dev)
{
	dev->bio = dev->biotail = NULL;
	dev->gd = alloc_disk(CONFIG_MMC_BLOCK_MINORS);
	if (dev->gd == NULL) {
		pr_err(DRIVER_NAME ": Failed to alloc_disk");
		return;
	}

	strcpy(dev->gd->disk_name, dev->devname);
	dev->queue = blk_alloc_queue(GFP_KERNEL);
	if (dev->queue == NULL) {
		pr_err(DRIVER_NAME ": Failed to alloc blk queue");
		return;
	}
	dev->queue->queuedata = dev;
	blk_queue_logical_block_size(dev->queue, 512);
	queue_flag_set_unlocked(QUEUE_FLAG_NONROT, dev->queue);
	blk_queue_make_request(dev->queue, tssdcard_make_request);

	set_capacity(dev->gd, dev->sectors);
	dev->gd->major = dev->major;
	dev->gd->first_minor = dev->tssdcore.sd_lun * CONFIG_MMC_BLOCK_MINORS;
	dev->gd->flags = 0;
	dev->gd->fops = &tssdcard_ops;
	dev->gd->queue = dev->queue;
	dev->gd->private_data = dev;

	/* Check disk WP */
	set_disk_ro(dev->gd, dev->tssdcore.sd_wprot);

	add_disk(dev->gd);
}
static void tssdcard_cleanup_disk(struct tssdcard_dev *dev)
{
	pr_info("SD card was removed!\n");
	del_gendisk(dev->gd);
	blk_cleanup_queue(dev->queue);
	put_disk(dev->gd);
	dev->sectors = 0;
}

static void tssdcard_card_poll(unsigned long data)
{
	struct tssdcard_dev *dev = (struct tssdcard_dev *) data;

	queue_work(dev->diskpoll_queue, &dev->diskpoll_work);
}

static void diskpoll_thread(struct work_struct *work)
{
	struct tssdcard_dev *dev = container_of(work, struct tssdcard_dev,
		diskpoll_work);

	if (!dev->cardpresent && dev->sectors != 0)
		tssdcard_cleanup_disk(dev);
	else
		dev->sectors = sdreset(&dev->tssdcore);

	if (dev->sectors == 0) {
		dev->tssdcore.os_timeout = tssdcard_timeout_relaxed;
		if (!disable_poll)
			mod_timer(&dev->cd_timer, jiffies + (HZ * poll_rate));

	} else {
		dev->cardpresent = 1;
		dev->tssdcore.os_timeout = tssdcard_timeout;
		tssdcard_alloc_disk(dev);
	}
}


static int setup_device(struct tssdcard_host *host, int lun)
{
	int ret = 0;
	struct tssdcard_dev *dev = &host->luns[lun];

	dev->dev = &host->pdev->dev;
	/* IO Remapping (use the same virtual address for all LUNs) */
	dev->tssdcore.sd_regstart = (unsigned int)host->base;
	dev->tssdcore.sd_syscon = (unsigned int)host->syscon;
	dev->tssdcore.sd_lun = lun;
	dev->tssdcore.os_timeout = tssdcard_timeout;
	dev->tssdcore.os_reset_timeout = (int (*)(void *))tssdcard_reset_timeout;
	dev->tssdcore.os_arg = dev;
	dev->tssdcore.os_delay = tssdcard_delay;
	dev->tssdcore.os_irqwait = tssdcard_irqwait;
	dev->tssdcore.sd_writeparking = 1;
	dev->tssdcore.debug = NULL;
	dev->tssdcore.debug_arg = dev;
	dev->major = register_blkdev(UNNAMED_MAJOR, DRIVER_NAME);

	if (dev->major < 0) {
		pr_err("Cannot register block device '%s'\n", DRIVER_NAME);
		return -ENOMEM;
	}

	dev->devname = kmalloc(32, GFP_KERNEL);
	if (!dev->devname)
		return -ENOMEM;
	snprintf(dev->devname, 32, "%s%c", DRIVER_NAME, lun + 'a');

	init_waitqueue_head(&dev->event);
	sema_init(&sem, 1);
	atomic_set(&busy, 0);
	spin_lock_init(&dev->lock);

	/* sdreset sleeps so we need our own workqueue */
	dev->diskpoll_queue = alloc_ordered_workqueue(dev->devname, 0);
	if (!dev->diskpoll_queue) {
		pr_err("Cannot allocate workqueue\n");
		return -ENOMEM;
	}

	INIT_WORK(&dev->diskpoll_work, diskpoll_thread);

	dev->cd_timer.function = tssdcard_card_poll;
	dev->cd_timer.data = (unsigned long)dev;

	/* Start polling for the card */
	queue_work(dev->diskpoll_queue,
		&dev->diskpoll_work);

	return ret;
}



static const struct of_device_id tssdcard_of_match[] = {
	{
		.compatible = "technologicsystems,tssdcard",
	}
};

static struct platform_driver tssdcard_driver = {
	.probe =  tssdcard_probe,
	.remove = tssdcard_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = tssdcard_of_match,
	}
};


static int tssdcard_probe(struct platform_device *pdev)
{
	int i, ret = 0;
	struct tssdcard_host *host;
	struct device_node *np = pdev->dev.of_node;
	struct resource *sdcore = 0;
	struct resource *syscon = 0;


	host = kzalloc(sizeof(struct tssdcard_host), GFP_KERNEL);
	if (host == NULL) {
		ret = -ENOMEM;
		goto out;
	}


	if (of_property_read_u32(np, "tssdcard,ndevices", &host->numluns) < 0) {
		pr_info("Can't read property 'tssdcard,ndevices' in device-tree; assuming 1\n");
		host->numluns = 1;
	}

	if (poll_rate == -1) {
		if (of_property_read_u32(np, "tssdcard,poll_rate", &poll_rate) < 0) {
			pr_info("Can't read property 'tssdcard,poll_rate' in device-tree\n");
			poll_rate = DEFAULT_POLL_RATE;
		}
	}

	if (disable_poll == -1) {
		if (of_property_read_u32(np, "tssdcard,disable_poll", &disable_poll) < 0) {
			pr_info("Can't read property 'tssdcard,disable_poll' in device-tree\n");
			disable_poll = DEFAULT_POLLING_DISABLE;
		}
	}

	sdcore = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	syscon = platform_get_resource(pdev, IORESOURCE_MEM, 1);

	if (sdcore == NULL || syscon == NULL) {
		pr_err("Can't get device address\n");
		kfree(host);
		ret = -EFAULT;
		goto out;
	}

	if (!devm_request_mem_region(&pdev->dev, syscon->start,
		 resource_size(syscon), pdev->name)) {
		ret = -EBUSY;
		goto out;
	}

	host->syscon = devm_ioremap_nocache(&pdev->dev, syscon->start,
					  resource_size(syscon));
	if (!host->syscon) {

		ret = -EFAULT;
		goto out;
	}

	pr_info("Model ID: 0x%08X\n", readl(host->syscon));

	if (readl(host->syscon) == 0xdeadbeef) {
		pr_err("Error! FPGA is deadbeef\n");
		ret = -EFAULT;
		goto out;
	}

	if (!devm_request_mem_region(&pdev->dev, sdcore->start,
		 resource_size(sdcore), pdev->name)) {
		ret = -EBUSY;
		goto out;
	}

	host->base = devm_ioremap_nocache(&pdev->dev, sdcore->start,
					  resource_size(sdcore));
	if (!host->base) {
		devm_iounmap(&pdev->dev, host->syscon);

		ret = -EFAULT;
		goto out;
	}

	for (i = 0; i < host->numluns; i++) {
		ret = setup_device(host, i);
		if (ret)
			goto out;
	}

	platform_set_drvdata(pdev, host);

	return 0;

out:
	return ret;
}

static int tssdcard_remove(struct platform_device *pdev)
{
	struct tssdcard_host *host = platform_get_drvdata(pdev); //pdev->dev.p;
	int i;

	for (i = 0; i < host->numluns; i++) {
		struct tssdcard_dev *dev = &host->luns[i];

		dev_dbg(dev->dev, "dev[%d] ...\n", i);

		if (dev->sectors == 0)
			continue;

		if (dev->gd) {
			del_gendisk(dev->gd);
			put_disk(dev->gd);
		}

		unregister_blkdev(dev->major, DRIVER_NAME);
		kfree(dev->devname);

		if (dev->queue) {
			blk_cleanup_queue(dev->queue);
			blk_put_queue(dev->queue);
		}
	}
	return 0;
}

static const struct platform_device_id tssdcard_devtype[] = {
	{
		.name = "tssdcard-mmc",
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(platform, tssdcard_devtype);


module_platform_driver(tssdcard_driver);

MODULE_DESCRIPTION("TS-7120 SDHC Driver");
MODULE_AUTHOR("Ian Coughlan, Technologic Systems");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tssdcard");
