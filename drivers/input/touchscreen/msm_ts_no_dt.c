/* drivers/input/touchscreen/msm_ts_no_dt.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2011-2013 The CyanogenMod Project
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mfd/marimba-tsadc.h>
#include <linux/pm.h>
#include <linux/slab.h>

#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#include <linux/input/msm_ts.h>
#include <linux/jiffies.h>
#define	TSSC_CTL			0x100 // 256
#define	TSSC_CTL_PENUP_IRQ		(1 << 12) 
#define	TSSC_CTL_DATA_FLAG		(1 << 11)
#define	TSSC_CTL_DEBOUNCE_EN		(1 << 6) // 40
#define	TSSC_CTL_EN_AVERAGE		(1 << 5) // 20
#define	TSSC_CTL_MODE_MASTER		(3 << 3) // 18
#define	TSSC_CTL_SW_RESET		(1 << 2)
#define	TSSC_CTL_ENABLE			(1 << 0) // 1
#define	TSSC_OPN			0x104 // 260
#define	TSSC_OPN_NOOP			0x00 // 0
#define	TSSC_OPN_4WIRE_X		0x01 // 1
#define	TSSC_OPN_4WIRE_Y		0x02 // 2
#define	TSSC_OPN_4WIRE_Z1		0x03 // 3
#define	TSSC_OPN_4WIRE_Z2		0x04 // 4
#define	TSSC_SAMPLING_INT		0x108 // 264
#define	TSSC_STATUS			0x10c // 268
#define	TSSC_AVG_12			0x110 // 272
#define	TSSC_AVG_34			0x114 // 276
#define	TSSC_SAMPLE(op,samp)		((0x118 + ((op & 0x3) * 0x20)) + ((samp & 0x7) * 0x4))
#define	TSSC_TEST_1			0x198
#define	TSSC_TEST_1_EN_GATE_DEBOUNCE	(1 << 2)
#define	TSSC_TEST_2			0x19c
#define	TS_PENUP_TIMEOUT_MS		70 // 20  -->  70

struct msm_ts {
	struct msm_ts_platform_data	*pdata;
	struct input_dev		*input_dev;
	void __iomem			*tssc_base;
	uint32_t			ts_down:1;
	struct ts_virt_key		*vkey_down;

	unsigned int			sample_irq;
	unsigned int			pen_up_irq;

#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend		early_suspend;
#endif
	struct device			*dev;
	struct timer_list timer;
};

static uint32_t msm_tsdebug;
module_param_named(tsdebug, msm_tsdebug, uint, 0664);

static int32_t msm_tscal_xscale = 17388;
static int32_t msm_tscal_xymix = -76;
static int32_t msm_tscal_xoffset = -871728;
static int32_t msm_tscal_yxmix = 119;
static int32_t msm_tscal_yscale = 25403;
static int32_t msm_tscal_yoffset = 45240;

module_param_named(tscal_xscale, msm_tscal_xscale, int, 0664);
module_param_named(tscal_xymix, msm_tscal_xymix, int, 0664);
module_param_named(tscal_xoffset, msm_tscal_xoffset, int, 0664);
module_param_named(tscal_yxmix, msm_tscal_yxmix, int, 0664);
module_param_named(tscal_yscale, msm_tscal_yscale, int, 0664);
module_param_named(tscal_yoffset, msm_tscal_yoffset, int, 0664);

#define tssc_readl(t, a)	(readl(((t)->tssc_base) + (a)))
#define tssc_writel(t, v, a)	do {writel(v, ((t)->tssc_base) + (a));} while(0)

static void setup_next_sample(struct msm_ts *ts)
{
	uint32_t tmp;
	/* 7: 6ms debounce time
	 * 5: 3ms debounce time
	 * 3: 1.2ms debounce time */
	tmp = ((1 << 2) | TSSC_CTL_DEBOUNCE_EN | TSSC_CTL_EN_AVERAGE |
	       TSSC_CTL_MODE_MASTER | TSSC_CTL_ENABLE);
	tssc_writel(ts, tmp, TSSC_CTL);
}

static irqreturn_t msm_ts_irq(int irq, void *dev_id)
{
	struct msm_ts *ts = dev_id;
	struct msm_ts_platform_data *pdata = ts->pdata;

	uint32_t tssc_avg12, tssc_avg34, tssc_status, tssc_ctl;
	int x, y, z1, z2;
	int down;
	int z=0;
	tssc_ctl = tssc_readl(ts, TSSC_CTL);
	tssc_status = tssc_readl(ts, TSSC_STATUS);
	tssc_avg12 = tssc_readl(ts, TSSC_AVG_12);
	tssc_avg34 = tssc_readl(ts, TSSC_AVG_34);

	setup_next_sample(ts);

	x = tssc_avg12 & 0xffff;
	y = tssc_avg12 >> 16;
	z1 = tssc_avg34 & 0xffff;
	z2 = tssc_avg34 >> 16;

	/* invert the inputs if necessary */
	if (pdata->inv_x) x = pdata->inv_x - x;
	if (pdata->inv_y) y = pdata->inv_y - y;

	if (x < 0) x = 0;
	if (y < 0) y = 0;

	down = !(tssc_ctl & TSSC_CTL_PENUP_IRQ);
	ts->ts_down = down;

	/* no valid data */
	if (down && !(tssc_ctl & TSSC_CTL_DATA_FLAG))
		return IRQ_HANDLED;

	/* Debug:
	printk("%s: down=%d, x=%d, y=%d, z1=%d, z2=%d, status %x\n",
			__func__, down, x, y, z1, z2, tssc_status);
	*/

	if (down)
	{
        if ( 0 == z1 ) return IRQ_HANDLED;
		z = ( ( z2 - z1 - 2)*x) / ( z1 + 2 );
		z = ( 2500 - z ) * 1000 / ( 2500 - 900 );
		//printk("msm_ts_irq,z=%d,z1=%d,z2=%d,x=%d\n",z,z1,z2,x);
		if( z < 0 ) return IRQ_HANDLED;
	}

	/* Calibrate */
	x = (x * msm_tscal_xscale + y * msm_tscal_xymix + msm_tscal_xoffset ) / 65536;
	y = (y * msm_tscal_yscale + x * msm_tscal_yxmix + msm_tscal_yoffset ) / 65536;

	if (down)
	{
		input_report_abs(ts->input_dev, ABS_X, x);
		input_report_abs(ts->input_dev, ABS_Y, y);
		input_report_abs(ts->input_dev, ABS_PRESSURE, z);
		input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, 10);
		//printk("Reported values -> x: %d, y: %d, z: %d, z1: %d, z2: %d\n", x, y, z, z1, z2);
	}

	input_report_key(ts->input_dev, BTN_TOUCH, down);
	input_sync(ts->input_dev);

	return IRQ_HANDLED;
}

static int __devinit msm_ts_hw_init(struct msm_ts *ts)
{
	setup_next_sample(ts);

	return 0;
}

static void msm_ts_enable(struct msm_ts *ts, bool enable)
{
	uint32_t val;

	if (enable == true)
		msm_ts_hw_init(ts);
	else {
		val = tssc_readl(ts, TSSC_CTL);
		val &= ~TSSC_CTL_ENABLE;
		tssc_writel(ts, val, TSSC_CTL);
	}
}


#ifdef CONFIG_PM
static int
msm_ts_suspend(struct device *dev)
{
	struct msm_ts *ts =  dev_get_drvdata(dev);

	if (device_may_wakeup(dev) &&
			device_may_wakeup(dev->parent))
		enable_irq_wake(ts->sample_irq);
	else {
		disable_irq(ts->sample_irq);
		disable_irq(ts->pen_up_irq);
		msm_ts_enable(ts, false);
	}

	return 0;
}

static int
msm_ts_resume(struct device *dev)
{
	struct msm_ts *ts =  dev_get_drvdata(dev);

	if (device_may_wakeup(dev) &&
			device_may_wakeup(dev->parent))
		disable_irq_wake(ts->sample_irq);
	else {
		msm_ts_enable(ts, true);
		enable_irq(ts->sample_irq);
		enable_irq(ts->pen_up_irq);
	}

	return 0;
}

static struct dev_pm_ops msm_touchscreen_pm_ops = {
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= msm_ts_suspend,
	.resume		= msm_ts_resume,
#endif
};
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void msm_ts_early_suspend(struct early_suspend *h)
{
	struct msm_ts *ts = container_of(h, struct msm_ts, early_suspend);

	msm_ts_suspend(ts->dev);
}

static void msm_ts_late_resume(struct early_suspend *h)
{
	struct msm_ts *ts = container_of(h, struct msm_ts, early_suspend);

	msm_ts_resume(ts->dev);
}
#endif

#if defined(CONFIG_TOUCHSCREEN_VIRTUAL_KEYS)
#define virtualkeys virtualkeys.msm-touchscreen
#if defined(CONFIG_MACH_MOONCAKE)
static const char ts_keys_size[] = "0x01:102:30:350:40:60:0x01:139:120:350:50:60:0x01:158:210:350:40:60";
#elif defined(CONFIG_MACH_V9)
static const char ts_keys_size[] = "0x01:102:70:850:60:50:0x01:139:230:850:60:50:0x01:158:390:850:60:50";
#endif

static ssize_t virtualkeys_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	sprintf(buf,"%s\n",ts_keys_size);
	printk("wly:%s\n",__FUNCTION__);
	return strlen(ts_keys_size)+2;
}
static DEVICE_ATTR(virtualkeys, 0664, virtualkeys_show, NULL);
extern struct kobject *android_touch_kobj;
static struct kobject * virtual_key_kobj;
static int ts_key_report_init(void)
{
	int ret;
	virtual_key_kobj = kobject_get(android_touch_kobj);
	if (virtual_key_kobj == NULL) {
		virtual_key_kobj = kobject_create_and_add("board_properties", NULL);
		if (virtual_key_kobj == NULL) {
			printk(KERN_ERR "%s: subsystem_register failed\n", __func__);
			ret = -ENOMEM;
			return ret;
		}
	}
	ret = sysfs_create_file(virtual_key_kobj, &dev_attr_virtualkeys.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_file failed\n", __func__);
		return ret;
	}

	return 0;
}
#endif

static int __devinit msm_ts_probe(struct platform_device *pdev)
{
	struct msm_ts_platform_data *pdata = pdev->dev.platform_data;
	struct msm_ts *ts;
	struct resource *tssc_res;
	struct resource *irq1_res;
	struct resource *irq2_res;
	int err = 0;

	tssc_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "tssc");
	irq1_res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "tssc1");
	irq2_res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "tssc2");

	if (!tssc_res || !irq1_res || !irq2_res) {
		pr_err("%s: required resources not defined\n", __func__);
		return -ENODEV;
	}

	if (pdata == NULL) {
		pr_err("%s: missing platform_data\n", __func__);
		return -ENODEV;
	}

	ts = kzalloc(sizeof(struct msm_ts), GFP_KERNEL);
	if (ts == NULL) {
		pr_err("%s: No memory for struct msm_ts\n", __func__);
		return -ENOMEM;
	}
	ts->pdata = pdata;
	ts->dev	  = &pdev->dev;

	ts->sample_irq = irq1_res->start;
	ts->pen_up_irq = irq2_res->start;

	ts->tssc_base = ioremap(tssc_res->start, resource_size(tssc_res));
	if (ts->tssc_base == NULL) {
		pr_err("%s: Can't ioremap region (0x%08x - 0x%08x)\n", __func__,
		       (uint32_t)tssc_res->start, (uint32_t)tssc_res->end);
		err = -ENOMEM;
		goto err_ioremap_tssc;
	}

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		pr_err("failed to allocate touchscreen input device\n");
		err = -ENOMEM;
		goto err_alloc_input_dev;
	}
	ts->input_dev->name = "msm-touchscreen";
	input_set_drvdata(ts->input_dev, ts);

	input_set_capability(ts->input_dev, EV_KEY, BTN_TOUCH);
	set_bit(EV_ABS, ts->input_dev->evbit);

#if defined(CONFIG_TOUCHSCREEN_VIRTUAL_KEYS)
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_BACK, ts->input_dev->keybit);
#endif

	input_set_abs_params(ts->input_dev, ABS_X, pdata->min_x, pdata->max_x, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, pdata->min_y, pdata->max_y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, pdata->min_press, pdata->max_press, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, 245, 0, 0);

	err = input_register_device(ts->input_dev);
	if (err != 0) {
		pr_err("%s: failed to register input device\n", __func__);
		goto err_input_dev_reg;
	}

	msm_ts_hw_init(ts);

	err = request_irq(ts->sample_irq, msm_ts_irq,
			  (irq1_res->flags & ~IORESOURCE_IRQ) | IRQF_DISABLED,
			  "msm_touchscreen", ts);
	if (err != 0) {
		pr_err("%s: Cannot register irq1 (%d)\n", __func__, err);
		goto err_request_irq1;
	}

	err = request_irq(ts->pen_up_irq, msm_ts_irq,
			  (irq2_res->flags & ~IORESOURCE_IRQ) | IRQF_DISABLED,
			  "msm_touchscreen", ts);
	if (err != 0) {
		pr_err("%s: Cannot register irq2 (%d)\n", __func__, err);
		goto err_request_irq2;
	}

	platform_set_drvdata(pdev, ts);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +
						TSSC_SUSPEND_LEVEL;
	ts->early_suspend.suspend = msm_ts_early_suspend;
	ts->early_suspend.resume = msm_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	device_init_wakeup(&pdev->dev, pdata->can_wakeup);
	pr_info("%s: tssc_base=%p irq1=%d irq2=%d\n", __func__,
		ts->tssc_base, (int)ts->sample_irq, (int)ts->pen_up_irq);

#if defined(CONFIG_TOUCHSCREEN_VIRTUAL_KEYS)
	ts_key_report_init();
#endif
	return 0;

err_request_irq2:
	free_irq(ts->sample_irq, ts);

err_request_irq1:
	/* disable the tssc */
	tssc_writel(ts, TSSC_CTL_ENABLE, TSSC_CTL);

err_input_dev_reg:
	input_set_drvdata(ts->input_dev, NULL);
	input_free_device(ts->input_dev);

err_alloc_input_dev:
	iounmap(ts->tssc_base);

err_ioremap_tssc:
	kfree(ts);
	return err;
}

static struct platform_driver msm_touchscreen_driver = {
	.driver = {
		.name = "msm_touchscreen",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &msm_touchscreen_pm_ops,
#endif
	},
	.probe		= msm_ts_probe,
};

static int __init msm_ts_init(void)
{
	return platform_driver_register(&msm_touchscreen_driver);
}

static void __exit msm_ts_exit(void)
{
	platform_driver_unregister(&msm_touchscreen_driver);
}

module_init(msm_ts_init);
module_exit(msm_ts_exit);
MODULE_DESCRIPTION("Qualcomm MSM/QSD Touchscreen controller driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:msm_touchscreen");
