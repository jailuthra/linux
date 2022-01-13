// SPDX-License-Identifier: GPL-2.0
/*
 * Texas Instruments K3 RTC driver
 *
 * Copyright (C) 2021-2022 Texas Instruments Incorporated - https://www.ti.com/
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>

/* Registers */
#define REG_K3RTC_MOD_VER		0x00
#define REG_K3RTC_S_CNT_LSW		0x08
#define REG_K3RTC_S_CNT_MSW		0x0c
#define REG_K3RTC_COMP			0x10
#define REG_K3RTC_ON_OFF_S_CNT_LSW	0x20
#define REG_K3RTC_ON_OFF_S_CNT_MSW	0x24
#define REG_K3RTC_SCRATCH0		0x30
#define REG_K3RTC_SCRATCH7		0x4c
#define REG_K3RTC_GENERAL_CTL		0x50
#define REG_K3RTC_IRQSTATUS_RAW_SYS	0x54
#define REG_K3RTC_IRQSTATUS_SYS		0x58
#define REG_K3RTC_IRQENABLE_SET_SYS	0x5c
#define REG_K3RTC_IRQENABLE_CLR_SYS	0x60
#define REG_K3RTC_SYNCPEND		0x68
#define REG_K3RTC_KICK0			0x70
#define REG_K3RTC_KICK1			0x74

/* Control Bits for REG_K3RTC_MOD_VER */
#define K3RTC_MINOR_REVISION_MASK	GENMASK(5, 0)
#define K3RTC_CUSTOM_REVISION_MASK	GENMASK(7, 6)
#define K3RTC_MAJOR_REVISION_MASK	GENMASK(10, 8)
#define K3RTC_RTL_VERSION_MASK		GENMASK(15, 11)
#define K3RTC_MODULE_ID_MASK		GENMASK(31, 16)

/* Control Bits for REG_K3RTC_S_CNT_LSW */
#define K3RTC_S_CNT_LSW_MASK		GENMASK(31, 0)

/* Control Bits for REG_K3RTC_S_CNT_MSW */
#define K3RTC_S_CNT_MSW_MASK		GENMASK(15, 0)

/* Control Bits for REG_K3RTC_GENERAL_CTL */
#define K3RTC_O32K_OSC_DEP_EN_BIT	BIT(21)
#define K3RTC_UNLOCK_BIT		BIT(23)
#define K3RTC_CNT_FMODE_MASK		GENMASK(25, 24)
#define K3RTC_CNT_FMODE_S_CNT_VALUE	(0x2 << 24)

/* Control Bits for REG_K3RTC_IRQ* */
#define K3RTC_EVENT_ON_OFF_BIT		BIT(0)
#define K3RTC_EVENT_OFF_ON_BIT		BIT(1)

/* Control Bits for REG_K3RTC_SYNCPEND */
#define K3RTC_WR_PEND_BIT		BIT(0)
#define K3RTC_RD_PEND_BIT		BIT(1)
#define K3RTC_RELOAD_FROM_BBD_BIT	BIT(31)

/* Magic values for lock/unlock */
#define K3RTC_KICK0_UNLOCK_VALUE	0x83e70b13
#define K3RTC_KICK1_UNLOCK_VALUE	0x95a4f1e0

/* Multiplier for ppb conversions */
#define K3RTC_PPB_MULT			(1000000000LL)
/* Min and max values supported with 'offset' interface (swapped sign) */
#define K3RTC_MIN_OFFSET		(-277761)
#define K3RTC_MAX_OFFSET		(277778)

/**
 * struct ti_k3_rtc - Private data for ti-k3-rtc
 * @irq:		IRQ
 * @sync_timeout_us:	data sync timeout period in uSec (2 * 32k period)
 * @rate_32k:		32k clock rate in Hz
 * @rtc_dev:		rtc device
 * @rtc_base:		rtc base address
 */
struct ti_k3_rtc {
	int irq;
	u32 sync_timeout_us;
	unsigned long rate_32k;
	struct rtc_device *rtc_dev;
	void __iomem *rtc_base;
};

static inline u32 k3rtc_readl(struct ti_k3_rtc *priv, u32 offset)
{
	return readl(priv->rtc_base + offset);
}

static inline void k3rtc_writel(struct ti_k3_rtc *priv, u32 offset, u32 value)
{
	writel(value, priv->rtc_base + offset);
}

/**
 * k3rtc_fence  - Ensure a register sync took place between the two domains
 * @priv:      pointer to priv data
 *
 * Return: 0 if the sync took place, else returns -ETIMEDOUT
 */
static int k3rtc_fence(struct ti_k3_rtc *priv)
{
	u32 timeout = priv->sync_timeout_us;
	u32 mask = K3RTC_RD_PEND_BIT | K3RTC_WR_PEND_BIT;
	u32 val = 0;

	while (timeout--) {
		val = k3rtc_readl(priv, REG_K3RTC_SYNCPEND);
		if (!(val & mask))
			return 0;
		usleep_range(1, 2);
	}

	pr_err("RTC Fence timeout: 0x%08x\n", val);
	return -ETIMEDOUT;
}

static inline int k3rtc_check_unlocked(struct ti_k3_rtc *priv)
{
	u32 val;

	val = k3rtc_readl(priv, REG_K3RTC_GENERAL_CTL);
	return (val & K3RTC_UNLOCK_BIT) ? 0 : 1;
}

static int k3rtc_unlock_rtc(struct ti_k3_rtc *priv)
{
	u32 timeout = priv->sync_timeout_us;
	int ret;

	ret = k3rtc_check_unlocked(priv);
	if (ret <= 0)
		return ret;

	k3rtc_writel(priv, REG_K3RTC_KICK0, K3RTC_KICK0_UNLOCK_VALUE);
	k3rtc_writel(priv, REG_K3RTC_KICK1, K3RTC_KICK1_UNLOCK_VALUE);

	/* Skip fence since we are going to check the unlock bit as fence */
	while (timeout--) {
		ret = k3rtc_check_unlocked(priv);
		if (ret <= 0)
			return ret;
		usleep_range(1, 2);
	}

	return -ETIMEDOUT;
}

static int k3rtc_configure(struct device *dev)
{
	int ret;
	u32 ctl;
	struct ti_k3_rtc *priv = dev_get_drvdata(dev);

	/* TBD: Introduce proper erratum check for SR revision */
	ret = k3rtc_check_unlocked(priv);
	/* If there is an error OR if we are locked, return error */
	if (ret) {
		dev_err(dev, HW_ERR "Erratum unlock QUIRK! Cannot operate!!\n");
		return -EFAULT;
	}

	/* May Need to explicitly unlock first time */
	ret = k3rtc_unlock_rtc(priv);
	if (ret) {
		dev_err(dev, "Failed to unlock(%d)!\n", ret);
		return ret;
	}

	/* Enable Shadow register sync on 32k clk boundary */
	ctl = k3rtc_readl(priv, REG_K3RTC_GENERAL_CTL);
	ctl |= K3RTC_O32K_OSC_DEP_EN_BIT;
	k3rtc_writel(priv, REG_K3RTC_GENERAL_CTL, ctl);

	/*
	 * Wait at least 2 clk sync time before proceeding further programming.
	 * This ensures that the sync has switched over.
	 */
	usleep_range(priv->sync_timeout_us, priv->sync_timeout_us + 5);

	/* We need to ensure fence here to make sure sync here */
	ret = k3rtc_fence(priv);
	if (ret) {
		dev_err(dev, "Failed fence osc_dep enable(%d) - is 32k clk working?!\n", ret);
		return ret;
	}

	/* Lets just make sure we get consistent time value */
	ctl &= ~K3RTC_CNT_FMODE_MASK;
	/*
	 * reading lower seconds will freeze value on higher seconds
	 * This also implies that we must *ALWAYS* read lower seconds
	 * prior to reading higher seconds
	 */
	ctl |= K3RTC_CNT_FMODE_S_CNT_VALUE;
	k3rtc_writel(priv, REG_K3RTC_GENERAL_CTL, ctl);

	/* Clear any spurious IRQ sources if any */
	k3rtc_writel(priv, REG_K3RTC_IRQSTATUS_SYS,
		     K3RTC_EVENT_ON_OFF_BIT | K3RTC_EVENT_OFF_ON_BIT);
	/* Disable all IRQs */
	k3rtc_writel(priv, REG_K3RTC_IRQENABLE_CLR_SYS,
		     K3RTC_EVENT_ON_OFF_BIT | K3RTC_EVENT_OFF_ON_BIT);

	/* And.. Let us Sync the writes in */
	ret = k3rtc_fence(priv);
	if (ret) {
		dev_err(dev, "Failed to fence(%d)!\n",  ret);
		return ret;
	}

	return ret;
}

static int ti_k3_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct ti_k3_rtc *priv = dev_get_drvdata(dev);
	u32 seconds_lo, seconds_hi;

	seconds_lo = k3rtc_readl(priv, REG_K3RTC_S_CNT_LSW);
	seconds_hi = k3rtc_readl(priv, REG_K3RTC_S_CNT_MSW);

	rtc_time64_to_tm((((time64_t)seconds_hi) << 32) | (time64_t)seconds_lo, tm);

	return 0;
}

static int ti_k3_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	int ret;
	struct ti_k3_rtc *priv = dev_get_drvdata(dev);
	time64_t seconds;
	u32 seconds_lo, seconds_hi;

	seconds = rtc_tm_to_time64(tm);

	seconds_lo = seconds & K3RTC_S_CNT_LSW_MASK;
	seconds_hi = (seconds >> 32) & K3RTC_S_CNT_MSW_MASK;

	k3rtc_writel(priv, REG_K3RTC_S_CNT_LSW, seconds_lo);
	k3rtc_writel(priv, REG_K3RTC_S_CNT_MSW, seconds_hi);

	ret = k3rtc_fence(priv);
	if (ret) {
		dev_err(dev, "Failed to fence(%d)!\n",  ret);
		return ret;
	}

	return 0;
}

static int ti_k3_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct ti_k3_rtc *priv = dev_get_drvdata(dev);
	u32 seconds_lo, seconds_hi;
	u32 alm;

	seconds_lo = k3rtc_readl(priv, REG_K3RTC_ON_OFF_S_CNT_LSW);
	seconds_hi = k3rtc_readl(priv, REG_K3RTC_ON_OFF_S_CNT_MSW);

	rtc_time64_to_tm((((time64_t)seconds_hi) << 32) | (time64_t)seconds_lo, &alarm->time);

	alm = k3rtc_readl(priv, REG_K3RTC_IRQENABLE_SET_SYS);
	alarm->enabled = (alm & K3RTC_EVENT_ON_OFF_BIT) ? 1 : 0;

	return 0;
}

static int ti_k3_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	int ret;
	struct ti_k3_rtc *priv = dev_get_drvdata(dev);
	u32 reg;
	u32 offset =
	    enabled ? REG_K3RTC_IRQENABLE_SET_SYS : REG_K3RTC_IRQENABLE_CLR_SYS;

	reg = k3rtc_readl(priv, REG_K3RTC_IRQENABLE_SET_SYS);
	reg &= K3RTC_EVENT_ON_OFF_BIT;
	if ((enabled && reg) || (!enabled && !reg))
		return 0;

	k3rtc_writel(priv, offset, K3RTC_EVENT_ON_OFF_BIT);

	/*
	 * Ensure the write sync is through - NOTE: it should be OK to have
	 * isr to fire as we are checking sync (which should be done in a 32k
	 * cycle or so).
	 */
	ret = k3rtc_fence(priv);
	if (ret) {
		dev_err(dev, "Failed to fence(%d)!\n",  ret);
		return ret;
	}

	return 0;
}

static int ti_k3_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct ti_k3_rtc *priv = dev_get_drvdata(dev);
	time64_t seconds;
	u32 seconds_lo, seconds_hi;
	int ret;

	seconds = rtc_tm_to_time64(&alarm->time);

	seconds_lo = seconds & K3RTC_S_CNT_LSW_MASK;
	seconds_hi = (seconds >> 32) & K3RTC_S_CNT_MSW_MASK;

	k3rtc_writel(priv, REG_K3RTC_ON_OFF_S_CNT_LSW, seconds_lo);
	k3rtc_writel(priv, REG_K3RTC_ON_OFF_S_CNT_MSW, seconds_hi);

	/* Make sure the alarm time is synced in */
	ret = k3rtc_fence(priv);
	if (ret) {
		dev_err(dev, "Failed to fence(%d)!\n",  ret);
		return ret;
	}

	/* Alarm irq enable will do a sync */
	return ti_k3_rtc_alarm_irq_enable(dev, alarm->enabled);
}

static int ti_k3_rtc_read_offset(struct device *dev, long *offset)
{
	struct ti_k3_rtc *priv = dev_get_drvdata(dev);
	u32 ticks_per_hr = priv->rate_32k * 3600;
	int comp;
	s64 tmp;

	comp = k3rtc_readl(priv, REG_K3RTC_COMP);

	/* Convert from RTC calibration register format to ppb format */
	tmp = comp * (s64)K3RTC_PPB_MULT;
	if (tmp < 0)
		tmp -= ticks_per_hr / 2LL;
	else
		tmp += ticks_per_hr / 2LL;
	tmp = div_s64(tmp, ticks_per_hr);

	/* Offset value operates in negative way, so swap sign */
	*offset = (long)-tmp;

	return 0;
}

static int ti_k3_rtc_set_offset(struct device *dev, long offset)
{
	struct ti_k3_rtc *priv = dev_get_drvdata(dev);
	u32 ticks_per_hr = priv->rate_32k * 3600;
	int comp;
	s64 tmp;

	/* Make sure offset value is within supported range */
	if (offset < K3RTC_MIN_OFFSET || offset > K3RTC_MAX_OFFSET)
		return -ERANGE;

	/* Convert from ppb format to RTC calibration register format */
	tmp = offset * (s64)ticks_per_hr;
	if (tmp < 0)
		tmp -= K3RTC_PPB_MULT / 2LL;
	else
		tmp += K3RTC_PPB_MULT / 2LL;
	tmp = div_s64(tmp, K3RTC_PPB_MULT);

	/* Offset value operates in negative way, so swap sign */
	comp = (int)-tmp;

	k3rtc_writel(priv, REG_K3RTC_COMP, comp);

	return k3rtc_fence(priv);
}

static irqreturn_t ti_k3_rtc_interrupt(s32 irq, void *dev_id)
{
	struct device *dev = dev_id;
	struct ti_k3_rtc *priv = dev_get_drvdata(dev);
	u32 reg;
	u32 timeout;
	int ret;

	/*
	 * IRQ assertion can be very fast, however, the IRQ Status clear
	 * de-assert depends on 32k clock edge in the 32k domain
	 * If we clear the status prior to the first 32k clock edge,
	 * the status bit is cleared, but the IRQ is stays re-asserted.
	 *
	 * To prevent this condition, we need to wait for one 32k clock cycle
	 * We can either do that by polling the 32k observability signal for
	 * a toggle OR we could just sleep and let the processor do other
	 * stuff. We use sync_timeout_us which is 2x32k clock period to
	 * make sure.
	 */
	usleep_range(priv->sync_timeout_us, priv->sync_timeout_us + 2);

	/* Lets make sure that this is a valid interrupt */
	reg = k3rtc_readl(priv, REG_K3RTC_IRQSTATUS_SYS);

	if (!(reg & K3RTC_EVENT_ON_OFF_BIT)) {
		u32 raw = k3rtc_readl(priv, REG_K3RTC_IRQSTATUS_RAW_SYS);

		dev_err(dev, "erratum / false interrupt? IRQ status: 0x%08x / 0x%08x\n",
			reg, raw);
		return IRQ_NONE;
	}

	/* Write 1 to clear status reg */
	k3rtc_writel(priv, REG_K3RTC_IRQSTATUS_SYS, K3RTC_EVENT_ON_OFF_BIT);

	/* Sync the write in */
	ret = k3rtc_fence(priv);
	if (ret) {
		dev_err(dev, "Failed to fence irq status clr(%d)!\n",  ret);
		return IRQ_NONE;
	}

	/*
	 * Force the 32k status to be reloaded back in to ensure status is
	 * reflected back correctly.
	 */
	reg = k3rtc_readl(priv, REG_K3RTC_SYNCPEND);
	if (ret) {
		dev_err(dev, "sync read fail:%d\n",  ret);
		return IRQ_NONE;
	}
	reg |= K3RTC_RELOAD_FROM_BBD_BIT;
	k3rtc_writel(priv, REG_K3RTC_SYNCPEND, reg);

	/* Ensure the write sync is through */
	ret = k3rtc_fence(priv);
	if (ret) {
		dev_err(dev, "Failed to fence reload from bbd(%d)!\n",  ret);
		return IRQ_NONE;
	}

	/* Now we ensure that the status bit is cleared */
	timeout = priv->sync_timeout_us;
	while (timeout--) {
		reg = k3rtc_readl(priv, REG_K3RTC_IRQSTATUS_SYS);
		if (!(reg & K3RTC_EVENT_ON_OFF_BIT))
			break;

		usleep_range(1, 2);
	}
	if (!timeout) {
		dev_err(dev, "Time out waiting for status clear: 0x%08x\n", reg);
		return IRQ_NONE;
	}

	/* Notify RTC core on event */
	rtc_update_irq(priv->rtc_dev, 1, RTC_IRQF | RTC_AF);

	return IRQ_HANDLED;
}

static const struct rtc_class_ops ti_k3_rtc_ops = {
	.read_time = ti_k3_rtc_read_time,
	.set_time = ti_k3_rtc_set_time,
	.read_alarm = ti_k3_rtc_read_alarm,
	.set_alarm = ti_k3_rtc_set_alarm,
	.read_offset = ti_k3_rtc_read_offset,
	.set_offset = ti_k3_rtc_set_offset,
	.alarm_irq_enable = ti_k3_rtc_alarm_irq_enable,
};

static int ti_k3_rtc_scratch_read(void *priv_data, unsigned int offset,
				  void *_val, size_t bytes)
{
	struct ti_k3_rtc *priv = (struct ti_k3_rtc *)priv_data;
	u32 *val = _val;
	int i;

	for (i = 0; i < bytes / 4; i++) {
		val[i] = k3rtc_readl(priv,
				     REG_K3RTC_SCRATCH0 + offset + (i * 4));
	}

	return 0;
}

static int ti_k3_rtc_scratch_write(void *priv_data, unsigned int offset,
				   void *_val, size_t bytes)
{
	struct ti_k3_rtc *priv = (struct ti_k3_rtc *)priv_data;
	u32 *val = _val;
	int i;

	for (i = 0; i < bytes / 4; i++) {
		k3rtc_writel(priv, REG_K3RTC_SCRATCH0 + offset + (i * 4),
			     val[i]);
	}
	return k3rtc_fence(priv);
}

static struct nvmem_config ti_k3_rtc_nvmem_config = {
	.name = "ti_k3_rtc_scratch",
	.word_size = 4,
	.stride = 4,
	.size = REG_K3RTC_SCRATCH7 - REG_K3RTC_SCRATCH0 + 4,
	.reg_read = ti_k3_rtc_scratch_read,
	.reg_write = ti_k3_rtc_scratch_write,
};

static int k3rtc_get_32kclk(struct device *dev, struct ti_k3_rtc *priv)
{
	int ret;
	struct clk *clk;

	clk = devm_clk_get(dev, "osc32k");
	if (IS_ERR(clk)) {
		dev_err(dev, "No input reference 32k clock\n");
		return PTR_ERR(clk);
	}

	ret = clk_prepare_enable(clk);
	if (ret) {
		dev_err(dev, "Failed to enable the reference 32k clock(%d)\n", ret);
		return ret;
	}

	ret = devm_add_action_or_reset(dev, (void (*)(void *))clk_disable_unprepare, clk);
	if (ret)
		return ret;

	priv->rate_32k = clk_get_rate(clk);

	/* Make sure we are exact 32k clock. Else, try to compensate delay */
	if (priv->rate_32k != 32768)
		dev_warn(dev, "Clock rate %ld is not 32768! Could misbehave!\n", priv->rate_32k);

	/* Max sync timeout will be two 32k clk sync cycles = ~61uS */
	priv->sync_timeout_us = (u32)(DIV_ROUND_UP_ULL(1000000, priv->rate_32k) * 2);

	return ret;
}

static int k3rtc_get_vbusclk(struct device *dev, struct ti_k3_rtc *priv)
{
	int ret;
	struct clk *clk;

	clk = devm_clk_get(dev, "vbus");
	if (IS_ERR(clk)) {
		dev_err(dev, "No input vbus clock\n");
		return PTR_ERR(clk);
	}

	ret = clk_prepare_enable(clk);
	if (ret) {
		dev_err(dev, "Failed to enable the vbus clock(%d)\n", ret);
		return ret;
	}

	ret = devm_add_action_or_reset(dev, (void (*)(void *))clk_disable_unprepare, clk);
	return ret;
}

static int ti_k3_rtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ti_k3_rtc *priv;
	int ret;
	u32 rev;

	priv = devm_kzalloc(dev, sizeof(struct ti_k3_rtc), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->rtc_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->rtc_base))
		return PTR_ERR(priv->rtc_base);

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0)
		return -EINVAL;

	priv->rtc_dev = devm_rtc_allocate_device(dev);
	if (IS_ERR(priv->rtc_dev))
		return PTR_ERR(priv->rtc_dev);

	priv->rtc_dev->ops = &ti_k3_rtc_ops;
	priv->rtc_dev->range_max = (1ULL << 48) - 1;	/* 48Bit seconds */
	ti_k3_rtc_nvmem_config.priv = priv;

	ret = devm_request_threaded_irq(dev, priv->irq, NULL,
					ti_k3_rtc_interrupt,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					dev_name(dev), dev);
	if (ret) {
		dev_err(dev, "Could not request IRQ: %d\n", ret);
		return ret;
	}

	ret = k3rtc_get_32kclk(dev, priv);
	if (ret)
		return ret;
	ret = k3rtc_get_vbusclk(dev, priv);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, priv);

	rev = k3rtc_readl(priv, REG_K3RTC_MOD_VER);

	dev_dbg(dev, "Detected revision: %ld.%ld.%ld/RTL 0x%02lx/0x%04lx\n",
		FIELD_GET(K3RTC_MAJOR_REVISION_MASK, rev),
		FIELD_GET(K3RTC_CUSTOM_REVISION_MASK, rev),
		FIELD_GET(K3RTC_MINOR_REVISION_MASK, rev),
		FIELD_GET(K3RTC_RTL_VERSION_MASK, rev),
		FIELD_GET(K3RTC_MODULE_ID_MASK, rev));

	ret = k3rtc_configure(dev);
	if (ret)
		return ret;

	if (device_property_present(dev, "wakeup-source"))
		device_init_wakeup(dev, true);
	else
		device_set_wakeup_capable(dev, true);

	ret = rtc_register_device(priv->rtc_dev);
	if (ret)
		return ret;

	ret = rtc_nvmem_register(priv->rtc_dev, &ti_k3_rtc_nvmem_config);
	return ret;
}

static const struct of_device_id ti_k3_rtc_of_match_table[] = {
	{.compatible = "ti,am62-rtc"},
	{}
};
MODULE_DEVICE_TABLE(of, ti_k3_rtc_of_match_table);

#ifdef CONFIG_PM_SLEEP
static int ti_k3_rtc_suspend(struct device *dev)
{
	struct ti_k3_rtc *priv = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(priv->irq);
	return 0;
}

static int ti_k3_rtc_resume(struct device *dev)
{
	struct ti_k3_rtc *priv = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(priv->irq);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(ti_k3_rtc_pm_ops, ti_k3_rtc_suspend, ti_k3_rtc_resume);

static struct platform_driver ti_k3_rtc_driver = {
	.probe = ti_k3_rtc_probe,
	.driver = {
		   .name = "rtc-ti-k3",
		   .of_match_table = ti_k3_rtc_of_match_table,
		   .pm = &ti_k3_rtc_pm_ops,
		   },
};
module_platform_driver(ti_k3_rtc_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TI K3 RTC driver");
MODULE_AUTHOR("Nishanth Menon");
MODULE_ALIAS("platform:rtc-ti-k3");
