// SPDX-License-Identifier: GPL-2.0
/*
 * dwc3-am62.c - TI specific Glue layer for AM62 DWC3 USB Controller
 *
 * Copyright (C) 2021 Texas Instruments Incorporated - https://www.ti.com
 */

#include <linux/extcon-provider.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/extcon.h>
#include <linux/workqueue.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/regmap.h>
#include <linux/pinctrl/consumer.h>

/* USB WRAPPER register offsets */
#define USBSS_PID			0x0
#define USBSS_OVERCURRENT_CTRL		0x4
#define USBSS_PHY_CONFIG		0x8
#define USBSS_PHY_TEST			0xc
#define USBSS_CORE_STAT			0x14
#define USBSS_HOST_VBUS_CTRL		0x18
#define USBSS_MODE_CONTROL		0x1c
#define USBSS_WAKEUP_CONFIG		0x30
#define USBSS_WAKEUP_STAT		0x34
#define USBSS_OVERRIDE_CONFIG		0x38
#define USBSS_IRQ_MISC_STATUS_RAW	0x430
#define USBSS_IRQ_MISC_STATUS		0x434
#define USBSS_IRQ_MISC_ENABLE_SET	0x438
#define USBSS_IRQ_MISC_ENABLE_CLR	0x43c
#define USBSS_IRQ_MISC_EOI		0x440
#define USBSS_INTR_TEST			0x490
#define USBSS_VBUS_FILTER		0x614
#define USBSS_VBUS_STAT			0x618
#define USBSS_DEBUG_CFG			0x708
#define USBSS_DEBUG_DATA		0x70c
#define USBSS_HOST_HUB_CTRL		0x714

/* PHY CONFIG register bits */
#define USBSS_PHY_VBUS_SEL_MASK		GENMASK(2, 1)
#define USBSS_PHY_VBUS_SEL_SHIFT	1
#define USBSS_PHY_LANE_REVERSE		BIT(0)

/* MODE CONTROL register bits */
#define USBSS_MODE_VALID	BIT(0)

/* WAKEUP CONFIG register bits */
#define USBSS_WAKEUP_CFG_OVERCURRENT_EN	BIT(3)
#define USBSS_WAKEUP_CFG_LINESTATE_EN	BIT(2)
#define USBSS_WAKEUP_CFG_SESSVALID_EN	BIT(1)
#define USBSS_WAKEUP_CFG_VBUSVALID_EN	BIT(0)

/* WAKEUP STAT register bits */
#define USBSS_WAKEUP_STAT_OVERCURRENT	BIT(4)
#define USBSS_WAKEUP_STAT_LINESTATE	BIT(3)
#define USBSS_WAKEUP_STAT_SESSVALID	BIT(2)
#define USBSS_WAKEUP_STAT_VBUSVALID	BIT(1)
#define USBSS_WAKEUP_STAT_CLR		BIT(0)

/* IRQ_MISC_STATUS_RAW register bits */
#define USBSS_IRQ_MISC_RAW_VBUSVALID	BIT(22)
#define USBSS_IRQ_MISC_RAW_SESSVALID	BIT(20)

/* IRQ_MISC_STATUS register bits */
#define USBSS_IRQ_MISC_VBUSVALID	BIT(22)
#define USBSS_IRQ_MISC_SESSVALID	BIT(20)

/* IRQ_MISC_ENABLE_SET register bits */
#define USBSS_IRQ_MISC_ENABLE_SET_VBUSVALID	BIT(22)
#define USBSS_IRQ_MISC_ENABLE_SET_SESSVALID	BIT(20)

/* IRQ_MISC_ENABLE_CLR register bits */
#define USBSS_IRQ_MISC_ENABLE_CLR_VBUSVALID	BIT(22)
#define USBSS_IRQ_MISC_ENABLE_CLR_SESSVALID	BIT(20)

/* IRQ_MISC_EOI register bits */
#define USBSS_IRQ_MISC_EOI_VECTOR	BIT(0)

/* VBUS_STAT register bits */
#define USBSS_VBUS_STAT_SESSVALID	BIT(2)
#define USBSS_VBUS_STAT_VBUSVALID	BIT(0)

/* Mask for PHY PLL REFCLK */
#define PHY_PLL_REFCLK_MASK	GENMASK(3, 0)

enum role {
	DEVICE,
	HOST,
};

struct id_gpio_data {
	struct gpio_desc *gpiod;
	unsigned long debounce;
	int id_irq;
};

struct dwc3_data {
	struct device *dev;
	void __iomem *usbss;
	struct clk *usb2_refclk;
	struct extcon_dev *edev;
	unsigned int extcon_id;
	struct id_gpio_data *id_gpio;
	struct delayed_work work;
	int vbus_irq;
	int rate_code;
	enum role current_role;
	int connect;
	struct regmap *syscon;
	unsigned int offset;
	unsigned long debounce_jiffies; /* No idea how this is being populated !!!!! */
	unsigned int vbus_divider;
};

static const int dwc3_ti_rate_table[] = {	/* in KHZ */
	9600,
	10000,
	12000,
	19200,
	20000,
	24000,
	25000,
	26000,
	38400,
	40000,
	58000,
	50000,
	52000,
};

static const unsigned int usb_extcon_cable[] = {
        EXTCON_USB,
        EXTCON_USB_HOST,
        EXTCON_NONE,
};

static inline u32 dwc3_ti_readl(struct dwc3_data *data, u32 offset)
{
	return readl((data->usbss) + offset);
}

static inline void dwc3_ti_writel(struct dwc3_data *data, u32 offset, u32 value)
{
	writel(value, (data->usbss) + offset);
}

static void role_detect_work(struct work_struct *work)
{
	struct dwc3_data *data =
		container_of(to_delayed_work(work), struct dwc3_data,
			     work);
	u32 vbus_state, reg;
	int id_state = 0;
	enum role detect_role;
	int connect = 0;

	/* Read the status for VBUS valid register */
	vbus_state = dwc3_ti_readl(data, USBSS_VBUS_STAT);
	vbus_state = vbus_state & USBSS_VBUS_STAT_VBUSVALID;

	/* Read the status of ID GPIO */
	if (data->id_gpio)
		id_state = gpiod_get_value_cansleep(data->id_gpio->gpiod);

	/*
	 * ID | VBUS | STATE
	 * -----------------
	 * 0  | 0    | HOST
	 * 0  | 1    | HOST
         * 1  | 0    | Disconnected (set mode valid to 0)
         * 1  | 1    | Device
	 *
	 */

	/* detemine the role or find out if disconnect event */
	if (id_state) {
		detect_role = DEVICE;
		if (vbus_state)
			connect = 1;
	} else {
		detect_role = HOST;
		connect = 1;
	}

	/* Set or clear mode valid bit based on connect or disconnect even */
	if (data->connect != connect) {
		reg = dwc3_ti_readl(data, USBSS_MODE_CONTROL);
		if (connect)
			reg |= USBSS_MODE_VALID;
		else
			reg &= ~USBSS_MODE_VALID;

		dwc3_ti_writel(data, USBSS_MODE_CONTROL, reg);
		data->connect = connect;
	}

	/* send extcon syc signal to the dwc3 controller driver */
	if (data->current_role != detect_role) {

		if (detect_role == HOST) {
			extcon_set_state_sync(data->edev, EXTCON_USB, false);
			extcon_set_state_sync(data->edev, EXTCON_USB_HOST, true);
		} else {
			extcon_set_state_sync(data->edev, EXTCON_USB_HOST, false);
			extcon_set_state_sync(data->edev, EXTCON_USB, true);
		}

		data->current_role = detect_role;
	}

	return;
}

static irqreturn_t role_detect_irq_handler(int irq, void *dev_id)
{
	struct dwc3_data *data = dev_id;
	u32 reg;

	/* Clearing VBUS interrupt always as the the same work queue is used in both cases */
	reg = dwc3_ti_readl(data, USBSS_IRQ_MISC_STATUS);
	reg |= USBSS_IRQ_MISC_VBUSVALID;
	dwc3_ti_writel(data, USBSS_IRQ_MISC_STATUS, reg);

	queue_delayed_work(system_power_efficient_wq, &data->work,
			   data->debounce_jiffies);
	reg = dwc3_ti_readl(data, USBSS_IRQ_MISC_EOI);
	reg &= ~USBSS_IRQ_MISC_EOI_VECTOR;
	dwc3_ti_writel(data, USBSS_IRQ_MISC_EOI, reg);
	return IRQ_HANDLED;
}

static int dwc3_ti_vbus_irq_setup(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dwc3_data *data = platform_get_drvdata(pdev);
	int irq, ret;

	/* Get the misc interrupt */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(dev, irq, role_detect_irq_handler, IRQF_SHARED,
			       dev_name(dev), data);
	if (ret) {
		dev_err(dev, "failed to required IRQ #%d-->%d\n",
			irq, ret);
		return ret;
	}

	data->vbus_irq = irq;

	return 0;
}

/* Allocate the memory for extcon device and register extcond device */
static int dwc3_ti_register_extcon(struct dwc3_data *data)
{
	int ret;

	data->edev = devm_extcon_dev_allocate(data->dev, usb_extcon_cable);
	if (IS_ERR(data->edev)) {
		dev_err(data->dev, "failed to allocate extcon device\n");
		return -ENOMEM;
	}

	ret = devm_extcon_dev_register(data->dev, data->edev);
	if (ret < 0)
		return ret;

	return 0;
}

static int dwc3_ti_get_id_gpio(struct dwc3_data *data)
{
	struct id_gpio_data *id_gpio;
	struct device	*dev = data->dev;
	unsigned long irq_flags;
	int irq, ret;

	/* create the extcon device */
	id_gpio = devm_kzalloc(dev, sizeof(struct id_gpio_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/* Read id-gpio property */
	id_gpio->gpiod = devm_gpiod_get_optional(dev, "id", GPIOD_IN);
	if (IS_ERR(id_gpio->gpiod))
		return PTR_ERR(id_gpio->gpiod);

	/* If id-gpio property is not present */
	if (!id_gpio->gpiod)
		return 0;

	irq = gpiod_to_irq(id_gpio->gpiod);
	if (irq <= 0)
		return irq;
	id_gpio->id_irq = irq;

	/* SET irq flags
	 * Use both edges for detecting connect and disconnect events
	 */
	irq_flags = IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING;

	/* Setup the irq handler for both events */
	ret = devm_request_any_context_irq(dev, irq,
					   role_detect_irq_handler, irq_flags,
					   "id_gpio", data);
	if (ret < 0)
		return ret;

	/* To be enabled only after probing dwc3 core */
	disable_irq(irq);
	data->id_gpio = id_gpio;
	return 0;
}

static int phy_syscon_pll_refclk(struct dwc3_data *data)
{
	struct device *dev = data->dev;
	struct device_node *node = dev->of_node;
	struct of_phandle_args args;
	struct regmap *syscon;
	int ret;

	syscon = syscon_regmap_lookup_by_phandle(node, "ti,syscon-phy-pll-refclk");
	if (IS_ERR(syscon)) {
		dev_err(dev, "unable to get ti,syscon-phy-pll-refclk regmap\n");
		return PTR_ERR(syscon);
	}

	data->syscon = syscon;

	ret = of_parse_phandle_with_fixed_args(node, "ti,syscon-phy-pll-refclk", 1,
					       0, &args);
	if (ret)
		return ret;

	data->offset = args.args[0];

	ret = regmap_update_bits(data->syscon, data->offset, PHY_PLL_REFCLK_MASK, data->rate_code);
	if (ret) {
		dev_err(dev, "failed to set phy pll reference clock rate\n");
		return ret;
	}

	return 0;
}

static void dwc3_ti_enable_irqs(struct dwc3_data *data)
{
	u32 reg;

	/* enable VBUSVALID interrupt */
	reg = dwc3_ti_readl(data, USBSS_IRQ_MISC_ENABLE_SET);
	reg |= USBSS_IRQ_MISC_ENABLE_SET_VBUSVALID;
	dwc3_ti_writel(data, USBSS_IRQ_MISC_ENABLE_SET, reg);

	/* enable id-gpio interrupt */
	if (data->id_gpio) {
		enable_irq(data->id_gpio->id_irq);
	}

	return;
}

static void dwc3_ti_disable_irqs(struct dwc3_data *data)
{
	u32 reg;

	/* disable VBUSVALID interrupt */
	reg = dwc3_ti_readl(data, USBSS_IRQ_MISC_ENABLE_CLR);
	reg |= USBSS_IRQ_MISC_ENABLE_CLR_VBUSVALID;
	dwc3_ti_writel(data, USBSS_IRQ_MISC_ENABLE_CLR, reg);

	/* disable id-gpio interrupt */
	if (data->id_gpio)
		disable_irq(data->id_gpio->id_irq);

	return;
}

static int dwc3_ti_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = pdev->dev.of_node;
	struct dwc3_data *data;
	int i, ret;
	unsigned long rate;
	u32 reg;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	platform_set_drvdata(pdev, data);

	data->usbss = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(data->usbss)) {
		dev_err(dev, "can't map IOMEM resource\n");
		return PTR_ERR(data->usbss);
	}

        pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	/*
	 * Don't ignore its dependencies with its children
	 */
	pm_suspend_ignore_children(dev, false);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "pm_runtime_get_sync failed: %d\n", ret);
		goto err;
	}

	data->usb2_refclk = devm_clk_get(dev, "ref");
	if (IS_ERR(data->usb2_refclk)) {
		dev_err(dev, "can't get usb2_refclk\n");
		return PTR_ERR(data->usb2_refclk);
	}

	// Calcuate the rate code
	rate = clk_get_rate(data->usb2_refclk);
	rate /= 1000;	// To KHz
	for (i = 0; i < ARRAY_SIZE(dwc3_ti_rate_table); i++) {
		if (dwc3_ti_rate_table[i] == rate)
			break;
	}

	if (i == ARRAY_SIZE(dwc3_ti_rate_table)) {
		dev_err(dev, "unsupported usb2_refclk rate: %lu KHz\n", rate);
		ret = -EINVAL;
		goto err_clk_disable;
	}

	data->rate_code = i;

	// Read the syscon property
	ret = phy_syscon_pll_refclk(data);
	if (ret)
		goto err_clk_disable;

	// VBUS divider select
	data->vbus_divider = device_property_read_bool(dev, "ti,vbus-divider");
	reg = dwc3_ti_readl(data, USBSS_PHY_CONFIG);
	if (data->vbus_divider)
		reg |= 1 << USBSS_PHY_VBUS_SEL_SHIFT;

	dwc3_ti_writel(data, USBSS_PHY_CONFIG, reg);

	// Register as an extcon device
	ret = dwc3_ti_register_extcon(data);
	if (ret)
		goto err_clk_disable;

	// Initialize a work queue
	INIT_DELAYED_WORK(&data->work, role_detect_work);

	// Get struct gpio and irq for ID pin
	ret = dwc3_ti_get_id_gpio(data);
	if (ret) {
		dev_err(dev, "error reading id-gpio property");
		goto err_clk_disable;
	}

	// Read a set VBUS IRQs
	ret = dwc3_ti_vbus_irq_setup(pdev);
	if (ret)
		goto err_clk_disable;

	ret = of_platform_populate(node, NULL, NULL, dev);
	if (ret) {
		dev_err(dev, "failed to create dwc3 core: %d\n", ret);
		goto err_clk_disable;
	}

	/* Enable vbus irq */
	dwc3_ti_enable_irqs(data);

	role_detect_work(&data->work.work);
	return 0;

err_clk_disable:
	clk_put(data->usb2_refclk);
err:
	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);
        pm_runtime_set_suspended(dev);

	return ret;

}

static int dwc3_ti_remove_core(struct device *dev, void *c)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dwc3_data *data = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&data->work);
	platform_device_unregister(pdev);

	return 0;
}

static int dwc3_ti_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dwc3_data *data = platform_get_drvdata(pdev);
	u32 reg;

	dwc3_ti_disable_irqs(data);

	device_for_each_child(dev, NULL, dwc3_ti_remove_core);

	/* Clear mode valid bit */
	reg = dwc3_ti_readl(data, USBSS_MODE_CONTROL);
	reg &= ~USBSS_MODE_VALID;
	dwc3_ti_writel(data, USBSS_MODE_CONTROL, reg);

	clk_put(data->usb2_refclk);

	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);


	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id dwc3_ti_of_match[] = {
	{ .compatible = "ti,am62-usb"},
	{},
};
MODULE_DEVICE_TABLE(of, dwc3_ti_of_match);

#ifdef CONFIG_PM_SLEEP
static int dwc3_ti_suspend_common(struct device *dev)
{
	struct dwc3_data *data = dev_get_drvdata(dev);
	int ret;
	u32 reg;

	if (device_may_wakeup(dev)) {
		if (data->id_gpio) {
			ret = enable_irq_wake(data->id_gpio->id_irq);
			if (ret)
				return ret;
		}

		enable_irq_wake(data->vbus_irq);
	}

	/* clear mode valid bit */
	reg = dwc3_ti_readl(data, USBSS_MODE_CONTROL);
	reg &= ~USBSS_MODE_VALID;
	dwc3_ti_writel(data, USBSS_MODE_CONTROL, reg);

	/* disable irqs */
	dwc3_ti_disable_irqs(data);

	if (!device_may_wakeup(dev))
		pinctrl_pm_select_sleep_state(dev);

	clk_disable_unprepare(data->usb2_refclk);

	/* Set wakeup config enable bits */
	reg = dwc3_ti_readl(data, USBSS_WAKEUP_CONFIG);
	reg |= USBSS_WAKEUP_CFG_OVERCURRENT_EN | USBSS_WAKEUP_CFG_LINESTATE_EN |
	       USBSS_WAKEUP_CFG_SESSVALID_EN | USBSS_WAKEUP_CFG_VBUSVALID_EN;
	dwc3_ti_writel(data, USBSS_WAKEUP_CONFIG, reg);

	return 0;

}

static int dwc3_ti_resume_common(struct device *dev)
{
	struct dwc3_data *data = dev_get_drvdata(dev);
	int ret;
	u32 reg;

	/* Clear wakeup config enable bits */
	reg = dwc3_ti_readl(data, USBSS_WAKEUP_CONFIG);
	reg &= ~(USBSS_WAKEUP_CFG_OVERCURRENT_EN | USBSS_WAKEUP_CFG_LINESTATE_EN |
		 USBSS_WAKEUP_CFG_SESSVALID_EN | USBSS_WAKEUP_CFG_VBUSVALID_EN);
	dwc3_ti_writel(data, USBSS_WAKEUP_CONFIG, reg);

	clk_prepare_enable(data->usb2_refclk);

	if (!device_may_wakeup(dev))
		pinctrl_pm_select_default_state(dev);

	if (device_may_wakeup(dev)) {
		if (data->id_gpio) {
			ret = disable_irq_wake(data->id_gpio->id_irq);
			if (ret)
				return ret;
		}

		disable_irq_wake(data->vbus_irq);
	}

	/* Restore PLL rate code */
	ret = regmap_update_bits(data->syscon, data->offset, PHY_PLL_REFCLK_MASK, data->rate_code);
	if (ret) {
		dev_err(dev, "failed to set phy pll reference clock rate while resuming\n");
		return ret;
	}

	/* Based on the status of the connect field set or clear mode valid */
	if (data->connect) {
		reg = dwc3_ti_readl(data, USBSS_MODE_CONTROL);
		reg |= USBSS_MODE_VALID;
		dwc3_ti_writel(data, USBSS_MODE_CONTROL, reg);
	}

	/* Enable irqs */
	dwc3_ti_enable_irqs(data);

	/* Read wakeup status bits */
	reg = dwc3_ti_readl(data, USBSS_WAKEUP_STAT);

	/* Clear the wakeup status with wakeup clear bits */
	reg |= USBSS_WAKEUP_STAT_CLR;
	dwc3_ti_writel(data, USBSS_WAKEUP_STAT, reg);

	/* If there was a VBUSVALID event possiblity of connect or
	 * disconnect event. So, call role detect work once
	 */
	if (reg & USBSS_WAKEUP_STAT_VBUSVALID)
		role_detect_work(&data->work.work);

	return 0;
}

/* *
 * This is limited to the wrapper device which is always idle
 * except for waiting for interrupts which can be done even
 * sleep with enable_irq_wake
 *
 * Therefore always be ready to suspend
 */

static UNIVERSAL_DEV_PM_OPS(dwc3_ti_pm_ops, dwc3_ti_suspend_common,
			    dwc3_ti_resume_common, NULL);
#endif

static struct platform_driver dwc3_ti_driver = {
	.probe		= dwc3_ti_probe,
	.remove		= dwc3_ti_remove,
	.driver		= {
		.name 	= "dwc3-ti",
		.pm	= &dwc3_ti_pm_ops,
		.of_match_table = dwc3_ti_of_match,
	},
};

module_platform_driver(dwc3_ti_driver);

MODULE_ALIAS("platform:dwc3-ti");
MODULE_AUTHOR("Aswath Govindraju <a-govindraju@ti.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 TI Glue Layer");
