// SPDX-License-Identifier: GPL-2.0
/*
 * pci-j721e - PCIe controller driver for TI's J721E SoCs
 *
 * Copyright (C) 2018-2019 Texas Instruments Incorporated - http://www.ti.com
 *
 * Author: Kishon Vijay Abraham I <kishon@ti.com>
 */

#include <dt-bindings/pci/pci.h>
#include <linux/io.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>

#define EOI_REG			0x10

#define ENABLE_REG_SYS_0	0x100
#define STATUS_REG_SYS_0	0x500
#define INTx_EN(num)		(1 << (num))

#define J721E_TRANS_CTRL(a)		((a) * 0xc)
#define J721E_TRANS_REQ_ID(a)		(((a) * 0xc) + 0x4)
#define J721E_TRANS_VIRT_ID(a)		(((a) * 0xc) + 0x8)

#define J721E_REQID_MASK			0xffff
#define J721E_REQID_SHIFT		16

#define J721E_EN				BIT(0)
#define J721E_ATYPE_SHIFT		16

enum j721e_atype {
	PHYS_ADDR,
	INT_ADDR,
	VIRT_ADDR,
	TRANS_ADDR,
};

struct j721e_pcie {
	struct device		*dev;
	struct device_node	*node;
	void __iomem		*intd_cfg_base;
	void __iomem		*user_cfg_base;
	void __iomem		*vmap_lp_base;
	u8			vmap_lp_index;
	struct irq_domain	*legacy_irq_domain;
	bool			enable_smmu;
};

static inline u32 j721e_pcie_vmap_readl(struct j721e_pcie *pcie, u32 offset)
{
	return readl(pcie->vmap_lp_base + offset);
}

static inline void j721e_pcie_vmap_writel(struct j721e_pcie *pcie, u32 offset,
					  u32 value)
{
	writel(value, pcie->vmap_lp_base + offset);
}

static inline u32 j721e_pcie_intd_readl(struct j721e_pcie *pcie, u32 offset)
{
	return readl(pcie->intd_cfg_base + offset);
}

static inline void j721e_pcie_intd_writel(struct j721e_pcie *pcie, u32 offset,
					  u32 value)
{
	writel(value, pcie->intd_cfg_base + offset);
}

static inline u32 j721e_pcie_user_readl(struct j721e_pcie *pcie, u32 offset)
{
	return readl(pcie->user_cfg_base + offset);
}

static inline void j721e_pcie_user_writel(struct j721e_pcie *pcie, u32 offset,
					  u32 value)
{
	writel(value, pcie->user_cfg_base + offset);
}

static void j721e_pcie_quirk(struct pci_dev *pci_dev)
{
	struct pci_bus *root_bus;
	struct pci_dev *bridge;
	struct j721e_pcie *pcie;
	struct pci_bus *bus;
	struct device *dev;
	int index;
	u32 val;

	static const struct pci_device_id rc_pci_devids[] = {
		/* Should be replaced by TI Specific VendorID, DeviceID */
		{ PCI_DEVICE(0x17CD, 0x200),
		.class = PCI_CLASS_BRIDGE_PCI << 8, .class_mask = ~0, },
		{ 0, },
	};

	dev = pci_get_host_bridge_device(pci_dev);
	pcie = dev_get_drvdata(dev->parent->parent);
	bus = pci_dev->bus;
	index = pcie->vmap_lp_index;

	if (!pcie->enable_smmu)
		return;

	if (index >= 32)
		return;

	if (pci_is_root_bus(bus))
		return;

	root_bus = bus;
	while (!pci_is_root_bus(root_bus)) {
		bridge = root_bus->self;
		root_bus = root_bus->parent;
	}

	if (pci_match_id(rc_pci_devids, bridge)) {
		val = J721E_REQID_MASK << J721E_REQID_SHIFT |
			(bus->number << 8 | pci_dev->devfn);
		j721e_pcie_vmap_writel(pcie, J721E_TRANS_REQ_ID(index), val);
		val = VIRT_ADDR << J721E_ATYPE_SHIFT;
		j721e_pcie_vmap_writel(pcie, J721E_TRANS_VIRT_ID(index), val);
		j721e_pcie_vmap_writel(pcie, J721E_TRANS_CTRL(index), J721E_EN);
	}

	pcie->vmap_lp_index++;
}
DECLARE_PCI_FIXUP_ENABLE(PCI_ANY_ID, PCI_ANY_ID, j721e_pcie_quirk);

static void j721e_pcie_legacy_irq_handler(struct irq_desc *desc)
{
	int i;
	u32 reg;
	int virq;
	struct j721e_pcie *pcie = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);

	chained_irq_enter(chip, desc);

	for (i = 0; i < PCI_NUM_INTX; i++) {
		reg = j721e_pcie_intd_readl(pcie, STATUS_REG_SYS_0);
		if (!(reg & INTx_EN(i)))
			continue;

		virq = irq_linear_revmap(pcie->legacy_irq_domain, i);
		generic_handle_irq(virq);
		j721e_pcie_intd_writel(pcie, STATUS_REG_SYS_0, INTx_EN(i));
		j721e_pcie_intd_writel(pcie, EOI_REG, i);
	}

	chained_irq_exit(chip, desc);
}

static int j721e_pcie_intx_map(struct irq_domain *domain, unsigned int irq,
			       irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &dummy_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops j721e_pcie_intx_domain_ops = {
	.map = j721e_pcie_intx_map,
};

static int j721e_pcie_config_legacy_irq(struct j721e_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct irq_domain *legacy_irq_domain;
	struct device_node *node = pcie->node;
	struct device_node *intc_node;
	int irq;
	u32 reg;
	int i;

	intc_node = of_get_child_by_name(node, "legacy-interrupt-controller");
	if (!intc_node) {
		dev_WARN(dev, "legacy-interrupt-controller node is absent\n");
		return -EINVAL;
	}

	irq = irq_of_parse_and_map(intc_node, 0);
	if (!irq) {
		dev_err(dev, "Failed to parse and map legacy irq\n");
		return -EINVAL;
	}
	irq_set_chained_handler_and_data(irq, j721e_pcie_legacy_irq_handler,
					 pcie);

	legacy_irq_domain = irq_domain_add_linear(intc_node,  PCI_NUM_INTX,
						  &j721e_pcie_intx_domain_ops,
						  NULL);
	if (!legacy_irq_domain) {
		dev_err(dev, "Failed to add irq domain for legacy irqs\n");
		return -EINVAL;
	}
	pcie->legacy_irq_domain = legacy_irq_domain;

	for (i = 0; i < PCI_NUM_INTX; i++) {
		reg = j721e_pcie_intd_readl(pcie, ENABLE_REG_SYS_0);
		reg |= INTx_EN(i);
		j721e_pcie_intd_writel(pcie, ENABLE_REG_SYS_0, reg);
	}

	return 0;
}

static const struct of_device_id of_j721e_pcie_match[] = {
	{
		.compatible = "ti,k3-j721e-pcie",
	},
	{},
};

static int j721e_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct platform_device *platform_dev;
	struct device_node *child_node;
	struct j721e_pcie *pcie;
	struct resource *res;
	void __iomem *base;
	u32 mode;
	int ret;

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	pcie->dev = dev;
	pcie->node = node;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "intd_cfg");
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);
	pcie->intd_cfg_base = base;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "user_cfg");
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);
	pcie->user_cfg_base = base;

	ret = of_property_read_u32(node, "pci-mode", &mode);
	if (ret < 0) {
		dev_err(dev, "Failed to get pci-mode binding\n");
		return ret;
	}

	dev_set_drvdata(dev, pcie);
	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "pm_runtime_get_sync failed\n");
		goto err_get_sync;
	}

	switch (mode) {
	case PCI_MODE_RC:
		if (!IS_ENABLED(CONFIG_PCIE_CADENCE_HOST)) {
			ret = -ENODEV;
			goto err_get_sync;
		}

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   "vmap");
		base = devm_ioremap_resource(dev, res);
		if (IS_ERR(base))
			goto err_get_sync;
		pcie->vmap_lp_base = base;

		ret = j721e_pcie_config_legacy_irq(pcie);
		if (ret < 0)
			goto err_get_sync;

		child_node = of_get_child_by_name(node, "pcie");
		if (!child_node) {
			dev_WARN(dev, "pcie-rc node is absent\n");
			goto err_get_sync;
		}

		if (of_property_read_bool(child_node, "iommu-map"))
			pcie->enable_smmu = true;

		platform_dev = of_platform_device_create(child_node, NULL, dev);
		if (!platform_dev) {
			ret = -ENODEV;
			dev_err(dev, "Failed to create Cadence RC device\n");
			goto err_get_sync;
		}

		break;
	case PCI_MODE_EP:
		if (!IS_ENABLED(CONFIG_PCIE_CADENCE_EP)) {
			ret = -ENODEV;
			goto err_get_sync;
		}

		child_node = of_get_child_by_name(node, "pcie-ep");
		if (!child_node) {
			dev_WARN(dev, "pcie-ep node is absent\n");
			goto err_get_sync;
		}

		platform_dev = of_platform_device_create(child_node, NULL, dev);
		if (!platform_dev) {
			ret = -ENODEV;
			dev_err(dev, "Failed to create Cadence EP device\n");
			goto err_get_sync;
		}

		break;
	default:
		dev_err(dev, "INVALID device type %d\n", mode);
	}

	return 0;

err_get_sync:
	pm_runtime_put(dev);
	pm_runtime_disable(dev);

	return ret;
}

static int j721e_pcie_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	pm_runtime_put(dev);
	pm_runtime_disable(dev);
	of_platform_depopulate(dev);

	return 0;
}

static struct platform_driver j721e_pcie_driver = {
	.probe  = j721e_pcie_probe,
	.remove = j721e_pcie_remove,
	.driver = {
		.name	= "j721e-pcie",
		.of_match_table = of_j721e_pcie_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(j721e_pcie_driver);
