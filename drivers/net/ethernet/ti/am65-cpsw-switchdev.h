/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2020 Texas Instruments Incorporated - https://www.ti.com/
 */

#ifndef DRIVERS_NET_ETHERNET_TI_AM65_CPSW_SWITCHDEV_H_
#define DRIVERS_NET_ETHERNET_TI_AM65_CPSW_SWITCHDEV_H_

bool am65_cpsw_port_dev_check(const struct net_device *dev);
int am65_cpsw_switchdev_register_notifiers(struct am65_cpsw_common *cpsw);
void am65_cpsw_switchdev_unregister_notifiers(struct am65_cpsw_common *cpsw);

#endif /* DRIVERS_NET_ETHERNET_TI_AM65_CPSW_SWITCHDEV_H_ */
