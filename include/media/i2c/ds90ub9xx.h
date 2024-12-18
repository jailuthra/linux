/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __MEDIA_I2C_DS90UB9XX_H__
#define __MEDIA_I2C_DS90UB9XX_H__

#include <linux/types.h>

struct i2c_atr;

/**
 * struct ds90ub9xx_platform_data - platform data for FPD-Link Serializers.
 * @port: Deserializer RX port for this Serializer
 * @atr: I2C ATR
 * @bc_rate: back-channel clock rate
 * @deser_priv: Private data for Deserializer driver instance
 * @read_sensor_sts: Read sensor status registers on deser
 */
struct ds90ub9xx_platform_data {
	u32 port;
	struct i2c_atr *atr;
	unsigned long bc_rate;
	struct ub960_data *deser_priv;
	int (*read_sensor_sts)(struct ub960_data *priv, u8 nport, u8 sts_reg,
			       u8 *val);
};

#endif /* __MEDIA_I2C_DS90UB9XX_H__ */
