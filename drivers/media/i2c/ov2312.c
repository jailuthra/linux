// SPDX-License-Identifier: GPL-2.0
/*
 * Omnivision OV2312 RGB-IR Image Sensor driver
 *
 * Copyright (c) 2022 Jai Luthra <j-luthra@ti.com>
 */

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/v4l2-mediabus.h>
#include <linux/videodev2.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>

#include "ov2312.h"

struct ov2312 {
	struct device *dev;

	struct clk *clk;
	unsigned long clk_rate;

	struct i2c_client *client;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *again;

	u32 fps;

	struct mutex lock;
	bool streaming;
};

static inline struct ov2312 *to_ov2312(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov2312, sd);
}

static int ov2312_read(struct ov2312 *ov2312, u16 addr, u32 *val, size_t nbytes)
{
	int ret;
	__le32 val_le = 0;

	ret = regmap_bulk_read(ov2312->regmap, addr, &val_le, nbytes);
	if (ret < 0) {
		dev_err(ov2312->dev, "%s: failed to read reg 0x%04x: %d\n",
			__func__, addr, ret);
		return ret;
	} else {
		/*dev_info(ov2312->dev, "%s: read reg 0x%04x => 0x%04x\n",
			 __func__, addr, val_le);*/
	}

	*val = le32_to_cpu(val_le);
	return 0;
}

static int ov2312_write(struct ov2312 *ov2312, u16 addr, u32 val, size_t nbytes)
{
	int ret;
	__le32 val_le = cpu_to_le32(val);

	ret = regmap_bulk_write(ov2312->regmap, addr, &val_le, nbytes);
	if (ret < 0)
		dev_err(ov2312->dev, "%s: failed to write reg 0x%04x: %d\n",
			__func__, addr, ret);
	return ret;
}

static int ov2312_write_table(struct ov2312 *ov2312,
			      const struct reg_sequence *regs,
			      unsigned int nr_regs)
{
	int ret, i;
	unsigned int val = 0;

	/*ret = regmap_multi_reg_write(ov2312->regmap, regs, nr_regs);
	if (ret < 0)
		dev_err(ov2312->dev,
			"%s: failed to write reg table (%d)!\n", __func__, ret);*/

	for (i = 0; i < nr_regs; i++) {
		/*ret = ov2312_read(ov2312, regs[i].reg, &val, 1);
		if (ret < 0) {
			dev_err(ov2312->dev,
				"%s: failed to read reg[%d] 0x%04x (%d)!\n",
				__func__, i, regs[i].reg, ret);
		} else {
			dev_info(ov2312->dev, "%s: read reg[%d] 0x%04x => 0x%04x\n",
				 __func__, i, regs[i].reg, val);
		}*/
		ret = regmap_write(ov2312->regmap, regs[i].reg, regs[i].def);
		if (ret < 0) {
			dev_err(ov2312->dev,
				"%s: failed to write reg[%d] 0x%04x = 0x%02x (%d)!\n",
				__func__, i, regs[i].reg, regs[i].def, ret);
			return ret;
		}
	}
	return 0;
}

static void ov2312_init_formats(struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *format;

	format = v4l2_state_get_stream_format(state, 0, 0);
	format->code = ov2312_mbus_formats[0];
	format->width = ov2312_framesizes[0].width;
	format->height = ov2312_framesizes[0].height;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_DEFAULT;
}


static int ov2312_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct ov2312 *ov2312 = to_ov2312(sd);
	struct v4l2_mbus_framefmt *format;
	const struct v4l2_area *fsize;
	u32 code;
	int ret = 0;

	if (fmt->pad != 0)
		return -EINVAL;

	if (fmt->stream != 0)
		return -EINVAL;

	/* Sensor only supports a single format. */
	code = ov2312_mbus_formats[0];

	/* Find the nearest supported frame size. */
	fsize = v4l2_find_nearest_size(ov2312_framesizes,
				       ARRAY_SIZE(ov2312_framesizes), width,
				       height, fmt->format.width,
				       fmt->format.height);

	v4l2_subdev_lock_state(state);

	/* Update the stored format and return it. */
	format = v4l2_state_get_stream_format(state, fmt->pad, fmt->stream);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE && ov2312->streaming) {
		ret = -EBUSY;
		goto done;
	}

	format->code = code;
	format->width = fsize->width;
	format->height = fsize->height;

	fmt->format = *format;

done:
	v4l2_subdev_unlock_state(state);

	return ret;
}

static int _ov2312_set_routing(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_route routes[] = {
		{
			.source_pad = 0,
			.source_stream = 0,
			.flags = V4L2_SUBDEV_ROUTE_FL_IMMUTABLE |
				 V4L2_SUBDEV_ROUTE_FL_SOURCE |
				 V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		},
		{
			.source_pad = 0,
			.source_stream = 1,
			.flags = V4L2_SUBDEV_ROUTE_FL_IMMUTABLE |
				 V4L2_SUBDEV_ROUTE_FL_SOURCE,
		}
	};

	struct v4l2_subdev_krouting routing = {
		.num_routes = ARRAY_SIZE(routes),
		.routes = routes,
	};

	int ret;

	ret = v4l2_subdev_set_routing(sd, state, &routing);
	if (ret < 0)
		return ret;

	ov2312_init_formats(state);

	return 0;
}

static int ov2312_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				 struct v4l2_mbus_frame_desc *fd)
{
	struct v4l2_subdev_state *state;
	struct v4l2_mbus_framefmt *fmt;
	u32 bpp;
	int ret = 0;

	if (pad != 0)
		return -EINVAL;

	state = v4l2_subdev_lock_active_state(sd);

	fmt = v4l2_state_get_stream_format(state, 0, 0);
	if (!fmt) {
		ret = -EPIPE;
		goto out;
	}

	memset(fd, 0, sizeof(*fd));

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;

	/* pixel stream */

	bpp = 10;

	fd->entry[fd->num_entries].stream = 0;

	fd->entry[fd->num_entries].flags = V4L2_MBUS_FRAME_DESC_FL_LEN_MAX;
	fd->entry[fd->num_entries].length = fmt->width * fmt->height * bpp / 8;
	fd->entry[fd->num_entries].pixelcode = fmt->code;
	fd->entry[fd->num_entries].bus.csi2.vc = 0;
	fd->entry[fd->num_entries].bus.csi2.dt = 0x2b; /* SRGGB10 */

	fd->num_entries++;

out:
	v4l2_subdev_unlock_state(state);

	return ret;
}

static int ov2312_set_routing(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      enum v4l2_subdev_format_whence which,
			      struct v4l2_subdev_krouting *routing)
{
	int ret;

	if (routing->num_routes == 0 || routing->num_routes > 1)
		return -EINVAL;

	v4l2_subdev_lock_state(state);

	ret = _ov2312_set_routing(sd, state);

	v4l2_subdev_unlock_state(state);

	return ret;
}


static int ov2312_init_cfg(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state)
{
	int ret;

	v4l2_subdev_lock_state(state);

	ret = _ov2312_set_routing(sd, state);

	v4l2_subdev_unlock_state(state);

	return ret;
}

static int ov2312_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(ov2312_mbus_formats))
		return -EINVAL;

	code->code = ov2312_mbus_formats[code->index];

	return 0;
}

static int ov2312_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ov2312_mbus_formats); ++i) {
		if (ov2312_mbus_formats[i] == fse->code)
			break;
	}

	if (i == ARRAY_SIZE(ov2312_mbus_formats))
		return -EINVAL;

	if (fse->index >= ARRAY_SIZE(ov2312_framesizes))
		return -EINVAL;

	fse->min_width = ov2312_framesizes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->max_height = ov2312_framesizes[fse->index].height;
	fse->min_height = fse->max_height;

	return 0;
}

static int ov2312_get_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct ov2312 *ov2312 = to_ov2312(sd);

	fi->interval.numerator = 1;
	fi->interval.denominator = ov2312->fps;

	dev_info(ov2312->dev, "%s: Framerate is %dfps\n", __func__, ov2312->fps);

	return 0;
}

static int ov2312_set_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct ov2312 *ov2312 = to_ov2312(sd);

	dev_info(ov2312->dev, "%s: Set framereate %dfps\n", __func__,
		 fi->interval.denominator/fi->interval.numerator);
	if (fi->interval.denominator/fi->interval.numerator != ov2312->fps) {
		return -EINVAL;
	}
	return 0;
}

static int ov2312_detect(struct ov2312 *ov2312)
{
	int ret;
	u32 id;

	ret = ov2312_read(ov2312, OV2312_SC_CHIP_ID_HIGH, &id, 2);
	if (ret < 0)
		return ret;

	id = cpu_to_be16(id);

	if (id != OV2312_CHIP_ID) {
		dev_err(ov2312->dev,
			"%s: unknown chip ID 0x%04x\n", __func__, id);
		return -ENODEV;
	}

	dev_info(ov2312->dev, "%s: detected chip ID 0x%04x\n", __func__, id);
	/*ret = ov2312_read(ov2312, 0x0103, &id, 1);
	ret = ov2312_read(ov2312, 0x0109, &id, 1);*/
	return 0;
}

static int ov2312_power_on(struct ov2312 *ov2312)
{
	int ret;
	ret = clk_prepare_enable(ov2312->clk);
	if (ret < 0)
		return ret;

	if (ov2312->reset_gpio) {
		gpiod_set_value_cansleep(ov2312->reset_gpio, 0);
		msleep(10);
		gpiod_set_value_cansleep(ov2312->reset_gpio, 1);
		msleep(30);
	}
	return 0;
}

static int ov2312_power_off(struct ov2312 *ov2312)
{
	if (ov2312->reset_gpio) {
		gpiod_set_value_cansleep(ov2312->reset_gpio, 0);
		msleep(10);
	}

	clk_disable_unprepare(ov2312->clk);

	return 0;
}

static int ov2312_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov2312 *ov2312 = to_ov2312(sd);

	return ov2312_power_on(ov2312);
}

static int ov2312_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov2312 *ov2312 = to_ov2312(sd);

	return ov2312_power_off(ov2312);
}

static int ov2312_start_stream(struct ov2312 *ov2312)
{
	int ret;
	ret = ov2312_write_table(ov2312, ov2312_1600x1300_30fps,
				 ARRAY_SIZE(ov2312_1600x1300_30fps));
	if (ret < 0)
		return ret;

	msleep(100);

	/* Set active */
	ret = ov2312_write(ov2312, OV2312_SYS_MODE_SEL, 1, 1);
	if (ret < 0)
		return ret;

	/* No communication is possible for a while after exiting standby */
	msleep(20);
	return 0;
}

static int ov2312_stop_stream(struct ov2312 *ov2312)
{
	int ret;

	/* Set standby */
	ret = ov2312_write(ov2312, OV2312_SYS_MODE_SEL, 0, 1);
	if (ret < 0)
		return ret;

	/* No communication is possible for a while after entering standby */
	usleep_range(10000, 20000);
	return 0;
}

static int ov2312_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov2312 *ov2312 = to_ov2312(sd);
	int ret;

	/*mutex_lock(&ov2312->lock);*/
	if (ov2312->streaming == enable) {
		/*mutex_unlock(&ov2312->lock);*/
		return 0;
	}

	if (enable) {
		/*ret = pm_runtime_get_sync(ov2312->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(ov2312->dev);
			goto err_unlock;
		}*/

		ret = ov2312_start_stream(ov2312);
		if (ret < 0)
			goto err_runtime_put;
	} else {
		ret = ov2312_stop_stream(ov2312);
		if (ret < 0)
			goto err_runtime_put;
		/*pm_runtime_mark_last_busy(ov2312->dev);*/
		/*pm_runtime_put_autosuspend(ov2312->dev);*/
	}

	ov2312->streaming = enable;
	/* WDR, HFLIP, VFLIP, TEST PATTERN cannot change during streaming */
	/*__v4l2_ctrl_grab(ov2312->ctrl.wdr, enable);
	__v4l2_ctrl_grab(ov2312->ctrl.h_flip, enable);
	__v4l2_ctrl_grab(ov2312->ctrl.v_flip, enable);
	__v4l2_ctrl_grab(ov2312->ctrl.pg_mode, enable);*/

	/*mutex_unlock(&ov2312->lock);*/
	return 0;

err_runtime_put:
	/*pm_runtime_put(ov2312->dev);*/

err_unlock:
	/*mutex_unlock(&ov2312->lock);*/
	dev_err(ov2312->dev,
		"%s: failed to setup streaming %d\n", __func__, ret);
	return ret;
}

static struct v4l2_subdev_video_ops ov2312_subdev_video_ops = {
	.s_stream = ov2312_set_stream,
	.g_frame_interval = ov2312_get_frame_interval,
	.s_frame_interval = ov2312_set_frame_interval,
};

/*static struct v4l2_subdev_core_ops ov2312_subdev_core_ops = {
	.s_power	= ,
};*/

static struct v4l2_subdev_pad_ops ov2312_subdev_pad_ops = {
	.init_cfg = ov2312_init_cfg,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = ov2312_set_fmt,
	.enum_mbus_code = ov2312_enum_mbus_code,
	.enum_frame_size = ov2312_enum_frame_sizes,
	.set_routing = ov2312_set_routing,
	.get_frame_desc	= ov2312_get_frame_desc,
};

static struct v4l2_subdev_ops ov2312_subdev_ops = {
	//.core	= &ov2312_subdev_core_ops,
	.video	= &ov2312_subdev_video_ops,
	.pad	= &ov2312_subdev_pad_ops,
};

static const struct dev_pm_ops ov2312_pm_ops = {
	SET_RUNTIME_PM_OPS(ov2312_suspend, ov2312_resume, NULL)
};

static int ov2312_probe(struct i2c_client *client)
{
	struct ov2312 *ov2312;
	struct v4l2_subdev *sd;
	struct v4l2_ctrl_handler *ctrl_hdr;
	int ret;

	/* Allocate internal struct */
	ov2312 = devm_kzalloc(&client->dev, sizeof(*ov2312), GFP_KERNEL);
	if (!ov2312)
		return -ENOMEM;

	ov2312->dev = &client->dev;
	ov2312->client = client;

	/* Initialize I2C Regmap */
	ov2312->regmap = devm_regmap_init_i2c(client, &ov2312_regmap_config);
	if (IS_ERR(ov2312->regmap))
		return PTR_ERR(ov2312->regmap);

	/* Initialize Shutdown GPIO */
	ov2312->reset_gpio = devm_gpiod_get_optional(ov2312->dev,
							 "reset",
							 GPIOD_OUT_HIGH);
	if (IS_ERR(ov2312->reset_gpio))
		return PTR_ERR(ov2312->reset_gpio);

	ov2312->clk = devm_clk_get(ov2312->dev, "xvclk");
	if (IS_ERR(ov2312->clk))
		return PTR_ERR(ov2312->clk);

	ov2312->clk_rate = clk_get_rate(ov2312->clk);
	dev_info(ov2312->dev, "xvclk rate: %lu Hz\n", ov2312->clk_rate);

	if (ov2312->clk_rate < 6000000 || ov2312->clk_rate > 27000000)
		return -EINVAL;

	/* Power on */
	ret = ov2312_power_on(ov2312);
	if (ret < 0)
		return ret;

	/* Detect sensor */
	ret = ov2312_detect(ov2312);
	if (ret < 0)
		return ret;

	/* Initialize the subdev and its controls. */
	sd = &ov2312->sd;
	v4l2_i2c_subdev_init(sd, client, &ov2312_subdev_ops);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;

	/* Initialize the media entity. */
	ov2312->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &ov2312->pad);
	if (ret < 0) {
		dev_err(ov2312->dev,
			"%s: media entity init failed %d\n", __func__, ret);
		return ret;
	}

	ov2312->fps = OV2312_FRAMERATE_DEFAULT;

	/*[> Initialize controls <]
	ctrl_hdr = &ov2312->ctrls;
	ret = v4l2_ctrl_handler_init(ctrl_hdr, 2);
	if (ret < 0) {
		dev_err(ov2312->dev,
			"%s: ctrl handler init failed: %d\n", __func__, ret);
		goto err_media_cleanup;
	}

	mutex_init(&ov2312->lock);
	ov2312->ctrls.lock = &ov2312->lock;

	[> Add new controls <]
	ov2312->exposure = v4l2_ctrl_new_std(ctrl_hdr, &ov2312_ctrl_ops,
					     V4L2_CID_EXPOSURE, 0,
					     IMX390_EXPOSURE_MAX(ov2312->fps),
					     1, IMX390_EXPOSURE_DEFAULT);

	ov2312->again = v4l2_ctrl_new_std(ctrl_hdr, &ov2312_ctrl_ops,
					  V4L2_CID_ANALOGUE_GAIN, 0,
					  IMX390_ANALOG_GAIN_MAX, 1,
					  IMX390_ANALOG_GAIN_DEFAULT);

	ov2312->sd.ctrl_handler = ctrl_hdr;
	if (ov2312->ctrls.error) {
		ret = ov2312->ctrls.error;
		dev_err(ov2312->dev,
			"%s: failed to add the ctrls: %d\n", __func__, ret);
		goto err_ctrl_free;
	}*/

	/* PM Runtime */
	/*pm_runtime_set_active(ov2312->dev);*/
	/*pm_runtime_enable(ov2312->dev);*/

	ret = v4l2_subdev_init_finalize(sd);
	if (ret < 0)
		goto err_pm_disable;

	/* Finally, register the subdev. */
	ret = v4l2_async_register_subdev(sd);
	if (ret < 0) {
		dev_err(ov2312->dev,
			"%s: v4l2 subdev register failed %d\n", __func__, ret);
		goto err_subdev_cleanup;
	}

	dev_info(ov2312->dev, "ov2312 probed\n");
	return 0;

err_subdev_cleanup:
	v4l2_subdev_cleanup(&ov2312->sd);

err_pm_disable:
	/*pm_runtime_disable(ov2312->dev);
	pm_runtime_set_suspended(ov2312->dev);*/

err_ctrl_free:
	//v4l2_ctrl_handler_free(ctrl_hdr);
	//mutex_destroy(&ov2312->lock);

err_media_cleanup:
	media_entity_cleanup(&ov2312->sd.entity);

	return ret;
}

static int ov2312_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov2312 *ov2312 = to_ov2312(sd);

	v4l2_async_unregister_subdev(sd);
	//v4l2_ctrl_handler_free(&ov2312->ctrl.handler);
	v4l2_subdev_cleanup(&ov2312->sd);
	media_entity_cleanup(&sd->entity);
	//mutex_destroy(&ov2312->lock);

	/*pm_runtime_disable(ov2312->dev);
	pm_runtime_set_suspended(ov2312->dev);*/

	return 0;
}

static const struct i2c_device_id ov2312_id[] = {
	{ "ov2312", 0 },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, ov2312_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ov2312_of_match[] = {
	{ .compatible = "ovti,ov2312", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ov2312_of_match);
#endif

static struct i2c_driver ov2312_i2c_driver = {
	.driver = {
		.name	= "ov2312",
		.pm	= &ov2312_pm_ops,
		.of_match_table = of_match_ptr(ov2312_of_match),
	},
	.probe_new	= ov2312_probe,
	.remove		= ov2312_remove,
	.id_table	= ov2312_id,
};

module_i2c_driver(ov2312_i2c_driver);

MODULE_AUTHOR("Jai Luthra <j-luthra@ti.com>");
MODULE_DESCRIPTION("OV2312 RGB-IR Image Sensor driver");
MODULE_LICENSE("GPL v2");
