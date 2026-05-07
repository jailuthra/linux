// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony imx678 cameras.
 *
 * Based on Sony imx477 camera driver
 * Copyright (C) 2019-2020 Raspberry Pi (Trading) Ltd
 * Modified by Will WHANG
 * Modified by sohonomura2020 in Soho Enterprise Ltd.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

/*
 * Initialisation delay between XCLR low->high and the moment when the sensor
 * can start capture (i.e. can leave software standby)
 */
#define IMX678_XCLR_MIN_DELAY_US    500000
#define IMX678_XCLR_DELAY_RANGE_US  1000

/* Standby or streaming mode */
#define IMX678_REG_MODE_SELECT          CCI_REG8(0x3000)
#define IMX678_MODE_STANDBY             0x01
#define IMX678_MODE_STREAMING           0x00
#define IMX678_STREAM_DELAY_US          25000
#define IMX678_STREAM_DELAY_RANGE_US    1000

/* Group hold */
#define IMX678_REG_HOLD                 CCI_REG8(0x3001)

/* XVS/XHS sync control */
#define IMX678_REG_XMSTA                CCI_REG8(0x3002)
#define IMX678_REG_XXS_DRV              CCI_REG8(0x30A6)
#define IMX678_REG_XXS_OUTSEL           CCI_REG8(0x30A4)

/* Clk selection */
#define IMX678_REG_INCK_SEL             CCI_REG8(0x3014)

/* Link Speed */
#define IMX678_REG_DATARATE_SEL         CCI_REG8(0x3015)

/* Lane Count */
#define IMX678_REG_LANEMODE             CCI_REG8(0x3040)

/* VMAX internal VBLANK*/
#define IMX678_REG_VMAX                 CCI_REG24_LE(0x3028)
#define IMX678_VMAX_MAX                 0xfffff
#define IMX678_VMAX_DEFAULT             2250

/* HMAX internal HBLANK*/
#define IMX678_REG_HMAX                 CCI_REG16_LE(0x302C)
#define IMX678_HMAX_MAX                 0xffff

/* SHR internal */
#define IMX678_REG_SHR                  CCI_REG24_LE(0x3050)
#define IMX678_SHR_MIN                  8
#define IMX678_SHR_MIN_CLEARHDR         10
#define IMX678_SHR_MAX                  0xfffff

/* Exposure control */
#define IMX678_EXPOSURE_MIN             2
#define IMX678_EXPOSURE_STEP            1
#define IMX678_EXPOSURE_DEFAULT         1000
#define IMX678_EXPOSURE_MAX             49865

/* Black level control */
#define IMX678_REG_BLKLEVEL             CCI_REG16_LE(0x30DC)
#define IMX678_BLKLEVEL_DEFAULT         50

/* Digital Clamp */
#define IMX678_REG_DIGITAL_CLAMP        CCI_REG8(0x3458)

/* Analog gain control */
#define IMX678_REG_ANALOG_GAIN          CCI_REG16_LE(0x3070)
#define IMX678_ANA_GAIN_MIN_NORMAL      0
#define IMX678_ANA_GAIN_MAX_NORMAL      240
#define IMX678_ANA_GAIN_STEP            1
#define IMX678_ANA_GAIN_DEFAULT         0

/* Flip */
#define IMX678_REG_WINMODEH             CCI_REG8(0x3020)
#define IMX678_REG_WINMODEV             CCI_REG8(0x3021)

/* Common configuration registers */
#define IMX678_REG_WDMODE               CCI_REG8(0x301A)
#define IMX678_REG_ADDMODE              CCI_REG8(0x301B)
#define IMX678_REG_THIN_V_EN            CCI_REG8(0x301C)
#define IMX678_REG_VCMODE               CCI_REG8(0x301E)
#define IMX678_REG_ADBIT                CCI_REG8(0x3022)
#define IMX678_REG_MDBIT                CCI_REG8(0x3023)
#define IMX678_REG_GAIN_PGC_FIDMD       CCI_REG8(0x3400)

/* Test pattern generator */
#define IMX678_REG_TPG_EN_DUOUT		CCI_REG8(0x30E0)
#define IMX678_REG_TPG_PATSEL_DUOUT	CCI_REG8(0x30E2)
#define IMX678_TPG_ALL_000		0
#define IMX678_TPG_ALL_FFF		1
#define IMX678_TPG_ALL_555		2
#define IMX678_TPG_ALL_AAA		3
#define IMX678_TPG_TOG_555_AAA		4
#define IMX678_TPG_TOG_AAA_555		5
#define IMX678_TPG_TOG_000_555		6
#define IMX678_TPG_TOG_555_000		7
#define IMX678_TPG_TOG_000_FFF		8
#define IMX678_TPG_TOG_FFF_000		9
#define IMX678_TPG_H_COLOR_BARS		10
#define IMX678_TPG_V_COLOR_BARS		11
#define IMX678_REG_TPG_COLORWIDTH	CCI_REG8(0x30E4)
#define IMX678_TPG_COLORWIDTH_80PIX	0
#define IMX678_TPG_COLORWIDTH_160PIX	1
#define IMX678_TPG_COLORWIDTH_320PIX	2
#define IMX678_TPG_COLORWIDTH_640PIX	3

#define IMX678_PIXEL_RATE               74250000

/* imx678 native and active pixel array size. */
#define IMX678_NATIVE_WIDTH         3856U
#define IMX678_NATIVE_HEIGHT        2180U
#define IMX678_PIXEL_ARRAY_LEFT     8U
#define IMX678_PIXEL_ARRAY_TOP      8U
#define IMX678_PIXEL_ARRAY_WIDTH    3840U
#define IMX678_PIXEL_ARRAY_HEIGHT   2160U

/* Link frequency setup */
enum {
	IMX678_LINK_FREQ_297MHZ,  // 594Mbps/lane
	IMX678_LINK_FREQ_360MHZ,  // 720Mbps/lane
	IMX678_LINK_FREQ_445MHZ,  // 891Mbps/lane
	IMX678_LINK_FREQ_594MHZ,  // 1188Mbps/lane
	IMX678_LINK_FREQ_720MHZ,  // 1440Mbps/lane
	IMX678_LINK_FREQ_891MHZ,  // 1782Mbps/lane
	IMX678_LINK_FREQ_1039MHZ, // 2079Mbps/lane
	IMX678_LINK_FREQ_1188MHZ, // 2376Mbps/lane
};

static const u8 link_freqs_reg_value[] = {
	[IMX678_LINK_FREQ_297MHZ]  = 0x07,
	[IMX678_LINK_FREQ_360MHZ]  = 0x06,
	[IMX678_LINK_FREQ_445MHZ]  = 0x05,
	[IMX678_LINK_FREQ_594MHZ]  = 0x04,
	[IMX678_LINK_FREQ_720MHZ]  = 0x03,
	[IMX678_LINK_FREQ_891MHZ]  = 0x02,
	[IMX678_LINK_FREQ_1039MHZ] = 0x01,
	[IMX678_LINK_FREQ_1188MHZ] = 0x00,
};

static const u64 link_freqs[] = {
	[IMX678_LINK_FREQ_297MHZ]  = 297000000,
	[IMX678_LINK_FREQ_360MHZ]  = 360000000,
	[IMX678_LINK_FREQ_445MHZ]  = 445500000,
	[IMX678_LINK_FREQ_594MHZ]  = 594000000,
	[IMX678_LINK_FREQ_720MHZ]  = 720000000,
	[IMX678_LINK_FREQ_891MHZ]  = 891000000,
	[IMX678_LINK_FREQ_1039MHZ] = 1039500000,
	[IMX678_LINK_FREQ_1188MHZ] = 1188000000,
};

//min HMAX for 4-lane 4K full res mode, x2 for 2-lane, /2 for FHD
static const u16 HMAX_table_4lane_4K[] = {
	[IMX678_LINK_FREQ_297MHZ] = 1584,
	[IMX678_LINK_FREQ_360MHZ] = 1320,
	[IMX678_LINK_FREQ_445MHZ] = 1100,
	[IMX678_LINK_FREQ_594MHZ] =  792,
	[IMX678_LINK_FREQ_720MHZ] =  660,
	[IMX678_LINK_FREQ_891MHZ] =  550,
	[IMX678_LINK_FREQ_1039MHZ] = 440,
	[IMX678_LINK_FREQ_1188MHZ] = 396,
};

struct imx678_inck_cfg {
	u32 xclk_hz;   /* platform clock rate  */
	u8  inck_sel;  /* value for reg        */
};

static const struct imx678_inck_cfg imx678_inck_table[] = {
	{ 74250000, 0x00 },
	{ 37125000, 0x01 },
	{ 72000000, 0x02 },
	{ 27000000, 0x03 },
	{ 24000000, 0x04 },
	{ 36000000, 0x05 },
	{ 18000000, 0x06 },
	{ 13500000, 0x07 },
};

static const char * const imx678_tpg_menu[] = {
	"Disabled",
	"All 000h",
	"All FFFh",
	"All 555h",
	"All AAAh",
	"Toggle 555/AAAh",
	"Toggle AAA/555h",
	"Toggle 000/555h",
	"Toggle 555/000h",
	"Toggle 000/FFFh",
	"Toggle FFF/000h",
	"Horizontal color bars",
	"Vertical color bars",
};

static const int imx678_tpg_val[] = {
	IMX678_TPG_ALL_000,
	IMX678_TPG_ALL_000,
	IMX678_TPG_ALL_FFF,
	IMX678_TPG_ALL_555,
	IMX678_TPG_ALL_AAA,
	IMX678_TPG_TOG_555_AAA,
	IMX678_TPG_TOG_AAA_555,
	IMX678_TPG_TOG_000_555,
	IMX678_TPG_TOG_555_000,
	IMX678_TPG_TOG_000_FFF,
	IMX678_TPG_TOG_FFF_000,
	IMX678_TPG_H_COLOR_BARS,
	IMX678_TPG_V_COLOR_BARS,
};


struct imx678_reg_list {
	unsigned int num_of_regs;
	const struct cci_reg_sequence *regs;
};

/* Mode : resolution and related config&values */
struct imx678_mode {
	/* Frame width */
	unsigned int width;

	/* Frame height */
	unsigned int height;

	/* mode HMAX Scaling */
	u8   hmax_div;

	/* minimum H-timing */
	u16 min_HMAX;

	/* minimum V-timing */
	u64 min_VMAX;

	/* default H-timing */
	u16 default_HMAX;

	/* default V-timing */
	u64 default_VMAX;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* Default register values */
	struct imx678_reg_list reg_list;
};

/* IMX678 Register List */
/* Common Modes */
static const struct cci_reg_sequence common_regs[] = {
	{IMX678_REG_THIN_V_EN, 0x00}, {IMX678_REG_VCMODE, 0x01},
	{CCI_REG8(0x306B), 0x00}, {IMX678_REG_GAIN_PGC_FIDMD, 0x01},
	{CCI_REG8(0x3460), 0x22}, {CCI_REG8(0x355A), 0x64}, {CCI_REG8(0x3A02), 0x7A},
	{CCI_REG8(0x3A10), 0xEC}, {CCI_REG8(0x3A12), 0x71}, {CCI_REG8(0x3A14), 0xDE},
	{CCI_REG8(0x3A20), 0x2B}, {CCI_REG8(0x3A24), 0x22}, {CCI_REG8(0x3A25), 0x25},
	{CCI_REG8(0x3A26), 0x2A}, {CCI_REG8(0x3A27), 0x2C}, {CCI_REG8(0x3A28), 0x39},
	{CCI_REG8(0x3A29), 0x38}, {CCI_REG8(0x3A30), 0x04}, {CCI_REG8(0x3A31), 0x04},
	{CCI_REG8(0x3A32), 0x03}, {CCI_REG8(0x3A33), 0x03}, {CCI_REG8(0x3A34), 0x09},
	{CCI_REG8(0x3A35), 0x06}, {CCI_REG8(0x3A38), 0xCD}, {CCI_REG8(0x3A3A), 0x4C},
	{CCI_REG8(0x3A3C), 0xB9}, {CCI_REG8(0x3A3E), 0x30}, {CCI_REG8(0x3A40), 0x2C},
	{CCI_REG8(0x3A42), 0x39}, {CCI_REG8(0x3A4E), 0x00}, {CCI_REG8(0x3A52), 0x00},
	{CCI_REG8(0x3A56), 0x00}, {CCI_REG8(0x3A5A), 0x00}, {CCI_REG8(0x3A5E), 0x00},
	{CCI_REG8(0x3A62), 0x00}, {CCI_REG8(0x3A64), 0x00}, {CCI_REG8(0x3A6E), 0xA0},
	{CCI_REG8(0x3A70), 0x50}, {CCI_REG8(0x3A8C), 0x04}, {CCI_REG8(0x3A8D), 0x03},
	{CCI_REG8(0x3A8E), 0x09}, {CCI_REG8(0x3A90), 0x38}, {CCI_REG8(0x3A91), 0x42},
	{CCI_REG8(0x3A92), 0x3C}, {CCI_REG8(0x3B0E), 0xF3}, {CCI_REG8(0x3B12), 0xE5},
	{CCI_REG8(0x3B27), 0xC0}, {CCI_REG8(0x3B2E), 0xEF}, {CCI_REG8(0x3B30), 0x6A},
	{CCI_REG8(0x3B32), 0xF6}, {CCI_REG8(0x3B36), 0xE1}, {CCI_REG8(0x3B3A), 0xE8},
	{CCI_REG8(0x3B5A), 0x17}, {CCI_REG8(0x3B5E), 0xEF}, {CCI_REG8(0x3B60), 0x6A},
	{CCI_REG8(0x3B62), 0xF6}, {CCI_REG8(0x3B66), 0xE1}, {CCI_REG8(0x3B6A), 0xE8},
	{CCI_REG8(0x3B88), 0xEC}, {CCI_REG8(0x3B8A), 0xED}, {CCI_REG8(0x3B94), 0x71},
	{CCI_REG8(0x3B96), 0x72}, {CCI_REG8(0x3B98), 0xDE}, {CCI_REG8(0x3B9A), 0xDF},
	{CCI_REG8(0x3C0F), 0x06}, {CCI_REG8(0x3C10), 0x06}, {CCI_REG8(0x3C11), 0x06},
	{CCI_REG8(0x3C12), 0x06}, {CCI_REG8(0x3C13), 0x06}, {CCI_REG8(0x3C18), 0x20},
	{CCI_REG8(0x3C37), 0x10}, {CCI_REG8(0x3C3A), 0x7A}, {CCI_REG8(0x3C40), 0xF4},
	{CCI_REG8(0x3C48), 0xE6}, {CCI_REG8(0x3C54), 0xCE}, {CCI_REG8(0x3C56), 0xD0},
	{CCI_REG8(0x3C6C), 0x53}, {CCI_REG8(0x3C6E), 0x55}, {CCI_REG8(0x3C70), 0xC0},
	{CCI_REG8(0x3C72), 0xC2}, {CCI_REG8(0x3C7E), 0xCE}, {CCI_REG8(0x3C8C), 0xCF},
	{CCI_REG8(0x3C8E), 0xEB}, {CCI_REG8(0x3C98), 0x54}, {CCI_REG8(0x3C9A), 0x70},
	{CCI_REG8(0x3C9C), 0xC1}, {CCI_REG8(0x3C9E), 0xDD}, {CCI_REG8(0x3CB0), 0x7A},
	{CCI_REG8(0x3CB2), 0xBA}, {CCI_REG8(0x3CC8), 0xBC}, {CCI_REG8(0x3CCA), 0x7C},
	{CCI_REG8(0x3CD4), 0xEA}, {CCI_REG8(0x3CD5), 0x01}, {CCI_REG8(0x3CD6), 0x4A},
	{CCI_REG8(0x3CD8), 0x00}, {CCI_REG8(0x3CD9), 0x00}, {CCI_REG8(0x3CDA), 0xFF},
	{CCI_REG8(0x3CDB), 0x03}, {CCI_REG8(0x3CDC), 0x00}, {CCI_REG8(0x3CDD), 0x00},
	{CCI_REG8(0x3CDE), 0xFF}, {CCI_REG8(0x3CDF), 0x03}, {CCI_REG8(0x3CE4), 0x4C},
	{CCI_REG8(0x3CE6), 0xEC}, {CCI_REG8(0x3CE7), 0x01}, {CCI_REG8(0x3CE8), 0xFF},
	{CCI_REG8(0x3CE9), 0x03}, {CCI_REG8(0x3CEA), 0x00}, {CCI_REG8(0x3CEB), 0x00},
	{CCI_REG8(0x3CEC), 0xFF}, {CCI_REG8(0x3CED), 0x03}, {CCI_REG8(0x3CEE), 0x00},
	{CCI_REG8(0x3CEF), 0x00}, {CCI_REG8(0x3CF2), 0xFF}, {CCI_REG8(0x3CF3), 0x03},
	{CCI_REG8(0x3CF4), 0x00}, {CCI_REG8(0x3E28), 0x82}, {CCI_REG8(0x3E2A), 0x80},
	{CCI_REG8(0x3E30), 0x85}, {CCI_REG8(0x3E32), 0x7D}, {CCI_REG8(0x3E5C), 0xCE},
	{CCI_REG8(0x3E5E), 0xD3}, {CCI_REG8(0x3E70), 0x53}, {CCI_REG8(0x3E72), 0x58},
	{CCI_REG8(0x3E74), 0xC0}, {CCI_REG8(0x3E76), 0xC5}, {CCI_REG8(0x3E78), 0xC0},
	{CCI_REG8(0x3E79), 0x01}, {CCI_REG8(0x3E7A), 0xD4}, {CCI_REG8(0x3E7B), 0x01},
	{CCI_REG8(0x3EB4), 0x0B}, {CCI_REG8(0x3EB5), 0x02}, {CCI_REG8(0x3EB6), 0x4D},
	{CCI_REG8(0x3EB7), 0x42}, {CCI_REG8(0x3EEC), 0xF3}, {CCI_REG8(0x3EEE), 0xE7},
	{CCI_REG8(0x3F01), 0x01}, {CCI_REG8(0x3F24), 0x10}, {CCI_REG8(0x3F28), 0x2D},
	{CCI_REG8(0x3F2A), 0x2D}, {CCI_REG8(0x3F2C), 0x2D}, {CCI_REG8(0x3F2E), 0x2D},
	{CCI_REG8(0x3F30), 0x23}, {CCI_REG8(0x3F38), 0x2D}, {CCI_REG8(0x3F3A), 0x2D},
	{CCI_REG8(0x3F3C), 0x2D}, {CCI_REG8(0x3F3E), 0x28}, {CCI_REG8(0x3F40), 0x1E},
	{CCI_REG8(0x3F48), 0x2D}, {CCI_REG8(0x3F4A), 0x2D}, {CCI_REG8(0x3F4C), 0x00},
	{CCI_REG8(0x4004), 0xE4}, {CCI_REG8(0x4006), 0xFF}, {CCI_REG8(0x4018), 0x69},
	{CCI_REG8(0x401A), 0x84}, {CCI_REG8(0x401C), 0xD6}, {CCI_REG8(0x401E), 0xF1},
	{CCI_REG8(0x4038), 0xDE}, {CCI_REG8(0x403A), 0x00}, {CCI_REG8(0x403B), 0x01},
	{CCI_REG8(0x404C), 0x63}, {CCI_REG8(0x404E), 0x85}, {CCI_REG8(0x4050), 0xD0},
	{CCI_REG8(0x4052), 0xF2}, {CCI_REG8(0x4108), 0xDD}, {CCI_REG8(0x410A), 0xF7},
	{CCI_REG8(0x411C), 0x62}, {CCI_REG8(0x411E), 0x7C}, {CCI_REG8(0x4120), 0xCF},
	{CCI_REG8(0x4122), 0xE9}, {CCI_REG8(0x4138), 0xE6}, {CCI_REG8(0x413A), 0xF1},
	{CCI_REG8(0x414C), 0x6B}, {CCI_REG8(0x414E), 0x76}, {CCI_REG8(0x4150), 0xD8},
	{CCI_REG8(0x4152), 0xE3}, {CCI_REG8(0x417E), 0x03}, {CCI_REG8(0x417F), 0x01},
	{CCI_REG8(0x4186), 0xE0}, {CCI_REG8(0x4190), 0xF3}, {CCI_REG8(0x4192), 0xF7},
	{CCI_REG8(0x419C), 0x78}, {CCI_REG8(0x419E), 0x7C}, {CCI_REG8(0x41A0), 0xE5},
	{CCI_REG8(0x41A2), 0xE9}, {CCI_REG8(0x41C8), 0xE2}, {CCI_REG8(0x41CA), 0xFD},
	{CCI_REG8(0x41DC), 0x67}, {CCI_REG8(0x41DE), 0x82}, {CCI_REG8(0x41E0), 0xD4},
	{CCI_REG8(0x41E2), 0xEF}, {CCI_REG8(0x4200), 0xDE}, {CCI_REG8(0x4202), 0xDA},
	{CCI_REG8(0x4218), 0x63}, {CCI_REG8(0x421A), 0x5F}, {CCI_REG8(0x421C), 0xD0},
	{CCI_REG8(0x421E), 0xCC}, {CCI_REG8(0x425A), 0x82}, {CCI_REG8(0x425C), 0xEF},
	{CCI_REG8(0x4348), 0xFE}, {CCI_REG8(0x4349), 0x06}, {CCI_REG8(0x4352), 0xCE},
	{CCI_REG8(0x4420), 0x0B}, {CCI_REG8(0x4421), 0x02}, {CCI_REG8(0x4422), 0x4D},
	{CCI_REG8(0x4423), 0x0A}, {CCI_REG8(0x4426), 0xF5}, {CCI_REG8(0x442A), 0xE7},
	{CCI_REG8(0x4432), 0xF5}, {CCI_REG8(0x4436), 0xE7}, {CCI_REG8(0x4466), 0xB4},
	{CCI_REG8(0x446E), 0x32}, {CCI_REG8(0x449F), 0x1C}, {CCI_REG8(0x44A4), 0x2C},
	{CCI_REG8(0x44A6), 0x2C}, {CCI_REG8(0x44A8), 0x2C}, {CCI_REG8(0x44AA), 0x2C},
	{CCI_REG8(0x44B4), 0x2C}, {CCI_REG8(0x44B6), 0x2C}, {CCI_REG8(0x44B8), 0x2C},
	{CCI_REG8(0x44BA), 0x2C}, {CCI_REG8(0x44C4), 0x2C}, {CCI_REG8(0x44C6), 0x2C},
	{CCI_REG8(0x44C8), 0x2C}, {CCI_REG8(0x4506), 0xF3}, {CCI_REG8(0x450E), 0xE5},
	{CCI_REG8(0x4516), 0xF3}, {CCI_REG8(0x4522), 0xE5}, {CCI_REG8(0x4524), 0xF3},
	{CCI_REG8(0x452C), 0xE5}, {CCI_REG8(0x453C), 0x22}, {CCI_REG8(0x453D), 0x1B},
	{CCI_REG8(0x453E), 0x1B}, {CCI_REG8(0x453F), 0x15}, {CCI_REG8(0x4540), 0x15},
	{CCI_REG8(0x4541), 0x15}, {CCI_REG8(0x4542), 0x15}, {CCI_REG8(0x4543), 0x15},
	{CCI_REG8(0x4544), 0x15}, {CCI_REG8(0x4548), 0x00}, {CCI_REG8(0x4549), 0x01},
	{CCI_REG8(0x454A), 0x01}, {CCI_REG8(0x454B), 0x06}, {CCI_REG8(0x454C), 0x06},
	{CCI_REG8(0x454D), 0x06}, {CCI_REG8(0x454E), 0x06}, {CCI_REG8(0x454F), 0x06},
	{CCI_REG8(0x4550), 0x06}, {CCI_REG8(0x4554), 0x55}, {CCI_REG8(0x4555), 0x02},
	{CCI_REG8(0x4556), 0x42}, {CCI_REG8(0x4557), 0x05}, {CCI_REG8(0x4558), 0xFD},
	{CCI_REG8(0x4559), 0x05}, {CCI_REG8(0x455A), 0x94}, {CCI_REG8(0x455B), 0x06},
	{CCI_REG8(0x455D), 0x06}, {CCI_REG8(0x455E), 0x49}, {CCI_REG8(0x455F), 0x07},
	{CCI_REG8(0x4560), 0x7F}, {CCI_REG8(0x4561), 0x07}, {CCI_REG8(0x4562), 0xA5},
	{CCI_REG8(0x4564), 0x55}, {CCI_REG8(0x4565), 0x02}, {CCI_REG8(0x4566), 0x42},
	{CCI_REG8(0x4567), 0x05}, {CCI_REG8(0x4568), 0xFD}, {CCI_REG8(0x4569), 0x05},
	{CCI_REG8(0x456A), 0x94}, {CCI_REG8(0x456B), 0x06}, {CCI_REG8(0x456D), 0x06},
	{CCI_REG8(0x456E), 0x49}, {CCI_REG8(0x456F), 0x07}, {CCI_REG8(0x4572), 0xA5},
	{CCI_REG8(0x460C), 0x7D}, {CCI_REG8(0x460E), 0xB1}, {CCI_REG8(0x4614), 0xA8},
	{CCI_REG8(0x4616), 0xB2}, {CCI_REG8(0x461C), 0x7E}, {CCI_REG8(0x461E), 0xA7},
	{CCI_REG8(0x4624), 0xA8}, {CCI_REG8(0x4626), 0xB2}, {CCI_REG8(0x462C), 0x7E},
	{CCI_REG8(0x462E), 0x8A}, {CCI_REG8(0x4630), 0x94}, {CCI_REG8(0x4632), 0xA7},
	{CCI_REG8(0x4634), 0xFB}, {CCI_REG8(0x4636), 0x2F}, {CCI_REG8(0x4638), 0x81},
	{CCI_REG8(0x4639), 0x01}, {CCI_REG8(0x463A), 0xB5}, {CCI_REG8(0x463B), 0x01},
	{CCI_REG8(0x463C), 0x26}, {CCI_REG8(0x463E), 0x30}, {CCI_REG8(0x4640), 0xAC},
	{CCI_REG8(0x4641), 0x01}, {CCI_REG8(0x4642), 0xB6}, {CCI_REG8(0x4643), 0x01},
	{CCI_REG8(0x4644), 0xFC}, {CCI_REG8(0x4646), 0x25}, {CCI_REG8(0x4648), 0x82},
	{CCI_REG8(0x4649), 0x01}, {CCI_REG8(0x464A), 0xAB}, {CCI_REG8(0x464B), 0x01},
	{CCI_REG8(0x464C), 0x26}, {CCI_REG8(0x464E), 0x30}, {CCI_REG8(0x4654), 0xFC},
	{CCI_REG8(0x4656), 0x08}, {CCI_REG8(0x4658), 0x12}, {CCI_REG8(0x465A), 0x25},
	{CCI_REG8(0x4662), 0xFC}, {CCI_REG8(0x46A2), 0xFB}, {CCI_REG8(0x46D6), 0xF3},
	{CCI_REG8(0x46E6), 0x00}, {CCI_REG8(0x46E8), 0xFF}, {CCI_REG8(0x46E9), 0x03},
	{CCI_REG8(0x46EC), 0x7A}, {CCI_REG8(0x46EE), 0xE5}, {CCI_REG8(0x46F4), 0xEE},
	{CCI_REG8(0x46F6), 0xF2}, {CCI_REG8(0x470C), 0xFF}, {CCI_REG8(0x470D), 0x03},
	{CCI_REG8(0x470E), 0x00}, {CCI_REG8(0x4714), 0xE0}, {CCI_REG8(0x4716), 0xE4},
	{CCI_REG8(0x471E), 0xED}, {CCI_REG8(0x472E), 0x00}, {CCI_REG8(0x4730), 0xFF},
	{CCI_REG8(0x4731), 0x03}, {CCI_REG8(0x4734), 0x7B}, {CCI_REG8(0x4736), 0xDF},
	{CCI_REG8(0x4754), 0x7D}, {CCI_REG8(0x4756), 0x8B}, {CCI_REG8(0x4758), 0x93},
	{CCI_REG8(0x475A), 0xB1}, {CCI_REG8(0x475C), 0xFB}, {CCI_REG8(0x475E), 0x09},
	{CCI_REG8(0x4760), 0x11}, {CCI_REG8(0x4762), 0x2F}, {CCI_REG8(0x4766), 0xCC},
	{CCI_REG8(0x4776), 0xCB}, {CCI_REG8(0x477E), 0x4A}, {CCI_REG8(0x478E), 0x49},
	{CCI_REG8(0x4794), 0x7C}, {CCI_REG8(0x4796), 0x8F}, {CCI_REG8(0x4798), 0xB3},
	{CCI_REG8(0x4799), 0x00}, {CCI_REG8(0x479A), 0xCC}, {CCI_REG8(0x479C), 0xC1},
	{CCI_REG8(0x479E), 0xCB}, {CCI_REG8(0x47A4), 0x7D}, {CCI_REG8(0x47A6), 0x8E},
	{CCI_REG8(0x47A8), 0xB4}, {CCI_REG8(0x47A9), 0x00}, {CCI_REG8(0x47AA), 0xC0},
	{CCI_REG8(0x47AC), 0xFA}, {CCI_REG8(0x47AE), 0x0D}, {CCI_REG8(0x47B0), 0x31},
	{CCI_REG8(0x47B1), 0x01}, {CCI_REG8(0x47B2), 0x4A}, {CCI_REG8(0x47B3), 0x01},
	{CCI_REG8(0x47B4), 0x3F}, {CCI_REG8(0x47B6), 0x49}, {CCI_REG8(0x47BC), 0xFB},
	{CCI_REG8(0x47BE), 0x0C}, {CCI_REG8(0x47C0), 0x32}, {CCI_REG8(0x47C1), 0x01},
	{CCI_REG8(0x47C2), 0x3E}, {CCI_REG8(0x47C3), 0x01}, {IMX678_REG_WDMODE, 0x00},
	{IMX678_REG_ADBIT, 0x01}, {IMX678_REG_MDBIT, 0x01},
};

/* All pixel 4K60. 12-bit */
static const struct cci_reg_sequence mode_4k_regs_12bit[] = {
	{IMX678_REG_ADDMODE, 0x00},
};

/* 2x2 binned 1080p60. 12-bit */
static const struct cci_reg_sequence mode_1080_regs_12bit[] = {
	{IMX678_REG_ADDMODE, 0x01},
};

/* For Mode List:
 * Default:
 *   12Bit - FHD, 4K
 */

/* Mode configs */
struct imx678_mode supported_modes[] = {
	{
		/* 1080p60 2x2 binning */
		.width = 1928,
		.height = 1090,
		.hmax_div = 1,
		.min_HMAX = 366,
		.min_VMAX = IMX678_VMAX_DEFAULT,
		.default_HMAX = 366,
		.default_VMAX = IMX678_VMAX_DEFAULT,
		.crop = {
			.left = IMX678_PIXEL_ARRAY_LEFT,
			.top = IMX678_PIXEL_ARRAY_TOP,
			.width = IMX678_PIXEL_ARRAY_WIDTH,
			.height = IMX678_PIXEL_ARRAY_HEIGHT,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1080_regs_12bit),
			.regs = mode_1080_regs_12bit,
		},
	},
	{
		/* 4K60 All pixel */
		.width = 3856,
		.height = 2180,
		.min_HMAX = 550,
		.min_VMAX = IMX678_VMAX_DEFAULT,
		.default_HMAX = 550,
		.default_VMAX = IMX678_VMAX_DEFAULT,
		.hmax_div = 1,
		.crop = {
			.left = IMX678_PIXEL_ARRAY_LEFT,
			.top = IMX678_PIXEL_ARRAY_TOP,
			.width = IMX678_PIXEL_ARRAY_WIDTH,
			.height = IMX678_PIXEL_ARRAY_HEIGHT,
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_4k_regs_12bit),
			.regs = mode_4k_regs_12bit,
		},
	},
};


/*
 * The supported formats.
 * This table MUST contain 4 entries per format, to cover the various flip
 * combinations in the order
 * - no flip
 * - h flip
 * - v flip
 * - h&v flips
 */

/* 12bit Only */
static const u32 codes_normal[] = {
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SBGGR12_1X12,
	MEDIA_BUS_FMT_SBGGR12_1X12,
};

/* Flip isn’t relevant for mono */
static const u32 mono_codes[] = {
	MEDIA_BUS_FMT_Y16_1X16,   /* 16-bit mono */
	MEDIA_BUS_FMT_Y12_1X12,   /* 12-bit mono */
};

/* regulator supplies */
static const char * const imx678_supply_name[] = {
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (3.3V) supply */
	"VDIG",  /* Digital Core (1.1V) supply */
	"VDDL",  /* IF (1.8V) supply */
};

#define imx678_NUM_SUPPLIES ARRAY_SIZE(imx678_supply_name)

struct imx678 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct regmap *cci;

	unsigned int fmt_code;

	struct clk *xclk;
	u32 xclk_freq;

	/* chosen INCK_SEL register value */
	u8  inck_sel_val;

	/* Link configurations */
	unsigned int lane_count;
	unsigned int link_freq_idx;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[imx678_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;

	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;

	/* Current mode */
	const struct imx678_mode *mode;

	/* Tracking sensor VMAX/HMAX value */
	u16 HMAX;
	u32 VMAX;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;

	/* Rewrite common registers on stream on? */
	bool common_regs_written;
};


static inline struct imx678 *to_imx678(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx678, sd);
}

static inline void get_mode_table(struct imx678 *imx678, unsigned int code,
				  const struct imx678_mode **mode_list,
				  unsigned int *num_modes)
{
	*mode_list = NULL;
	*num_modes = 0;


	/* --- Color paths --- */
	switch (code) {
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		*mode_list = supported_modes;
		*num_modes = ARRAY_SIZE(supported_modes);
		break;
	default:
		*mode_list = NULL;
		*num_modes = 0;
	}

}

/* Hold register values until hold is disabled */
static inline void imx678_register_hold(struct imx678 *imx678, bool hold)
{
	cci_write(imx678->cci, IMX678_REG_HOLD, hold ? 1 : 0, NULL);
}

/* Get bayer order based on flip setting. */
static u32 imx678_get_format_code(struct imx678 *imx678, u32 code)
{
	unsigned int i;

	lockdep_assert_held(&imx678->mutex);

	for (i = 0; i < ARRAY_SIZE(codes_normal); i++)
		if (codes_normal[i] == code)
			break;

	if (i >= ARRAY_SIZE(codes_normal))
		i = 0;

	i = (i & ~3) | (imx678->vflip->val ? 2 : 0) |
	    (imx678->hflip->val ? 1 : 0);

	return codes_normal[i];
}

static void imx678_set_default_format(struct imx678 *imx678)
{
	/* Set default mode to max resolution */
	imx678->mode = &supported_modes[0];
	imx678->fmt_code = MEDIA_BUS_FMT_SRGGB12_1X12;
}

static int imx678_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx678 *imx678 = to_imx678(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_state_get_format(fh->state, 0);
	struct v4l2_rect *try_crop;

	mutex_lock(&imx678->mutex);

	/* Initialize try_fmt for the image pad */
	try_fmt->width = supported_modes[0].width;
	try_fmt->height = supported_modes[0].height;
	try_fmt->code = imx678_get_format_code(imx678, MEDIA_BUS_FMT_SRGGB12_1X12);
	try_fmt->field = V4L2_FIELD_NONE;

	/* Initialize try_crop */
	try_crop = v4l2_subdev_state_get_crop(fh->state, 0);
	try_crop->left = IMX678_PIXEL_ARRAY_LEFT;
	try_crop->top = IMX678_PIXEL_ARRAY_TOP;
	try_crop->width = IMX678_PIXEL_ARRAY_WIDTH;
	try_crop->height = IMX678_PIXEL_ARRAY_HEIGHT;

	mutex_unlock(&imx678->mutex);

	return 0;
}

static void imx678_update_hmax(struct imx678 *imx678)
{

	const u32 base_4lane = HMAX_table_4lane_4K[imx678->link_freq_idx];
	const u32 lane_scale = (imx678->lane_count == 2) ? 2 : 1;
	const u32 factor     = base_4lane * lane_scale;

	for (unsigned int i = 0; i < ARRAY_SIZE(supported_modes); ++i) {
		u32 h = factor / supported_modes[i].hmax_div;
		supported_modes[i].min_HMAX     = h;
		supported_modes[i].default_HMAX = h;
	}

}

static void imx678_set_framing_limits(struct imx678 *imx678)
{
	const struct imx678_mode *mode = imx678->mode;
	u64 default_hblank, max_hblank;
	u64 pixel_rate;

	imx678_update_hmax(imx678);

	imx678->VMAX = mode->default_VMAX;
	imx678->HMAX = mode->default_HMAX;

	pixel_rate = (u64)mode->width * IMX678_PIXEL_RATE;
	do_div(pixel_rate, mode->min_HMAX);
	__v4l2_ctrl_modify_range(imx678->pixel_rate, pixel_rate, pixel_rate, 1, pixel_rate);

	//int default_hblank = mode->default_HMAX*IMX678_PIXEL_RATE/72000000-IMX678_NATIVE_WIDTH;
	default_hblank = mode->default_HMAX * pixel_rate;
	do_div(default_hblank, IMX678_PIXEL_RATE);
	default_hblank = default_hblank - mode->width;

	max_hblank = IMX678_HMAX_MAX * pixel_rate;
	do_div(max_hblank, IMX678_PIXEL_RATE);
	max_hblank = max_hblank - mode->width;

	__v4l2_ctrl_modify_range(imx678->hblank, 0, max_hblank, 1, default_hblank);
	__v4l2_ctrl_s_ctrl(imx678->hblank, default_hblank);

	/* Update limits and set FPS to default */
	__v4l2_ctrl_modify_range(imx678->vblank, mode->min_VMAX - mode->height,
				 IMX678_VMAX_MAX - mode->height,
				 1, mode->default_VMAX - mode->height);
	__v4l2_ctrl_s_ctrl(imx678->vblank, mode->default_VMAX - mode->height);

	__v4l2_ctrl_modify_range(imx678->exposure, IMX678_EXPOSURE_MIN,
			 imx678->VMAX - IMX678_SHR_MIN_CLEARHDR, 1,
				IMX678_EXPOSURE_DEFAULT);

}

static int imx678_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx678 *imx678 = container_of(ctrl->handler, struct imx678, ctrl_handler);
	const struct imx678_mode *mode = imx678->mode;
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	int ret = 0;
	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE: {
		u32 shr = (imx678->VMAX - ctrl->val) & ~1u;

		ret = cci_write(imx678->cci, IMX678_REG_SHR, shr, NULL);
		break;
	}
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(imx678->cci, IMX678_REG_ANALOG_GAIN,
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK: {
		u32 current_exposure = imx678->exposure->cur.val;
		u32 minSHR = IMX678_SHR_MIN;

		imx678->VMAX = (mode->height + ctrl->val) & ~1u;

		current_exposure = clamp_t(u32, current_exposure, IMX678_EXPOSURE_MIN,
					   imx678->VMAX - minSHR);
		__v4l2_ctrl_modify_range(imx678->exposure, IMX678_EXPOSURE_MIN,
					 imx678->VMAX - minSHR, 1,
					 current_exposure);

		ret = cci_write(imx678->cci, IMX678_REG_VMAX, imx678->VMAX, NULL);
		break;
	}
	case V4L2_CID_HBLANK: {
		u64 pixel_rate;
		u64 hmax;

		pixel_rate = (u64)mode->width * IMX678_PIXEL_RATE;
		do_div(pixel_rate, mode->min_HMAX);
		hmax = (u64)(mode->width + ctrl->val) * IMX678_PIXEL_RATE;
		do_div(hmax, pixel_rate);
		imx678->HMAX = hmax;

		ret = cci_write(imx678->cci, IMX678_REG_HMAX, hmax, NULL);
		break;
	}
	case V4L2_CID_TEST_PATTERN: {
		cci_write(imx678->cci, IMX678_REG_TPG_COLORWIDTH,
			  IMX678_TPG_COLORWIDTH_160PIX, &ret);
		cci_write(imx678->cci, IMX678_REG_TPG_PATSEL_DUOUT,
			  imx678_tpg_val[ctrl->val], &ret);
		cci_write(imx678->cci, IMX678_REG_TPG_EN_DUOUT, (ctrl->val) ? 1 : 0,
			  &ret);
		break;
	}
	case V4L2_CID_HFLIP:
		ret = cci_write(imx678->cci, IMX678_REG_WINMODEH, ctrl->val, NULL);
		break;
	case V4L2_CID_VFLIP:
		ret = cci_write(imx678->cci, IMX678_REG_WINMODEV, ctrl->val, NULL);
		break;
	default:
		dev_warn(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx678_ctrl_ops = {
	.s_ctrl = imx678_set_ctrl,
};

static int imx678_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx678 *imx678 = to_imx678(sd);
	unsigned int entries;
	const u32 *tbl;

	tbl     = codes_normal;
	entries = ARRAY_SIZE(codes_normal) / 4;

	if (code->index >= entries)
		return -EINVAL;

	code->code = imx678_get_format_code(imx678, tbl[code->index * 4]);
	return 0;
}

static int imx678_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx678 *imx678 = to_imx678(sd);

	const struct imx678_mode *mode_list;
	unsigned int num_modes;

	get_mode_table(imx678, fse->code, &mode_list, &num_modes);

	if (fse->index >= num_modes)
		return -EINVAL;

	if (fse->code != imx678_get_format_code(imx678, fse->code))
		return -EINVAL;

	fse->min_width = mode_list[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = mode_list[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static void imx678_reset_colorspace(const struct imx678_mode *mode, struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void imx678_update_image_pad_format(struct imx678 *imx678,
					   const struct imx678_mode *mode,
					   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	imx678_reset_colorspace(mode, &fmt->format);
}

static int imx678_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx678 *imx678 = to_imx678(sd);

	mutex_lock(&imx678->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_state_get_format(sd_state, fmt->pad);
		/* update the code which could change due to vflip or hflip: */
		try_fmt->code = imx678_get_format_code(imx678, try_fmt->code);
		fmt->format = *try_fmt;
	} else {
		imx678_update_image_pad_format(imx678, imx678->mode, fmt);
		fmt->format.code =
			   imx678_get_format_code(imx678, imx678->fmt_code);
	}

	mutex_unlock(&imx678->mutex);
	return 0;
}


static int imx678_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt;
	const struct imx678_mode *mode;
	const struct imx678_mode *mode_list;
	unsigned int num_modes;
	struct imx678 *imx678 = to_imx678(sd);

	mutex_lock(&imx678->mutex);

	/* Bayer order varies with flips */
	fmt->format.code = imx678_get_format_code(imx678, fmt->format.code);
	get_mode_table(imx678, fmt->format.code, &mode_list, &num_modes);
	mode = v4l2_find_nearest_size(mode_list,
					  num_modes,
					  width, height,
					  fmt->format.width,
					  fmt->format.height);
	imx678_update_image_pad_format(imx678, mode, fmt);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_state_get_format(sd_state, fmt->pad);
		*framefmt = fmt->format;
	} else if (imx678->mode != mode ||
		   imx678->fmt_code != fmt->format.code) {
		imx678->mode = mode;
		imx678->fmt_code = fmt->format.code;
		imx678_set_framing_limits(imx678);
	}

	mutex_unlock(&imx678->mutex);

	return 0;
}

static const struct v4l2_rect *
__imx678_get_pad_crop(struct imx678 *imx678,
			  struct v4l2_subdev_state *sd_state,
			  unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_state_get_crop(sd_state, 0);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &imx678->mode->crop;
	}

	return NULL;
}

/* Start streaming */
static int imx678_start_streaming(struct imx678 *imx678)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	const struct imx678_reg_list *reg_list;
	int ret;

	if (!imx678->common_regs_written) {
		ret = cci_multi_reg_write(imx678->cci, common_regs,
					  ARRAY_SIZE(common_regs), NULL);
		if (ret) {
			dev_err(&client->dev, "%s failed to set common settings\n", __func__);
			return ret;
		}

		cci_write(imx678->cci, IMX678_REG_INCK_SEL, imx678->inck_sel_val, NULL);
		cci_write(imx678->cci, IMX678_REG_BLKLEVEL, IMX678_BLKLEVEL_DEFAULT, NULL);
		cci_write(imx678->cci, IMX678_REG_DATARATE_SEL,
			  link_freqs_reg_value[imx678->link_freq_idx], NULL);

		if (imx678->lane_count == 2)
			cci_write(imx678->cci, IMX678_REG_LANEMODE, 0x01, NULL);
		else
			cci_write(imx678->cci, IMX678_REG_LANEMODE, 0x03, NULL);

		/* Internal sync leader mode: enable XHS and XVS output */
		cci_write(imx678->cci, IMX678_REG_XXS_DRV, 0x00, NULL);
		cci_write(imx678->cci, IMX678_REG_XXS_OUTSEL, 0x0A, NULL);
		imx678->common_regs_written = true;
	}

	/* Apply default values of current mode */
	reg_list = &imx678->mode->reg_list;
	ret = cci_multi_reg_write(imx678->cci, reg_list->regs,
				  reg_list->num_of_regs, NULL);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	/* Apply customized values from user */
	ret = __v4l2_ctrl_handler_setup(imx678->sd.ctrl_handler);
	if (ret) {
		dev_err(&client->dev, "%s failed to apply user values\n", __func__);
		return ret;
	}

	/* Set stream on register */
	cci_write(imx678->cci, IMX678_REG_MODE_SELECT, IMX678_MODE_STREAMING, NULL);

	usleep_range(IMX678_STREAM_DELAY_US, IMX678_STREAM_DELAY_US + IMX678_STREAM_DELAY_RANGE_US);

	ret = cci_write(imx678->cci, IMX678_REG_XMSTA, 0x00, NULL);

	return ret;
}

/* Stop streaming */
static void imx678_stop_streaming(struct imx678 *imx678)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	int ret;

	cci_write(imx678->cci, IMX678_REG_XMSTA, 0x01, NULL);

	/* set stream off register */
	ret = cci_write(imx678->cci, IMX678_REG_MODE_SELECT, IMX678_MODE_STANDBY, NULL);
	if (ret)
		dev_err(&client->dev, "%s failed to stop stream\n", __func__);
}

static int imx678_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx678 *imx678 = to_imx678(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&imx678->mutex);
	if (imx678->streaming == enable) {
		mutex_unlock(&imx678->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto err_unlock;
		}

		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = imx678_start_streaming(imx678);
		if (ret)
			goto err_rpm_put;
	} else {
		imx678_stop_streaming(imx678);
		pm_runtime_put(&client->dev);
	}

	imx678->streaming = enable;

	/* vflip/hflip and hdr mode cannot change during streaming */
	__v4l2_ctrl_grab(imx678->vflip, enable);
	__v4l2_ctrl_grab(imx678->hflip, enable);

	mutex_unlock(&imx678->mutex);

	return ret;

err_rpm_put:
	pm_runtime_put(&client->dev);
err_unlock:
	mutex_unlock(&imx678->mutex);

	return ret;
}

/* Power/clock management functions */
static int imx678_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx678 *imx678 = to_imx678(sd);
	int ret;

	ret = regulator_bulk_enable(imx678_NUM_SUPPLIES,
					imx678->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(imx678->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}

	gpiod_set_value_cansleep(imx678->reset_gpio, 1);
	usleep_range(IMX678_XCLR_MIN_DELAY_US,
			 IMX678_XCLR_MIN_DELAY_US + IMX678_XCLR_DELAY_RANGE_US);

	return 0;

reg_off:
	regulator_bulk_disable(imx678_NUM_SUPPLIES, imx678->supplies);
	return ret;
}

static int imx678_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx678 *imx678 = to_imx678(sd);

	gpiod_set_value_cansleep(imx678->reset_gpio, 0);
	regulator_bulk_disable(imx678_NUM_SUPPLIES, imx678->supplies);
	clk_disable_unprepare(imx678->xclk);

	/* Force reprogramming of the common registers when powered up again. */
	imx678->common_regs_written = false;

	return 0;
}

static int imx678_get_regulators(struct imx678 *imx678)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	unsigned int i;

	for (i = 0; i < imx678_NUM_SUPPLIES; i++)
		imx678->supplies[i].supply = imx678_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
					   imx678_NUM_SUPPLIES,
					   imx678->supplies);
}

/* Verify chip ID */
static int imx678_check_module_exists(struct imx678 *imx678)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	int ret;
	u64 val;

	/* We don't actually have a CHIP ID register so we try to read from BLKLEVEL instead */
	ret = cci_read(imx678->cci, IMX678_REG_BLKLEVEL, &val, NULL);
	if (ret) {
		dev_err(&client->dev, "failed to read chip reg, with error %d\n", ret);
		return ret;
	}

	return 0;
}

static int imx678_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct imx678 *imx678 = to_imx678(sd);

		mutex_lock(&imx678->mutex);
		sel->r = *__imx678_get_pad_crop(imx678, sd_state, sel->pad, sel->which);
		mutex_unlock(&imx678->mutex);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = IMX678_NATIVE_WIDTH;
		sel->r.height = IMX678_NATIVE_HEIGHT;
		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = IMX678_PIXEL_ARRAY_LEFT;
		sel->r.top = IMX678_PIXEL_ARRAY_TOP;
		sel->r.width = IMX678_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX678_PIXEL_ARRAY_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static const struct v4l2_subdev_core_ops imx678_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops imx678_video_ops = {
	.s_stream = imx678_set_stream,
};

static const struct v4l2_subdev_pad_ops imx678_pad_ops = {
	.enum_mbus_code = imx678_enum_mbus_code,
	.get_fmt = imx678_get_pad_format,
	.set_fmt = imx678_set_pad_format,
	.get_selection = imx678_get_selection,
	.enum_frame_size = imx678_enum_frame_size,
};

static const struct v4l2_subdev_ops imx678_subdev_ops = {
	.core = &imx678_core_ops,
	.video = &imx678_video_ops,
	.pad = &imx678_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx678_internal_ops = {
	.open = imx678_open,
};

/* Initialize control handlers */
static int imx678_init_controls(struct imx678 *imx678)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct i2c_client *client = v4l2_get_subdevdata(&imx678->sd);
	struct v4l2_fwnode_device_properties props;
	int ret;

	ctrl_hdlr = &imx678->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 32);
	if (ret)
		return ret;

	mutex_init(&imx678->mutex);
	ctrl_hdlr->lock = &imx678->mutex;

	/*
	 * Create the controls here, but mode specific limits are setup
	 * in the imx678_set_framing_limits() call below.
	 */
	/* By default, PIXEL_RATE is read only */
	imx678->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx678_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       0xffff,
					       0xffff, 1,
					       0xffff);

	/* LINK_FREQ is also read only */
	imx678->link_freq =
		v4l2_ctrl_new_int_menu(ctrl_hdlr, &imx678_ctrl_ops,
				       V4L2_CID_LINK_FREQ, 0, 0,
				       &link_freqs[imx678->link_freq_idx]);
	if (imx678->link_freq)
		imx678->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx678->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx678_ctrl_ops,
					   V4L2_CID_VBLANK, 0, 0xfffff, 1, 0);
	imx678->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx678_ctrl_ops,
					   V4L2_CID_HBLANK, 0, 0xffff, 1, 0);

	imx678->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx678_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX678_EXPOSURE_MIN,
					     IMX678_EXPOSURE_MAX,
					     IMX678_EXPOSURE_STEP,
					     IMX678_EXPOSURE_DEFAULT);

	imx678->gain = v4l2_ctrl_new_std(ctrl_hdlr, &imx678_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
					 IMX678_ANA_GAIN_MIN_NORMAL, IMX678_ANA_GAIN_MAX_NORMAL,
					 IMX678_ANA_GAIN_STEP, IMX678_ANA_GAIN_DEFAULT);

	imx678->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx678_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (imx678->hflip)
		imx678->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	imx678->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx678_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (imx678->vflip)
		imx678->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx678_ctrl_ops, V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx678_tpg_menu) - 1, 0, 0,
				     imx678_tpg_menu);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx678_ctrl_ops, &props);
	if (ret)
		goto error;

	imx678->sd.ctrl_handler = ctrl_hdlr;

	/* Setup exposure and frame/line length limits. */
	imx678_set_framing_limits(imx678);

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&imx678->mutex);

	return ret;
}

static void imx678_free_controls(struct imx678 *imx678)
{
	v4l2_ctrl_handler_free(imx678->sd.ctrl_handler);
	mutex_destroy(&imx678->mutex);
}

static const struct of_device_id imx678_dt_ids[] = {
	{ .compatible = "sony,imx678"},
	{ /* sentinel */ }
};

static int imx678_check_hwcfg(struct device *dev, struct imx678 *imx678)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int ret = -EINVAL;
	int i;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg)) {
		dev_err(dev, "could not parse endpoint\n");
		goto error_out;
	}

	/* Check the number of MIPI CSI2 data lanes */
	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 2 && ep_cfg.bus.mipi_csi2.num_data_lanes != 4) {
		dev_err(dev, "only 2 or 4 data lanes are currently supported\n");
		goto error_out;
	}
	imx678->lane_count = ep_cfg.bus.mipi_csi2.num_data_lanes;

	/* Check the link frequency set in device tree */
	if (!ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}

	for (i = 0; i < ARRAY_SIZE(link_freqs); i++) {
		if (link_freqs[i] == ep_cfg.link_frequencies[0]) {
			imx678->link_freq_idx = i;
			break;
		}
	}

	if (i == ARRAY_SIZE(link_freqs)) {
		dev_err(dev, "Link frequency not supported: %lld\n",
			ep_cfg.link_frequencies[0]);
			ret = -EINVAL;
			goto error_out;
	}

	ret = 0;

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}

static int imx678_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx678 *imx678;
	const struct of_device_id *match;
	int ret, i;

	imx678 = devm_kzalloc(&client->dev, sizeof(*imx678), GFP_KERNEL);
	if (!imx678)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&imx678->sd, client, &imx678_subdev_ops);

	imx678->cci = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(imx678->cci))
		return dev_err_probe(dev, PTR_ERR(imx678->cci),
				     "failed to init CCI\n");

	match = of_match_device(imx678_dt_ids, dev);
	if (!match)
		return -ENODEV;

	/* Check the hardware configuration in device tree */
	if (imx678_check_hwcfg(dev, imx678))
		return -EINVAL;

	/* Get system clock (xclk) */
	imx678->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(imx678->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(imx678->xclk);
	}

	imx678->xclk_freq = clk_get_rate(imx678->xclk);

	for (i = 0; i < ARRAY_SIZE(imx678_inck_table); ++i) {
		if (imx678_inck_table[i].xclk_hz == imx678->xclk_freq) {
			imx678->inck_sel_val = imx678_inck_table[i].inck_sel;
			break;
		}
	}

	if (i == ARRAY_SIZE(imx678_inck_table)) {
		dev_err(dev, "unsupported XCLK rate %u Hz\n",
			imx678->xclk_freq);
		return -EINVAL;
	}

	ret = imx678_get_regulators(imx678);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	/* Request optional enable pin */
	imx678->reset_gpio = devm_gpiod_get_optional(dev, "reset",
							 GPIOD_OUT_HIGH);

	/*
	 * The sensor must be powered for imx678_check_module_exists()
	 * to be able to read register
	 */
	ret = imx678_power_on(dev);
	if (ret)
		return ret;

	ret = imx678_check_module_exists(imx678);
	if (ret)
		goto error_power_off;

	/* Initialize default format */
	imx678_set_default_format(imx678);

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	/* This needs the pm runtime to be registered. */
	ret = imx678_init_controls(imx678);
	if (ret)
		goto error_pm_runtime;

	/* Initialize subdev */
	imx678->sd.internal_ops = &imx678_internal_ops;
	imx678->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
				V4L2_SUBDEV_FL_HAS_EVENTS;
	imx678->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	imx678->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx678->sd.entity, 1, &imx678->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor(&imx678->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	return 0;

error_media_entity:
	media_entity_cleanup(&imx678->sd.entity);

error_handler_free:
	imx678_free_controls(imx678);

error_pm_runtime:
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);

error_power_off:
	imx678_power_off(&client->dev);

	return ret;
}

static void imx678_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx678 *imx678 = to_imx678(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx678_free_controls(imx678);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx678_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

MODULE_DEVICE_TABLE(of, imx678_dt_ids);

static const struct dev_pm_ops imx678_pm_ops = {
	SET_RUNTIME_PM_OPS(imx678_power_off, imx678_power_on, NULL)
};

static struct i2c_driver imx678_i2c_driver = {
	.driver = {
		.name = "imx678",
		.of_match_table = imx678_dt_ids,
		.pm = &imx678_pm_ops,
	},
	.probe = imx678_probe,
	.remove = imx678_remove,
};

module_i2c_driver(imx678_i2c_driver);

MODULE_AUTHOR("Will Whang <will@willwhang.com>");
MODULE_AUTHOR("Tetsuya NOMURA <tetsuya.nomura@soho-enterprise.com>");
MODULE_DESCRIPTION("Sony imx678 sensor driver");
MODULE_LICENSE("GPL");
