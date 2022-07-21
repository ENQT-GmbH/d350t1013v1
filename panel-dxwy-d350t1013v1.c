// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2022, ENQT GmbH.
 * Author: Sebastian Urban <surban@surban.net>
 */

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

/* Command2 BKx selection command */
#define DSI_CMD2BKX_SEL			0xFF

/* Command2, BK0 commands */
#define DSI_CMD2_BK0_PVGAMCTRL		0xB0 /* Positive Voltage Gamma Control */
#define DSI_CMD2_BK0_NVGAMCTRL		0xB1 /* Negative Voltage Gamma Control */
#define DSI_CMD2_BK0_LNESET		0xC0 /* Display Line setting */
#define DSI_CMD2_BK0_PORCTRL		0xC1 /* Porch control */
#define DSI_CMD2_BK0_INVSEL		0xC2 /* Inversion selection, Frame Rate Control */

/* Command2, BK1 commands */
#define DSI_CMD2_BK1_VRHS		0xB0 /* Vop amplitude setting */
#define DSI_CMD2_BK1_VCOM		0xB1 /* VCOM amplitude setting */
#define DSI_CMD2_BK1_VGHSS		0xB2 /* VGH Voltage setting */
#define DSI_CMD2_BK1_TESTCMD		0xB3 /* TEST Command Setting */
#define DSI_CMD2_BK1_VGLS		0xB5 /* VGL Voltage setting */
#define DSI_CMD2_BK1_PWCTLR1		0xB7 /* Power Control 1 */
#define DSI_CMD2_BK1_PWCTLR2		0xB8 /* Power Control 2 */
#define DSI_CMD2_BK1_SPD1		0xC1 /* Source pre_drive timing set1 */
#define DSI_CMD2_BK1_SPD2		0xC2 /* Source EQ2 Setting */
#define DSI_CMD2_BK1_MIPISET1		0xD0 /* MIPI Setting 1 */

/**
 * Command2 with BK function selection.
 *
 * BIT[4, 0]: [CN2, BKXSEL]
 * 10 = CMD2BK0, Command2 BK0
 * 11 = CMD2BK1, Command2 BK1
 * 00 = Command2 disable
 */
#define DSI_CMD2BK1_SEL			0x11
#define DSI_CMD2BK0_SEL			0x10
#define DSI_CMD2BKX_SEL_NONE		0x00

/* Command2, BK0 bytes */
#define DSI_LINESET_LINE		0x69
#define DSI_LINESET_LDE_EN		BIT(7)
#define DSI_LINESET_LINEDELTA		GENMASK(1, 0)
#define DSI_CMD2_BK0_LNESET_B1		DSI_LINESET_LINEDELTA
#define DSI_CMD2_BK0_LNESET_B0		(DSI_LINESET_LDE_EN | DSI_LINESET_LINE)
#define DSI_INVSEL_DEFAULT		GENMASK(5, 4)
#define DSI_INVSEL_NLINV		GENMASK(2, 0)
#define DSI_INVSEL_RTNI			GENMASK(2, 1)
#define DSI_CMD2_BK0_INVSEL_B1		DSI_INVSEL_RTNI
#define DSI_CMD2_BK0_INVSEL_B0		(DSI_INVSEL_DEFAULT | DSI_INVSEL_NLINV)
#define DSI_CMD2_BK0_PORCTRL_B0(m)	((m)->vtotal - (m)->vsync_end)
#define DSI_CMD2_BK0_PORCTRL_B1(m)	((m)->vsync_start - (m)->vdisplay)

/* Command2, BK1 bytes */
#define DSI_CMD2_BK1_VRHA_SET		0x45
#define DSI_CMD2_BK1_VCOM_SET		0x13
#define DSI_CMD2_BK1_VGHSS_SET		GENMASK(2, 0)
#define DSI_CMD2_BK1_TESTCMD_VAL	BIT(7)
#define DSI_VGLS_DEFAULT		BIT(6)
#define DSI_VGLS_SEL			GENMASK(2, 0)
#define DSI_CMD2_BK1_VGLS_SET		(DSI_VGLS_DEFAULT | DSI_VGLS_SEL)
#define DSI_PWCTLR1_AP			BIT(7) /* Gamma OP bias, max */
#define DSI_PWCTLR1_APIS		BIT(2) /* Source OP input bias, min */
#define DSI_PWCTLR1_APOS		BIT(0) /* Source OP output bias, min */
#define DSI_CMD2_BK1_PWCTLR1_SET	(DSI_PWCTLR1_AP | DSI_PWCTLR1_APIS | \
					DSI_PWCTLR1_APOS)
#define DSI_PWCTLR2_AVDD		BIT(5) /* AVDD 6.6v */
#define DSI_PWCTLR2_AVCL		0x0    /* AVCL -4.4v */
#define DSI_CMD2_BK1_PWCTLR2_SET	(DSI_PWCTLR2_AVDD | DSI_PWCTLR2_AVCL)
#define DSI_SPD1_T2D			BIT(3)
#define DSI_CMD2_BK1_SPD1_SET		(GENMASK(6, 4) | DSI_SPD1_T2D)
#define DSI_CMD2_BK1_SPD2_SET		DSI_CMD2_BK1_SPD1_SET
#define DSI_MIPISET1_EOT_EN		BIT(3)
#define DSI_CMD2_BK1_MIPISET1_SET	(BIT(7) | DSI_MIPISET1_EOT_EN)

struct st7701_panel_desc {
	const struct drm_display_mode *mode;
	unsigned int lanes;
	unsigned long flags;
	enum mipi_dsi_pixel_format format;
	const char *const *supply_names;
	unsigned int num_supplies;
	unsigned int panel_sleep_delay;
};

struct st7701 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	const struct st7701_panel_desc *desc;

	struct regulator_bulk_data *supplies;
	struct gpio_desc *reset;
	unsigned int sleep_delay;
};

static inline struct st7701 *panel_to_st7701(struct drm_panel *panel)
{
	return container_of(panel, struct st7701, panel);
}

static inline int st7701_dsi_write(struct st7701 *st7701, const void *seq,
				   size_t len)
{
	return mipi_dsi_dcs_write_buffer(st7701->dsi, seq, len);
}

#define ST7701_DSI(st7701, seq...)				\
	{							\
		const u8 d[] = { seq };				\
		st7701_dsi_write(st7701, d, ARRAY_SIZE(d));	\
	}

static void st7701_init_sequence(struct st7701 *st7701)
{
	const struct drm_display_mode *mode = st7701->desc->mode;


	dev_info(&st7701->dsi->dev, "st7701 init sequence new 2\n");

// 	ST7701_DSI(st7701, MIPI_DCS_SOFT_RESET, 0x00);
// 
// 	/* We need to wait 5ms before sending new commands */
// 	msleep(5);
// 
// 	ST7701_DSI(st7701, MIPI_DCS_EXIT_SLEEP_MODE, 0x00);
// 
// 	msleep(st7701->sleep_delay);
// 
// 	/* Command2, BK0 */
// 	ST7701_DSI(st7701, DSI_CMD2BKX_SEL,
// 		   0x77, 0x01, 0x00, 0x00, DSI_CMD2BK0_SEL);
// 	ST7701_DSI(st7701, DSI_CMD2_BK0_PVGAMCTRL, 0x00, 0x0E, 0x15, 0x0F,
// 		   0x11, 0x08, 0x08, 0x08, 0x08, 0x23, 0x04, 0x13, 0x12,
// 		   0x2B, 0x34, 0x1F);
// 	ST7701_DSI(st7701, DSI_CMD2_BK0_NVGAMCTRL, 0x00, 0x0E, 0x95, 0x0F,
// 		   0x13, 0x07, 0x09, 0x08, 0x08, 0x22, 0x04, 0x10, 0x0E,
// 		   0x2C, 0x34, 0x1F);
// 	ST7701_DSI(st7701, DSI_CMD2_BK0_LNESET,
// 		   DSI_CMD2_BK0_LNESET_B0, DSI_CMD2_BK0_LNESET_B1);
// 	ST7701_DSI(st7701, DSI_CMD2_BK0_PORCTRL,
// 		   DSI_CMD2_BK0_PORCTRL_B0(mode),
// 		   DSI_CMD2_BK0_PORCTRL_B1(mode));
// 	ST7701_DSI(st7701, DSI_CMD2_BK0_INVSEL,
// 		   DSI_CMD2_BK0_INVSEL_B0, DSI_CMD2_BK0_INVSEL_B1);
// 
// 	/* Command2, BK1 */
// 	ST7701_DSI(st7701, DSI_CMD2BKX_SEL,
// 			0x77, 0x01, 0x00, 0x00, DSI_CMD2BK1_SEL);
// 	ST7701_DSI(st7701, DSI_CMD2_BK1_VRHS, DSI_CMD2_BK1_VRHA_SET);
// 	ST7701_DSI(st7701, DSI_CMD2_BK1_VCOM, DSI_CMD2_BK1_VCOM_SET);
// 	ST7701_DSI(st7701, DSI_CMD2_BK1_VGHSS, DSI_CMD2_BK1_VGHSS_SET);
// 	ST7701_DSI(st7701, DSI_CMD2_BK1_TESTCMD, DSI_CMD2_BK1_TESTCMD_VAL);
// 	ST7701_DSI(st7701, DSI_CMD2_BK1_VGLS, DSI_CMD2_BK1_VGLS_SET);
// 	ST7701_DSI(st7701, DSI_CMD2_BK1_PWCTLR1, DSI_CMD2_BK1_PWCTLR1_SET);
// 	ST7701_DSI(st7701, DSI_CMD2_BK1_PWCTLR2, DSI_CMD2_BK1_PWCTLR2_SET);
// 	ST7701_DSI(st7701, DSI_CMD2_BK1_SPD1, DSI_CMD2_BK1_SPD1_SET);
// 	ST7701_DSI(st7701, DSI_CMD2_BK1_SPD2, DSI_CMD2_BK1_SPD2_SET);
// 	ST7701_DSI(st7701, DSI_CMD2_BK1_MIPISET1, DSI_CMD2_BK1_MIPISET1_SET);
// 
// 	/**
// 	 * ST7701_SPEC_V1.2 is unable to provide enough information above this
// 	 * specific command sequence, so grab the same from vendor BSP driver.
// 	 */
// 	ST7701_DSI(st7701, 0xE0, 0x00, 0x00, 0x02);
// 	ST7701_DSI(st7701, 0xE1, 0x0B, 0x00, 0x0D, 0x00, 0x0C, 0x00, 0x0E,
// 		   0x00, 0x00, 0x44, 0x44);
// 	ST7701_DSI(st7701, 0xE2, 0x33, 0x33, 0x44, 0x44, 0x64, 0x00, 0x66,
// 		   0x00, 0x65, 0x00, 0x67, 0x00, 0x00);
// 	ST7701_DSI(st7701, 0xE3, 0x00, 0x00, 0x33, 0x33);
// 	ST7701_DSI(st7701, 0xE4, 0x44, 0x44);
// 	ST7701_DSI(st7701, 0xE5, 0x0C, 0x78, 0x3C, 0xA0, 0x0E, 0x78, 0x3C,
// 		   0xA0, 0x10, 0x78, 0x3C, 0xA0, 0x12, 0x78, 0x3C, 0xA0);
// 	ST7701_DSI(st7701, 0xE6, 0x00, 0x00, 0x33, 0x33);
// 	ST7701_DSI(st7701, 0xE7, 0x44, 0x44);
// 	ST7701_DSI(st7701, 0xE8, 0x0D, 0x78, 0x3C, 0xA0, 0x0F, 0x78, 0x3C,
// 		   0xA0, 0x11, 0x78, 0x3C, 0xA0, 0x13, 0x78, 0x3C, 0xA0);
// 	ST7701_DSI(st7701, 0xEB, 0x02, 0x02, 0x39, 0x39, 0xEE, 0x44, 0x00);
// 	ST7701_DSI(st7701, 0xEC, 0x00, 0x00);
// 	ST7701_DSI(st7701, 0xED, 0xFF, 0xF1, 0x04, 0x56, 0x72, 0x3F, 0xFF,
// 		   0xFF, 0xFF, 0xFF, 0xF3, 0x27, 0x65, 0x40, 0x1F, 0xFF);
// 
// 	/* disable Command2 */
// 	ST7701_DSI(st7701, DSI_CMD2BKX_SEL,
// 		   0x77, 0x01, 0x00, 0x00, DSI_CMD2BKX_SEL_NONE);

// 	ST7701_DSI(st7701, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x13);
// 	ST7701_DSI(st7701, 0xEF, 0x08);
// 	ST7701_DSI(st7701, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x10);
// 	ST7701_DSI(st7701, 0xC0, 0x63, 0x00);
// 	ST7701_DSI(st7701, 0xC1, 0x10, 0x02);
// 	ST7701_DSI(st7701, 0xC2, 0x31, 0x02);
// 	ST7701_DSI(st7701, 0xCC, 0x10);
// 	ST7701_DSI(st7701, 0xB0, 0xC0, 0x0C, 0x92, 0x0C, 0x10, 0x05, 0x02, 0x0D, 0x07, 0x21, 0x04, 0x53, 0x11, 0x6A, 0x32, 0x1F);
// 	ST7701_DSI(st7701, 0xB1, 0xC0, 0x87, 0xCF, 0x0C, 0x10, 0x06, 0x00, 0x03, 0x08, 0x1D, 0x06, 0x54, 0x12, 0xE6, 0xEC, 0x0F);
// 	ST7701_DSI(st7701, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x11);
// 	ST7701_DSI(st7701, 0xB0, 0x5D);
// 	ST7701_DSI(st7701, 0xB1, 0x62);
// 	ST7701_DSI(st7701, 0xB2, 0x82);
// 	ST7701_DSI(st7701, 0xB3, 0x80);
// 	ST7701_DSI(st7701, 0xB5, 0x42);
// 	ST7701_DSI(st7701, 0xB7, 0x85);
// 	ST7701_DSI(st7701, 0xB8, 0x20);
// 	ST7701_DSI(st7701, 0xC0, 0x09);
// 	ST7701_DSI(st7701, 0xC1, 0x78);
// 	ST7701_DSI(st7701, 0xC2, 0x78);
// 	ST7701_DSI(st7701, 0xD0, 0x88);
// 	ST7701_DSI(st7701, 0xEE, 0x42);
// 	msleep(100);
// 
// 	ST7701_DSI(st7701, 0xE0, 0x00, 0x00, 0x02);
// 	ST7701_DSI(st7701, 0xE1, 0x04, 0xA0, 0x06, 0xA0, 0x05, 0xA0, 0x07, 0xA0, 0x00, 0x44, 0x44);
// 	ST7701_DSI(st7701, 0xE2, 0x00, 0x00, 0x33, 0x33, 0x01, 0xA0, 0x00, 0x00, 0x01, 0xA0, 0x00, 0x00);
// 	ST7701_DSI(st7701, 0xE3, 0x00, 0x00, 0x33, 0x33);
// 	ST7701_DSI(st7701, 0xE4, 0x44, 0x44);
// 	ST7701_DSI(st7701, 0xE5, 0x0C, 0x30, 0xA0, 0xA0, 0x0E, 0x32, 0xA0, 0xA0, 0x08, 0x2C, 0xA0, 0xA0, 0x0A, 0x2E, 0xA0, 0xA0);
// 	ST7701_DSI(st7701, 0xE6, 0x00, 0x00, 0x33, 0x33);
// 	ST7701_DSI(st7701, 0xE7, 0x44, 0x44);
// 	ST7701_DSI(st7701, 0xE8, 0x0D, 0x31, 0xA0, 0xA0, 0x0F, 0x33, 0xA0, 0xA0, 0x09, 0x2D, 0xA0, 0xA0, 0x0B, 0x2F, 0xA0, 0xA0);
// 	ST7701_DSI(st7701, 0xEB, 0x00, 0x01, 0xE4, 0xE4, 0x44, 0x88, 0x00);
// 	ST7701_DSI(st7701, 0xED, 0xFF, 0xF5, 0x47, 0x6F, 0x0B, 0xA1, 0xA2, 0xBF, 0xFB, 0x2A, 0x1A, 0xB0, 0xF6, 0x74, 0x5F, 0xFF);
// 	ST7701_DSI(st7701, 0xEF, 0x08, 0x08, 0x08, 0x40, 0x3F, 0x64);
// 	ST7701_DSI(st7701, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x13);
// 	ST7701_DSI(st7701, 0xE8, 0x00, 0x0E);
// 	ST7701_DSI(st7701, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x00);
// 	ST7701_DSI(st7701, 0x11);
// 	msleep(200);
// 
// 	ST7701_DSI(st7701, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x13);
// 	ST7701_DSI(st7701, 0xE8, 0x00, 0x0C);
// 	msleep(10);
// 
// 	ST7701_DSI(st7701, 0xE8, 0x00, 0x00);
// 	ST7701_DSI(st7701, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x00);
// 	ST7701_DSI(st7701, 0x3A, 0x50);
// 	ST7701_DSI(st7701, 0x29);
// 	msleep(50);

	// new sequence:

	// Software reset.
	ST7701_DSI(st7701, 0x01);
	msleep(500);

	// Sleep out.
	ST7701_DSI(st7701, 0x11);
	msleep(500);

	// Read Display Id.
	char buf[3];
	mipi_dsi_dcs_read(st7701->dsi, 0x04, buf, 3);
	dev_info(&st7701->dsi->dev, "st7701 display id: %02x %02x %02x\n", buf[0], buf[1], buf[2]);

	// ==============
// 	ST7701_DSI(st7701, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x11);
// 	ST7701_DSI(st7701, 0xD1, 0x11);
// 	ST7701_DSI(st7701, 0x55, 0xb0);
// 	ST7701_DSI(st7701, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x10);
// 	ST7701_DSI(st7701, 0xC0, 0x63, 0x00);
// 	ST7701_DSI(st7701, 0xC1, 0x09, 0x02);
// 	ST7701_DSI(st7701, 0xC2, 0x31, 0x08);
// 	ST7701_DSI(st7701, 0xCC, 0x18);
// 	ST7701_DSI(st7701, 0xB0, 0x00, 0x11, 0x19, 0x0C, 0x10, 0x06, 0x07, 0x0A, 0x09, 0x22, 0x04, 0x10, 0x0E, 0x28, 0x30, 0x1C);
// 	ST7701_DSI(st7701, 0xB1, 0x00, 0x12, 0x19, 0x0D, 0x10, 0x04, 0x06, 0x07, 0x08, 0x23, 0x04, 0x12, 0x11, 0x28, 0x30, 0x1C);
// 	ST7701_DSI(st7701, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x11);
// 	ST7701_DSI(st7701, 0xB0, 0x4D);
// 	ST7701_DSI(st7701, 0xB1, 0x6A);
// 	ST7701_DSI(st7701, 0xB2, 0x07);
// 	ST7701_DSI(st7701, 0xB3, 0x80);
// 	ST7701_DSI(st7701, 0xB5, 0x47);
// 	ST7701_DSI(st7701, 0xB7, 0x8A);
// 	ST7701_DSI(st7701, 0xB8, 0x21);
// 	ST7701_DSI(st7701, 0xB9, 0x10);
// 	ST7701_DSI(st7701, 0xC1, 0x78);
// 	ST7701_DSI(st7701, 0xC2, 0x78);
// 	ST7701_DSI(st7701, 0xD0, 0x88);
// 	msleep(500);
// 
// 	ST7701_DSI(st7701, 0xE0, 0x00, 0x00, 0x02);
// 	ST7701_DSI(st7701, 0xE1, 0x01, 0xA0, 0x03, 0xA0, 0x02, 0xA0, 0x04, 0xA0, 0x00, 0x44, 0x44);
// 	ST7701_DSI(st7701, 0xE2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
// 	ST7701_DSI(st7701, 0xE3, 0x00, 0x00, 0x33, 0x33);
// 	ST7701_DSI(st7701, 0xE4, 0x44, 0x44);
// 	ST7701_DSI(st7701, 0xE5, 0x01, 0x26, 0xA0, 0xA0, 0x03, 0x28, 0xA0, 0xA0, 0x05, 0x2A, 0xA0, 0xA0, 0x07, 0x2C, 0xA0, 0xA0);
// 	ST7701_DSI(st7701, 0xE6, 0x00, 0x00, 0x33, 0x33);
// 	ST7701_DSI(st7701, 0xE7, 0x44, 0x44);
// 	ST7701_DSI(st7701, 0xE8, 0x02, 0x26, 0xA0, 0xA0, 0x04, 0x28, 0xA0, 0xA0, 0x06, 0x2A, 0xA0, 0xA0, 0x08, 0x2C, 0xA0, 0xA0);
// 	ST7701_DSI(st7701, 0xEB, 0x00, 0x01, 0xE4, 0xE4, 0x44, 0x00, 0x40);
// 	ST7701_DSI(st7701, 0xED, 0xFF, 0xF7, 0x65, 0x4F, 0x0B, 0xA1, 0xCF, 0xFF, 0xFF, 0xFC, 0x1A, 0xB0, 0xF4, 0x56, 0x7F, 0xFF);
// 	ST7701_DSI(st7701, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x00);
// 	msleep(500);
	// ===============

	ST7701_DSI(st7701, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x13);
	ST7701_DSI(st7701, 0xEF, 0x08);
	ST7701_DSI(st7701, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x10);
	ST7701_DSI(st7701, 0xC0, 0x63, 0x00);
	ST7701_DSI(st7701, 0xC1, 0x10, 0x02);
	ST7701_DSI(st7701, 0xC2, 0x31, 0x02);
	ST7701_DSI(st7701, 0xCC, 0x10);
	ST7701_DSI(st7701, 0xB0, 0xC0, 0x0C, 0x92, 0x0C, 0x10, 0x05, 0x02, 0x0D, 0x07, 0x21, 0x04, 0x53, 0x11, 0x6A, 0x32, 0x1F);
	ST7701_DSI(st7701, 0xB1, 0xC0, 0x87, 0xCF, 0x0C, 0x10, 0x06, 0x00, 0x03, 0x08, 0x1D, 0x06, 0x54, 0x12, 0xE6, 0xEC, 0x0F);
	ST7701_DSI(st7701, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x11);
	ST7701_DSI(st7701, 0xB0, 0x5D);
	ST7701_DSI(st7701, 0xB1, 0x62);
	ST7701_DSI(st7701, 0xB2, 0x82);
	ST7701_DSI(st7701, 0xB3, 0x80);
	ST7701_DSI(st7701, 0xB5, 0x42);
	ST7701_DSI(st7701, 0xB7, 0x85);
	ST7701_DSI(st7701, 0xB8, 0x20);
	ST7701_DSI(st7701, 0xC0, 0x09);
	ST7701_DSI(st7701, 0xC1, 0x78);
	ST7701_DSI(st7701, 0xC2, 0x78);
	ST7701_DSI(st7701, 0xD0, 0x88);
	ST7701_DSI(st7701, 0xEE, 0x42);
	msleep(500);

	ST7701_DSI(st7701, 0xE0, 0x00, 0x00, 0x02);
	ST7701_DSI(st7701, 0xE1, 0x04, 0xA0, 0x06, 0xA0, 0x05, 0xA0, 0x07, 0xA0, 0x00, 0x44, 0x44);
	ST7701_DSI(st7701, 0xE2, 0x00, 0x00, 0x33, 0x33, 0x01, 0xA0, 0x00, 0x00, 0x01, 0xA0, 0x00, 0x00);
	ST7701_DSI(st7701, 0xE3, 0x00, 0x00, 0x33, 0x33);
	ST7701_DSI(st7701, 0xE4, 0x44, 0x44);
	ST7701_DSI(st7701, 0xE5, 0x0C, 0x30, 0xA0, 0xA0, 0x0E, 0x32, 0xA0, 0xA0, 0x08, 0x2C, 0xA0, 0xA0, 0x0A, 0x2E, 0xA0, 0xA0);
	ST7701_DSI(st7701, 0xE6, 0x00, 0x00, 0x33, 0x33);
	ST7701_DSI(st7701, 0xE7, 0x44, 0x44);
	ST7701_DSI(st7701, 0xE8, 0x0D, 0x31, 0xA0, 0xA0, 0x0F, 0x33, 0xA0, 0xA0, 0x09, 0x2D, 0xA0, 0xA0, 0x0B, 0x2F, 0xA0, 0xA0);
	ST7701_DSI(st7701, 0xEB, 0x00, 0x01, 0xE4, 0xE4, 0x44, 0x88, 0x00);
	ST7701_DSI(st7701, 0xED, 0xFF, 0xF5, 0x47, 0x6F, 0x0B, 0xA1, 0xA2, 0xBF, 0xFB, 0x2A, 0x1A, 0xB0, 0xF6, 0x74, 0x5F, 0xFF);
	ST7701_DSI(st7701, 0xEF, 0x08, 0x08, 0x08, 0x40, 0x3F, 0x64);
	ST7701_DSI(st7701, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x13);
	ST7701_DSI(st7701, 0xE8, 0x00, 0x0E);
	ST7701_DSI(st7701, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x00);
	ST7701_DSI(st7701, 0x11);
	msleep(500);

	ST7701_DSI(st7701, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x13);
	ST7701_DSI(st7701, 0xE8, 0x00, 0x0C);
	msleep(500);

	ST7701_DSI(st7701, 0xE8, 0x00, 0x00);
	ST7701_DSI(st7701, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x00);
	ST7701_DSI(st7701, 0x3A, 0x50);
	ST7701_DSI(st7701, 0x29);
	msleep(500);

	// Display on.
	ST7701_DSI(st7701, 0x29);
	msleep(500);

	// dev_info(&st7701->dsi->dev, "st7701 init all pixel on\n");
	// ST7701_DSI(st7701, 0x23);
	// msleep(500);

	// Read Display Id.
	mipi_dsi_dcs_read(st7701->dsi, 0x04, buf, 3);
	dev_info(&st7701->dsi->dev, "st7701 display id: %02x %02x %02x\n", buf[0], buf[1], buf[2]);

	// Read display power mode.
	mipi_dsi_dcs_read(st7701->dsi, 0x0a, buf, 1);
	dev_info(&st7701->dsi->dev, "st7701 power mode: %02x\n", buf[0]);

	// Read display pixel format.
	mipi_dsi_dcs_read(st7701->dsi, 0x0c, buf, 1);
	dev_info(&st7701->dsi->dev, "st7701 pixel format: %02x\n", buf[0]);

	// Read display image mode.
	mipi_dsi_dcs_read(st7701->dsi, 0x0d, buf, 1);
	dev_info(&st7701->dsi->dev, "st7701 image mode: %02x\n", buf[0]);

	// Read display signal mode.
	mipi_dsi_dcs_read(st7701->dsi, 0x0e, buf, 1);
	dev_info(&st7701->dsi->dev, "st7701 signal mode: %02x\n", buf[0]);


	dev_info(&st7701->dsi->dev, "st7701 init sequence done\n");

}

static int st7701_prepare(struct drm_panel *panel)
{
	struct st7701 *st7701 = panel_to_st7701(panel);
	int ret;

	dev_info(panel->dev, "st7701 prepare 10\n");

	dev_info(panel->dev, "st7701 regulator\n");

	ret = regulator_bulk_enable(st7701->desc->num_supplies,
				    st7701->supplies);
	if (ret < 0)
		return ret;
	msleep(500);

	dev_info(panel->dev, "st7701 reset\n");
	gpiod_set_value(st7701->reset, 0);
	msleep(500);
	gpiod_set_value(st7701->reset, 1);
	msleep(500);
	// reset verified okay!

	//mipi_dsi_dcs_soft_reset(st7701->dsi);
	//msleep(500);

	//mipi_dsi_dcs_exit_sleep_mode(st7701->dsi);
	//msleep(500);

	dev_info(panel->dev, "st7701 call init\n");
	msleep(500);
	st7701_init_sequence(st7701);
	msleep(500);
	dev_info(panel->dev, "st7701 call init done\n");

	//mipi_dsi_dcs_set_tear_on(st7701->dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	//mipi_dsi_dcs_exit_sleep_mode(st7701->dsi);
	//msleep(120);

	return 0;
}

static int st7701_enable(struct drm_panel *panel)
{
	struct st7701 *st7701 = panel_to_st7701(panel);

	dev_info(panel->dev, "st7701 enable\n");
	//mipi_dsi_dcs_set_display_on(st7701->dsi);
	//ST7701_DSI(st7701, MIPI_DCS_SET_DISPLAY_ON, 0x00);

// 	dev_info(panel->dev, "st7701 reset 2\n");
// 	gpiod_set_value(st7701->reset, 0);
// 	msleep(500);
// 	gpiod_set_value(st7701->reset, 1);
// 	msleep(500);
// 
// 	dev_info(panel->dev, "st7701 call init\n");
// 	msleep(500);
// 	st7701_init_sequence(st7701);
// 	msleep(500);
// 	dev_info(panel->dev, "st7701 call init done\n");

	dev_info(panel->dev, "st7701 enable done\n");

	return 0;
}

static int st7701_disable(struct drm_panel *panel)
{
	struct st7701 *st7701 = panel_to_st7701(panel);

	dev_info(panel->dev, "st7701 disable\n");
	mipi_dsi_dcs_set_display_off(st7701->dsi);

	//ST7701_DSI(st7701, MIPI_DCS_SET_DISPLAY_OFF, 0x00);

	return 0;
}

static int st7701_unprepare(struct drm_panel *panel)
{
	struct st7701 *st7701 = panel_to_st7701(panel);

	dev_info(panel->dev, "st7701 unprepare\n");

	//ST7701_DSI(st7701, MIPI_DCS_ENTER_SLEEP_MODE, 0x00);

	msleep(st7701->sleep_delay);

	gpiod_set_value(st7701->reset, 0);

	/**
	 * During the Resetting period, the display will be blanked
	 * (The display is entering blanking sequence, which maximum
	 * time is 120 ms, when Reset Starts in Sleep Out –mode. The
	 * display remains the blank state in Sleep In –mode.) and
	 * then return to Default condition for Hardware Reset.
	 *
	 * So we need wait sleep_delay time to make sure reset completed.
	 */
	msleep(st7701->sleep_delay);

	regulator_bulk_disable(st7701->desc->num_supplies, st7701->supplies);

	return 0;
}

static int st7701_get_modes(struct drm_panel *panel,
			    struct drm_connector *connector)
{
	struct st7701 *st7701 = panel_to_st7701(panel);
	const struct drm_display_mode *desc_mode = st7701->desc->mode;
	struct drm_display_mode *mode;

	dev_info(panel->dev, "st7701 get modes\n");

	mode = drm_mode_duplicate(connector->dev, desc_mode);
	if (!mode) {
		dev_err(&st7701->dsi->dev, "failed to add mode %ux%u@%u\n",
			desc_mode->hdisplay, desc_mode->vdisplay,
			drm_mode_vrefresh(desc_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = desc_mode->width_mm;
	connector->display_info.height_mm = desc_mode->height_mm;

	dev_info(panel->dev, "st7701 get modes done\n");

	return 1;
}

static const struct drm_panel_funcs st7701_funcs = {
	.disable	= st7701_disable,
	.unprepare	= st7701_unprepare,
	.prepare	= st7701_prepare,
	.enable		= st7701_enable,
	.get_modes	= st7701_get_modes,
};

static const struct drm_display_mode ts8550b_mode = {
	//.clock		= 27500,
	//.clock		= 25000,
	.clock 		= 1000,

	.hdisplay	= 480,
	.hsync_start	= 480 + 18,
	.hsync_end	= 480 + 18 + 16,
	.htotal		= 480 + 18 + 16 + 2,

	.vdisplay	= 800,
	.vsync_start	= 800 + 16,
	.vsync_end	= 800 + 16 + 14,
	.vtotal		= 800 + 16 + 14 + 2,

	.width_mm	= 45,
	.height_mm	= 76,

	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static const char * const ts8550b_supply_names[] = {
	"VCC",
	"IOVCC",
};

static const struct st7701_panel_desc ts8550b_desc = {
	.mode = &ts8550b_mode,
	.lanes = 2,
	.flags = MIPI_DSI_MODE_VIDEO,
	.format = MIPI_DSI_FMT_RGB888,
	.supply_names = ts8550b_supply_names,
	.num_supplies = ARRAY_SIZE(ts8550b_supply_names),
	.panel_sleep_delay = 80, /* panel need extra 80ms for sleep out cmd */
};

static int st7701_dsi_probe(struct mipi_dsi_device *dsi)
{
	const struct st7701_panel_desc *desc;
	struct st7701 *st7701;
	int ret, i;

	dev_info(&dsi->dev, "st7701 probe version 2\n");

	st7701 = devm_kzalloc(&dsi->dev, sizeof(*st7701), GFP_KERNEL);
	if (!st7701)
		return -ENOMEM;

	desc = of_device_get_match_data(&dsi->dev);
	dsi->mode_flags = desc->flags;
	dsi->format = desc->format;
	dsi->lanes = desc->lanes;

	st7701->supplies = devm_kcalloc(&dsi->dev, desc->num_supplies,
					sizeof(*st7701->supplies),
					GFP_KERNEL);
	if (!st7701->supplies)
		return -ENOMEM;

	for (i = 0; i < desc->num_supplies; i++)
		st7701->supplies[i].supply = desc->supply_names[i];

	dev_info(&dsi->dev, "st7701 probe: regulator\n");
	ret = devm_regulator_bulk_get(&dsi->dev, desc->num_supplies,
				      st7701->supplies);
	if (ret < 0)
		return ret;

	dev_info(&dsi->dev, "st7701 probe: reset\n");
	st7701->reset = devm_gpiod_get(&dsi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(st7701->reset)) {
		dev_err(&dsi->dev, "Couldn't get our reset GPIO\n");
		return PTR_ERR(st7701->reset);
	}

	dev_info(&dsi->dev, "st7701 probe: panel init\n");
	drm_panel_init(&st7701->panel, &dsi->dev, &st7701_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	/**
	 * Once sleep out has been issued, ST7701 IC required to wait 120ms
	 * before initiating new commands.
	 *
	 * On top of that some panels might need an extra delay to wait, so
	 * add panel specific delay for those cases. As now this panel specific
	 * delay information is referenced from those panel BSP driver, example
	 * ts8550b and there is no valid documentation for that.
	 */
	st7701->sleep_delay = 120 + desc->panel_sleep_delay;

	dev_info(&dsi->dev, "st7701 probe: backlight\n");
	ret = drm_panel_of_backlight(&st7701->panel);
	if (ret)
		return ret;

	drm_panel_add(&st7701->panel);

	mipi_dsi_set_drvdata(dsi, st7701);
	st7701->dsi = dsi;
	st7701->desc = desc;

    	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE
		| MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MSG_USE_LPM; // only supported mode
	//dsi->mode_flags = MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
    	dsi->format = MIPI_DSI_FMT_RGB888;
    	dsi->lanes = 2;
	//dsi->lanes = 1;

	dev_info(&dsi->dev, "st7701 probe done 2\n");

	return mipi_dsi_attach(dsi);
}

static int st7701_dsi_remove(struct mipi_dsi_device *dsi)
{
	struct st7701 *st7701 = mipi_dsi_get_drvdata(dsi);

	dev_info(&dsi->dev, "st7701 remove\n");

	mipi_dsi_detach(dsi);
	drm_panel_remove(&st7701->panel);

	return 0;
}

static const struct of_device_id st7701_of_match[] = {
	{ .compatible = "techstar,ts8550b", .data = &ts8550b_desc },
	{ }
};
MODULE_DEVICE_TABLE(of, st7701_of_match);

static struct mipi_dsi_driver st7701_dsi_driver = {
	.probe		= st7701_dsi_probe,
	.remove		= st7701_dsi_remove,
	.driver = {
		.name		= "st7701",
		.of_match_table	= st7701_of_match,
	},
};
module_mipi_dsi_driver(st7701_dsi_driver);

MODULE_AUTHOR("Jagan Teki <jagan@amarulasolutions.com>");
MODULE_DESCRIPTION("Sitronix ST7701 LCD Panel Driver");
MODULE_LICENSE("GPL");
