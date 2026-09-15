// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for panels based on Sitronix ST7703 controller, such as:
 *
 * - Rocktech jh057n00900 5.5" MIPI-DSI panel
 * - DXWY D500C2523V0 5" MIPI-DSI panel
 *
 * Copyright (C) Purism SPC 2019
 * Copyright (C) mlilabs GmbH 2026
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/media-bus-format.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include <video/display_timing.h>
#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#define DRV_NAME "panel-sitronix-st7703"

/* Manufacturer specific Commands send via DSI */
#define ST7703_CMD_ALL_PIXEL_OFF 0x22
#define ST7703_CMD_ALL_PIXEL_ON	 0x23
#define ST7703_CMD_SETAPID	 0xB1
#define ST7703_CMD_SETDISP	 0xB2
#define ST7703_CMD_SETRGBIF	 0xB3
#define ST7703_CMD_SETCYC	 0xB4
#define ST7703_CMD_SETBGP	 0xB5
#define ST7703_CMD_SETVCOM	 0xB6
#define ST7703_CMD_SETOTP	 0xB7
#define ST7703_CMD_SETPOWER_EXT	 0xB8
#define ST7703_CMD_SETEXTC	 0xB9
#define ST7703_CMD_SETMIPI	 0xBA
#define ST7703_CMD_SETVDC	 0xBC
#define ST7703_CMD_UNKNOWN_BF	 0xBF
#define ST7703_CMD_SETSCR	 0xC0
#define ST7703_CMD_SETPOWER	 0xC1
#define ST7703_CMD_SETPANEL	 0xCC
#define ST7703_CMD_UNKNOWN_C6	 0xC6
#define ST7703_CMD_SETGAMMA	 0xE0
#define ST7703_CMD_SETEQ	 0xE3
#define ST7703_CMD_SETGIP1	 0xE9
#define ST7703_CMD_SETGIP2	 0xEA
#define ST7703_CMD_SETIO	 0xC7
#define ST7703_CMD_SETCABC	 0xC8
#define ST7703_CMD_DGC_R	 0xCD
#define ST7703_CMD_DGC_G	 0xCE
#define ST7703_CMD_DGC_B	 0xCF
#define ST7703_CMD_UNKNOWN_EF	 0xEF

#define ST7703_GAMMA_LEN	34
#define ST7703_BGP_LEN		2
#define ST7703_VCOM_LEN		2
#define ST7703_DGC_LEN		33

struct st7703 {
	struct device *dev;
	struct drm_panel panel;
	struct gpio_desc *reset_gpio;
	struct regulator *vcc;
	struct regulator *iovcc;
	bool prepared;

	struct dentry *debugfs;
	const struct st7703_panel_desc *desc;

	/* Serialises runtime calibration updates from debugfs. */
	struct mutex calib_lock;

	u8 gamma[ST7703_GAMMA_LEN];
	u8 bgp[ST7703_BGP_LEN];
	u8 vcom[ST7703_VCOM_LEN];

	/* Digital Gamma Correction (per-channel 33-point 8-bit LUTs).
	 * Input control points are V0, V8, V16, ..., V240, V248, V255.
	 * The chip interpolates between entries and outputs 10-bit data
	 * to the dithering / source driver stage. The DGC_EN bit lives
	 * in parameter 1 of DGC_R (0xCD); we only store the 33 LUT bytes
	 * here and prepend the enable bit at write time.
	 *
	 * dgc_configured is set as soon as anything (panel descriptor, DT
	 * override, or a debugfs write) has supplied a non-default state.
	 * If it stays false the driver does not touch the DGC registers
	 * at all, preserving the chip's power-on defaults. */
	bool dgc_configured;
	bool dgc_enabled;
	u8 dgc_r[ST7703_DGC_LEN];
	u8 dgc_g[ST7703_DGC_LEN];
	u8 dgc_b[ST7703_DGC_LEN];
};

struct st7703_panel_desc {
	const struct drm_display_mode *mode;
	unsigned int lanes;
	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;
	int (*init_sequence)(struct st7703 *ctx);

	u8 gamma[ST7703_GAMMA_LEN];
	u8 bgp[ST7703_BGP_LEN];
	u8 vcom[ST7703_VCOM_LEN];

	/* Optional descriptor-level DGC defaults. If has_dgc is false,
	 * the driver falls back to an identity LUT and DGC stays off. */
	bool has_dgc;
	u8 dgc_r[ST7703_DGC_LEN];
	u8 dgc_g[ST7703_DGC_LEN];
	u8 dgc_b[ST7703_DGC_LEN];
};

static inline struct st7703 *panel_to_st7703(struct drm_panel *panel)
{
	return container_of(panel, struct st7703, panel);
}

#define ST7703_DSI_RETRIES	10
#define ST7703_DSI_RETRY_DELAY_MS	100

static bool st7703_dsi_err_is_transient(int err)
{
	return err == -ETIMEDOUT || err == -EIO || err == -EAGAIN;
}

static int st7703_dcs_write(struct st7703 *ctx, u8 cmd,
			    const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret = 0;
	int attempt;

	for (attempt = 0; attempt < ST7703_DSI_RETRIES; attempt++) {
		ret = mipi_dsi_dcs_write(dsi, cmd, data, len);
		if (ret >= 0)
			return ret;
		if (!st7703_dsi_err_is_transient(ret))
			return ret;
		dev_warn_ratelimited(ctx->dev,
			"DCS cmd 0x%02x (%zu bytes) transient err %d, retry %d/%d\n",
			cmd, len, ret, attempt + 1, ST7703_DSI_RETRIES);
		msleep(ST7703_DSI_RETRY_DELAY_MS);
	}

	return ret;
}

static int st7703_generic_write(struct st7703 *ctx,
				const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret = 0;
	int attempt;

	for (attempt = 0; attempt < ST7703_DSI_RETRIES; attempt++) {
		ret = mipi_dsi_generic_write(dsi, data, len);
		if (ret >= 0)
			return ret;
		if (!st7703_dsi_err_is_transient(ret))
			return ret;
		dev_warn_ratelimited(ctx->dev,
			"generic write (%zu bytes) transient err %d, retry %d/%d\n",
			len, ret, attempt + 1, ST7703_DSI_RETRIES);
		msleep(ST7703_DSI_RETRY_DELAY_MS);
	}

	return ret;
}

#define dsi_generic_write_seq(ctx, seq...) do {				\
		static const u8 d[] = { seq };				\
		int _r;							\
		_r = st7703_generic_write(ctx, d, ARRAY_SIZE(d));	\
		if (_r < 0)						\
			return _r;					\
	} while (0)

#define dsi_dcs_write_seq(ctx, cmd, seq...) do {			\
		static const u8 d[] = { seq };				\
		int _r;							\
		_r = st7703_dcs_write(ctx, cmd, d, ARRAY_SIZE(d));	\
		if (_r < 0)						\
			return _r;					\
	} while (0)

#define st7703_write_calib(ctx, cmd, field) do {			\
		int _r = st7703_dcs_write(ctx, cmd, (ctx)->field,	\
					  sizeof((ctx)->field));	\
		if (_r < 0)						\
			return _r;					\
	} while (0)


static int jh057n_init_sequence(struct st7703 *ctx)
{
	/*
	 * Init sequence was supplied by the panel vendor. Most of the commands
	 * resemble the ST7703 but the number of parameters often don't match
	 * so it's likely a clone.
	 */
	dsi_generic_write_seq(ctx, ST7703_CMD_SETEXTC,
			      0xF1, 0x12, 0x83);
	dsi_generic_write_seq(ctx, ST7703_CMD_SETRGBIF,
			      0x10, 0x10, 0x05, 0x05, 0x03, 0xFF, 0x00, 0x00,
			      0x00, 0x00);
	dsi_generic_write_seq(ctx, ST7703_CMD_SETSCR,
			      0x73, 0x73, 0x50, 0x50, 0x00, 0x00, 0x08, 0x70,
			      0x00);
	dsi_generic_write_seq(ctx, ST7703_CMD_SETVDC, 0x4E);
	dsi_generic_write_seq(ctx, ST7703_CMD_SETPANEL, 0x0B);
	dsi_generic_write_seq(ctx, ST7703_CMD_SETCYC, 0x80);
	dsi_generic_write_seq(ctx, ST7703_CMD_SETDISP, 0xF0, 0x12, 0x30);
	dsi_generic_write_seq(ctx, ST7703_CMD_SETEQ,
			      0x07, 0x07, 0x0B, 0x0B, 0x03, 0x0B, 0x00, 0x00,
			      0x00, 0x00, 0xFF, 0x00, 0xC0, 0x10);
	st7703_write_calib(ctx, ST7703_CMD_SETBGP, bgp);
	msleep(20);

	st7703_write_calib(ctx, ST7703_CMD_SETVCOM, vcom);
	dsi_generic_write_seq(ctx, ST7703_CMD_UNKNOWN_BF, 0x02, 0x11, 0x00);
	dsi_generic_write_seq(ctx, ST7703_CMD_SETGIP1,
			      0x82, 0x10, 0x06, 0x05, 0x9E, 0x0A, 0xA5, 0x12,
			      0x31, 0x23, 0x37, 0x83, 0x04, 0xBC, 0x27, 0x38,
			      0x0C, 0x00, 0x03, 0x00, 0x00, 0x00, 0x0C, 0x00,
			      0x03, 0x00, 0x00, 0x00, 0x75, 0x75, 0x31, 0x88,
			      0x88, 0x88, 0x88, 0x88, 0x88, 0x13, 0x88, 0x64,
			      0x64, 0x20, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
			      0x02, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	dsi_generic_write_seq(ctx, ST7703_CMD_SETGIP2,
			      0x02, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			      0x00, 0x00, 0x00, 0x00, 0x02, 0x46, 0x02, 0x88,
			      0x88, 0x88, 0x88, 0x88, 0x88, 0x64, 0x88, 0x13,
			      0x57, 0x13, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
			      0x75, 0x88, 0x23, 0x14, 0x00, 0x00, 0x02, 0x00,
			      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x0A,
			      0xA5, 0x00, 0x00, 0x00, 0x00);
	st7703_write_calib(ctx, ST7703_CMD_SETGAMMA, gamma);

	return 0;
}

static const struct drm_display_mode jh057n00900_mode = {
	.hdisplay    = 720,
	.hsync_start = 720 + 90,
	.hsync_end   = 720 + 90 + 20,
	.htotal	     = 720 + 90 + 20 + 20,
	.vdisplay    = 1440,
	.vsync_start = 1440 + 20,
	.vsync_end   = 1440 + 20 + 4,
	.vtotal	     = 1440 + 20 + 4 + 12,
	.clock	     = 75276,
	.flags	     = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
	.width_mm    = 65,
	.height_mm   = 130,
};

static const struct st7703_panel_desc jh057n00900_panel_desc = {
	.mode = &jh057n00900_mode,
	.lanes = 4,
	.mode_flags = MIPI_DSI_MODE_VIDEO |
		MIPI_DSI_MODE_VIDEO_BURST | MIPI_DSI_MODE_VIDEO_SYNC_PULSE,
	.format = MIPI_DSI_FMT_RGB888,
	.init_sequence = jh057n_init_sequence,
	.bgp   = { 0x08, 0x08 },
	.vcom  = { 0x3F, 0x3F },
	.gamma = { 0x00, 0x09, 0x0E, 0x29, 0x2D, 0x3C, 0x41, 0x37,
		   0x07, 0x0B, 0x0D, 0x10, 0x11, 0x0F, 0x10, 0x11,
		   0x18, 0x00, 0x09, 0x0E, 0x29, 0x2D, 0x3C, 0x41,
		   0x37, 0x07, 0x0B, 0x0D, 0x10, 0x11, 0x0F, 0x10,
		   0x11, 0x18 },
};


static int xbd599_init_sequence(struct st7703 *ctx)
{
	/*
	 * Init sequence was supplied by the panel vendor.
	 */

	/* Magic sequence to unlock user commands below. */
	dsi_dcs_write_seq(ctx, ST7703_CMD_SETEXTC, 0xF1, 0x12, 0x83);

	dsi_dcs_write_seq(ctx, ST7703_CMD_SETMIPI,
			  0x33, /* VC_main = 0, Lane_Number = 3 (4 lanes) */
			  0x81, /* DSI_LDO_SEL = 1.7V, RTERM = 90 Ohm */
			  0x05, /* IHSRX = x6 (Low High Speed driving ability) */
			  0xF9, /* TX_CLK_SEL = fDSICLK/16 */
			  0x0E, /* HFP_OSC (min. HFP number in DSI mode) */
			  0x0E, /* HBP_OSC (min. HBP number in DSI mode) */
			  /* The rest is undocumented in ST7703 datasheet */
			  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			  0x44, 0x25, 0x00, 0x91, 0x0a, 0x00, 0x00, 0x02,
			  0x4F, 0x11, 0x00, 0x00, 0x37);

	dsi_dcs_write_seq(ctx, ST7703_CMD_SETPOWER_EXT,
			  0x25, /* PCCS = 2, ECP_DC_DIV = 1/4 HSYNC */
			  0x22, /* DT = 15ms XDK_ECP = x2 */
			  0x20, /* PFM_DC_DIV = /1 */
			  0x03  /* ECP_SYNC_EN = 1, VGX_SYNC_EN = 1 */);

	/* RGB I/F porch timing */
	dsi_dcs_write_seq(ctx, ST7703_CMD_SETRGBIF,
			  0x10, /* VBP_RGB_GEN */
			  0x10, /* VFP_RGB_GEN */
			  0x05, /* DE_BP_RGB_GEN */
			  0x05, /* DE_FP_RGB_GEN */
			  /* The rest is undocumented in ST7703 datasheet */
			  0x03, 0xFF,
			  0x00, 0x00,
			  0x00, 0x00);

	/* Source driving settings. */
	dsi_dcs_write_seq(ctx, ST7703_CMD_SETSCR,
			  0x73, /* N_POPON */
			  0x73, /* N_NOPON */
			  0x50, /* I_POPON */
			  0x50, /* I_NOPON */
			  0x00, /* SCR[31,24] */
			  0xC0, /* SCR[23,16] */
			  0x08, /* SCR[15,8] */
			  0x70, /* SCR[7,0] */
			  0x00  /* Undocumented */);

	/* NVDDD_SEL = -1.8V, VDDD_SEL = out of range (possibly 1.9V?) */
	dsi_dcs_write_seq(ctx, ST7703_CMD_SETVDC, 0x4E);

	/*
	 * SS_PANEL = 1 (reverse scan), GS_PANEL = 0 (normal scan)
	 * REV_PANEL = 1 (normally black panel), BGR_PANEL = 1 (BGR)
	 */
	dsi_dcs_write_seq(ctx, ST7703_CMD_SETPANEL, 0x0B);

	/* Zig-Zag Type C column inversion. */
	dsi_dcs_write_seq(ctx, ST7703_CMD_SETCYC, 0x80);

	/* Set display resolution. */
	dsi_dcs_write_seq(ctx, ST7703_CMD_SETDISP,
			  0xF0, /* NL = 240 */
			  0x12, /* RES_V_LSB = 0, BLK_CON = VSSD,
				 * RESO_SEL = 720RGB
				 */
			  0xF0  /* WHITE_GND_EN = 1 (GND),
				 * WHITE_FRAME_SEL = 7 frames,
				 * ISC = 0 frames
				 */);

	dsi_dcs_write_seq(ctx, ST7703_CMD_SETEQ,
			  0x00, /* PNOEQ */
			  0x00, /* NNOEQ */
			  0x0B, /* PEQGND */
			  0x0B, /* NEQGND */
			  0x10, /* PEQVCI */
			  0x10, /* NEQVCI */
			  0x00, /* PEQVCI1 */
			  0x00, /* NEQVCI1 */
			  0x00, /* reserved */
			  0x00, /* reserved */
			  0xFF, /* reserved */
			  0x00, /* reserved */
			  0xC0, /* ESD_DET_DATA_WHITE = 1, ESD_WHITE_EN = 1 */
			  0x10  /* SLPIN_OPTION = 1 (no need vsync after sleep-in)
				 * VEDIO_NO_CHECK_EN = 0
				 * ESD_WHITE_GND_EN = 0
				 * ESD_DET_TIME_SEL = 0 frames
				 */);

	/* Undocumented command. */
	dsi_dcs_write_seq(ctx, ST7703_CMD_UNKNOWN_C6, 0x01, 0x00, 0xFF, 0xFF, 0x00);

	dsi_dcs_write_seq(ctx, ST7703_CMD_SETPOWER,
			  0x74, /* VBTHS, VBTLS: VGH = 17V, VBL = -11V */
			  0x00, /* FBOFF_VGH = 0, FBOFF_VGL = 0 */
			  0x32, /* VRP  */
			  0x32, /* VRN */
			  0x77, /* reserved */
			  0xF1, /* APS = 1 (small),
				 * VGL_DET_EN = 1, VGH_DET_EN = 1,
				 * VGL_TURBO = 1, VGH_TURBO = 1
				 */
			  0xFF, /* VGH1_L_DIV, VGL1_L_DIV (1.5MHz) */
			  0xFF, /* VGH1_R_DIV, VGL1_R_DIV (1.5MHz) */
			  0xCC, /* VGH2_L_DIV, VGL2_L_DIV (2.6MHz) */
			  0xCC, /* VGH2_R_DIV, VGL2_R_DIV (2.6MHz) */
			  0x77, /* VGH3_L_DIV, VGL3_L_DIV (4.5MHz) */
			  0x77  /* VGH3_R_DIV, VGL3_R_DIV (4.5MHz) */);

	/* Reference voltage. */
	st7703_write_calib(ctx, ST7703_CMD_SETBGP, bgp);
	msleep(20);

	st7703_write_calib(ctx, ST7703_CMD_SETVCOM, vcom);

	/* Undocumented command. */
	dsi_dcs_write_seq(ctx, ST7703_CMD_UNKNOWN_BF, 0x02, 0x11, 0x00);

	/* This command is to set forward GIP timing. */
	dsi_dcs_write_seq(ctx, ST7703_CMD_SETGIP1,
			  0x82, 0x10, 0x06, 0x05, 0xA2, 0x0A, 0xA5, 0x12,
			  0x31, 0x23, 0x37, 0x83, 0x04, 0xBC, 0x27, 0x38,
			  0x0C, 0x00, 0x03, 0x00, 0x00, 0x00, 0x0C, 0x00,
			  0x03, 0x00, 0x00, 0x00, 0x75, 0x75, 0x31, 0x88,
			  0x88, 0x88, 0x88, 0x88, 0x88, 0x13, 0x88, 0x64,
			  0x64, 0x20, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
			  0x02, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);

	/* This command is to set backward GIP timing. */
	dsi_dcs_write_seq(ctx, ST7703_CMD_SETGIP2,
			  0x02, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x02, 0x46, 0x02, 0x88,
			  0x88, 0x88, 0x88, 0x88, 0x88, 0x64, 0x88, 0x13,
			  0x57, 0x13, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
			  0x75, 0x88, 0x23, 0x14, 0x00, 0x00, 0x02, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0A,
			  0xA5, 0x00, 0x00, 0x00, 0x00);

	/* Adjust the gamma characteristics of the panel. */
	st7703_write_calib(ctx, ST7703_CMD_SETGAMMA, gamma);

	return 0;
}

static const struct drm_display_mode xbd599_mode = {
	.hdisplay    = 720,
	.hsync_start = 720 + 40,
	.hsync_end   = 720 + 40 + 40,
	.htotal	     = 720 + 40 + 40 + 40,
	.vdisplay    = 1440,
	.vsync_start = 1440 + 18,
	.vsync_end   = 1440 + 18 + 10,
	.vtotal	     = 1440 + 18 + 10 + 17,
	.clock	     = 69000,
	.flags	     = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
	.width_mm    = 68,
	.height_mm   = 136,
};

static const struct st7703_panel_desc xbd599_desc = {
	.mode = &xbd599_mode,
	.lanes = 4,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE,
	.format = MIPI_DSI_FMT_RGB888,
	.init_sequence = xbd599_init_sequence,
	.bgp   = { 0x07, 0x07 }, /* VREF/NVREF = 4.2V */
	.vcom  = { 0x2C, 0x2C }, /* VCOMDC = -0.67V */
	.gamma = { 0x00, 0x09, 0x0D, 0x23, 0x27, 0x3C, 0x41, 0x35,
		   0x07, 0x0D, 0x0E, 0x12, 0x13, 0x10, 0x12, 0x12,
		   0x18, 0x00, 0x09, 0x0D, 0x23, 0x27, 0x3C, 0x41,
		   0x35, 0x07, 0x0D, 0x0E, 0x12, 0x13, 0x10, 0x12,
		   0x12, 0x18 },
};

static int d500c2523v0_init_sequence(struct st7703 *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);

	/*
	 * Init sequence was supplied by the panel vendor for the
	 * DXWY D500C2523V0 5.0" 720x1280 panel with ST7703 controller.
	 */
	dsi_dcs_write_seq(ctx, ST7703_CMD_SETEXTC, 0xF1, 0x12, 0x83);
	dsi_dcs_write_seq(ctx, ST7703_CMD_SETAPID,
			  0x00, 0x00, 0x00, 0xDA, 0x80);
	dsi_dcs_write_seq(ctx, ST7703_CMD_SETDISP, 0xC8, 0x02, 0x70);
	dsi_dcs_write_seq(ctx, ST7703_CMD_SETRGBIF,
			  0x10, 0x10, 0x28, 0x28, 0x03, 0xFF, 0x00, 0x00,
			  0x00, 0x00);
	dsi_dcs_write_seq(ctx, ST7703_CMD_SETCYC, 0x80);
	st7703_write_calib(ctx, ST7703_CMD_SETBGP, bgp);
	st7703_write_calib(ctx, ST7703_CMD_SETVCOM, vcom);
	dsi_dcs_write_seq(ctx, ST7703_CMD_SETPOWER_EXT,
			  0x26, 0x22, 0xF0, 0x13);

	/* Configure DSI lanes: 0x31 = 2 lanes, 0x33 = 4 lanes. */
	{
		u8 setmipi[] = {
			0x31, 0x81, 0x05, 0xF9, 0x0E, 0x0E, 0x20, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x25,
			0x00, 0x90, 0x0A, 0x00, 0x00, 0x01, 0x4F, 0x01,
			0x00, 0x00, 0x37
		};
		int ret;

		if (dsi->lanes == 4)
			setmipi[0] = 0x33;

		ret = st7703_dcs_write(ctx, ST7703_CMD_SETMIPI,
				       setmipi, sizeof(setmipi));
		if (ret < 0)
			return ret;
	}

	dsi_dcs_write_seq(ctx, ST7703_CMD_SETVDC, 0x47);
	dsi_dcs_write_seq(ctx, ST7703_CMD_UNKNOWN_BF, 0x02, 0x11, 0x00);
	dsi_dcs_write_seq(ctx, ST7703_CMD_SETSCR,
			  0x73, 0x73, 0x50, 0x50, 0x00, 0x00, 0x12, 0x70,
			  0x00);
	dsi_dcs_write_seq(ctx, ST7703_CMD_SETPOWER,
			  0x25, 0x00, 0x32, 0x32, 0x77, 0xE4, 0xFF, 0xFF,
			  0xCC, 0xCC, 0x77, 0x77);
	dsi_dcs_write_seq(ctx, ST7703_CMD_UNKNOWN_C6,
			  0x82, 0x00, 0xBF, 0xFF, 0x00, 0xFF);
	dsi_dcs_write_seq(ctx, ST7703_CMD_SETIO,
			  0xB8, 0x00, 0x0A, 0x10, 0x01, 0x09);
	dsi_dcs_write_seq(ctx, ST7703_CMD_SETCABC,
			  0x10, 0x40, 0x1E, 0x02);
	dsi_dcs_write_seq(ctx, ST7703_CMD_SETPANEL, 0x0B);
	st7703_write_calib(ctx, ST7703_CMD_SETGAMMA, gamma);
	dsi_dcs_write_seq(ctx, ST7703_CMD_SETEQ,
			  0x07, 0x07, 0x0B, 0x0B, 0x0B, 0x0B, 0x00, 0x00,
			  0x00, 0x00, 0xFF, 0x80, 0xC0, 0x10);
	dsi_dcs_write_seq(ctx, ST7703_CMD_SETGIP1,
			  0xC8, 0x10, 0x0C, 0x00, 0x00, 0x80, 0x81, 0x12,
			  0x31, 0x23, 0x4F, 0x8A, 0x80, 0x38, 0x47, 0x18,
			  0x30, 0x00, 0x01, 0x00, 0x00, 0x00, 0x30, 0x00,
			  0x01, 0x00, 0x00, 0x00, 0x98, 0x02, 0x46, 0x88,
			  0x88, 0x88, 0x88, 0x88, 0x88, 0xFF, 0x02, 0x98,
			  0x13, 0x57, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
			  0xFF, 0x13, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	dsi_dcs_write_seq(ctx, ST7703_CMD_SETGIP2,
			  0x00, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x9F, 0x75, 0x31, 0x88,
			  0x88, 0x88, 0x88, 0x88, 0x88, 0xF8, 0x31, 0x9F,
			  0x64, 0x20, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
			  0xF8, 0x20, 0x23, 0x00, 0x00, 0x01, 0x44, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x05, 0x0D, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x70, 0x80, 0x81, 0x00);
	dsi_dcs_write_seq(ctx, ST7703_CMD_UNKNOWN_EF, 0xFF, 0xFF, 0x01);

	return 0;
}

static const struct drm_display_mode d500c2523v0_mode = {
	.hdisplay    = 720,
	.hsync_start = 720 + 48,
	.hsync_end   = 720 + 48 + 12,
	.htotal      = 720 + 48 + 12 + 43,
	.vdisplay    = 1280,
	.vsync_start = 1280 + 24,
	.vsync_end   = 1280 + 24 + 12,
	.vtotal      = 1280 + 24 + 12 + 28,
	.clock       = 60800,
	.flags       = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
	.width_mm    = 62,
	.height_mm   = 110,
};

static const struct st7703_panel_desc d500c2523v0_desc = {
	.mode = &d500c2523v0_mode,
	.lanes = 2,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
		      MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS,
	.format = MIPI_DSI_FMT_RGB888,
	.init_sequence = d500c2523v0_init_sequence,
	.bgp   = { 0x0C, 0x0C },
	.vcom  = { 0x7D, 0x7D },
	.gamma = { 0x08, 0x09, 0x09, 0x30, 0x30, 0x3F, 0x2D, 0x28,
		   0x05, 0x09, 0x0C, 0x10, 0x12, 0x10, 0x12, 0x0F,
		   0x18, 0x00, 0x09, 0x09, 0x30, 0x30, 0x3F, 0x2D,
		   0x28, 0x05, 0x09, 0x0C, 0x10, 0x12, 0x10, 0x12,
		   0x0F, 0x18 },
};

static int st7703_enable(struct drm_panel *panel)
{
	struct st7703 *ctx = panel_to_st7703(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret) {
		dev_err(ctx->dev, "Failed to set display on: %d\n", ret);
		return ret;
	}

	/* Vendor specifies 20ms delay after display_on */
	msleep(20);

	dev_dbg(ctx->dev, "Panel init sequence done\n");

	return 0;
}

static int st7703_disable(struct drm_panel *panel)
{
	struct st7703 *ctx = panel_to_st7703(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0)
		dev_err(ctx->dev, "Failed to turn off the display: %d\n", ret);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0)
		dev_err(ctx->dev, "Failed to enter sleep mode: %d\n", ret);

	return 0;
}

static int st7703_unprepare(struct drm_panel *panel)
{
	struct st7703 *ctx = panel_to_st7703(panel);

	if (!ctx->prepared)
		return 0;

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_disable(ctx->iovcc);
	regulator_disable(ctx->vcc);
	ctx->prepared = false;

	return 0;
}

static int st7703_apply_dgc_locked(struct st7703 *ctx);

static int st7703_prepare(struct drm_panel *panel)
{
	struct st7703 *ctx = panel_to_st7703(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	dev_dbg(ctx->dev, "Resetting the panel\n");
	ret = regulator_enable(ctx->vcc);
	if (ret < 0) {
		dev_err(ctx->dev, "Failed to enable vcc supply: %d\n", ret);
		return ret;
	}

	ret = regulator_enable(ctx->iovcc);
	if (ret < 0) {
		dev_err(ctx->dev, "Failed to enable iovcc supply: %d\n", ret);
		goto disable_vcc;
	}

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	msleep(20);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(200);

	/* Read display ID to verify DSI communication */
	{
		struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
		u8 id = 0;

		ret = mipi_dsi_dcs_read(dsi, 0x04, &id, 1);
		if (ret >= 0)
			dev_info(ctx->dev, "Display ID: 0x%02x\n", id);
	}

	ret = ctx->desc->init_sequence(ctx);
	if (ret < 0) {
		dev_err(ctx->dev, "Panel init sequence failed: %d\n", ret);
		goto disable_iovcc;
	}

	/*
	 * Apply Digital Gamma Correction only if it has actually been
	 * configured. The chip resets DGC state on every power cycle, so
	 * we must re-upload after each init sequence; but if nothing has
	 * supplied a LUT (no descriptor default, no DT override, no
	 * debugfs write), we leave the DGC registers at their power-on
	 * defaults to keep the previous behavior unchanged.
	 */
	mutex_lock(&ctx->calib_lock);
	if (ctx->dgc_configured)
		ret = st7703_apply_dgc_locked(ctx);
	else
		ret = 0;
	mutex_unlock(&ctx->calib_lock);
	if (ret < 0) {
		dev_err(ctx->dev, "DGC apply failed: %d\n", ret);
		goto disable_iovcc;
	}

	msleep(20);

	ret = mipi_dsi_dcs_exit_sleep_mode(to_mipi_dsi_device(ctx->dev));
	if (ret < 0) {
		dev_err(ctx->dev, "Failed to exit sleep mode: %d\n", ret);
		goto disable_iovcc;
	}

	/* Panel is operational 120 msec after reset */
	msleep(120);

	ctx->prepared = true;

	return 0;

disable_iovcc:
	regulator_disable(ctx->iovcc);
disable_vcc:
	regulator_disable(ctx->vcc);
	return ret;
}

static const u32 mantix_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
};

static int st7703_get_modes(struct drm_panel *panel,
			    struct drm_connector *connector)
{
	struct st7703 *ctx = panel_to_st7703(panel);
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, ctx->desc->mode);
	if (!mode) {
		dev_err(ctx->dev, "Failed to add mode %ux%u@%u\n",
			ctx->desc->mode->hdisplay, ctx->desc->mode->vdisplay,
			drm_mode_vrefresh(ctx->desc->mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	drm_display_info_set_bus_formats(&connector->display_info,
					 mantix_bus_formats,
					 ARRAY_SIZE(mantix_bus_formats));

	return 1;
}

static const struct drm_panel_funcs st7703_drm_funcs = {
	.disable   = st7703_disable,
	.unprepare = st7703_unprepare,
	.prepare   = st7703_prepare,
	.enable	   = st7703_enable,
	.get_modes = st7703_get_modes,
};

static int allpixelson_set(void *data, u64 val)
{
	struct st7703 *ctx = data;

	dev_dbg(ctx->dev, "Setting all pixels on\n");
	dsi_generic_write_seq(ctx, ST7703_CMD_ALL_PIXEL_ON);
	msleep(val * 1000);
	/* Reset the panel to get video back */
	drm_panel_disable(&ctx->panel);
	drm_panel_unprepare(&ctx->panel);
	drm_panel_prepare(&ctx->panel);
	drm_panel_enable(&ctx->panel);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(allpixelson_fops, NULL,
			allpixelson_set, "%llu\n");

static int st7703_apply_calibration_locked(struct st7703 *ctx)
{
	static const u8 setextc[] = { 0xF1, 0x12, 0x83 };
	int ret;

	ret = st7703_dcs_write(ctx, ST7703_CMD_SETEXTC,
			       setextc, sizeof(setextc));
	if (ret < 0) {
		dev_err(ctx->dev, "SETEXTC failed: %d\n", ret);
		return ret;
	}

	ret = st7703_dcs_write(ctx, ST7703_CMD_SETBGP,
			       ctx->bgp, sizeof(ctx->bgp));
	if (ret < 0) {
		dev_err(ctx->dev, "SETBGP failed: %d\n", ret);
		return ret;
	}
	msleep(20);

	ret = st7703_dcs_write(ctx, ST7703_CMD_SETVCOM,
			       ctx->vcom, sizeof(ctx->vcom));
	if (ret < 0) {
		dev_err(ctx->dev, "SETVCOM failed: %d\n", ret);
		return ret;
	}
	msleep(20);

	ret = st7703_dcs_write(ctx, ST7703_CMD_SETGAMMA,
			       ctx->gamma, sizeof(ctx->gamma));
	if (ret < 0) {
		dev_err(ctx->dev, "SETGAMMA failed: %d\n", ret);
		return ret;
	}

	return 0;
}

/*
 * Upload the current per-channel DGC LUTs and the DGC_EN bit.
 *
 * DGC_R (0xCD) takes 34 parameters: parameter 1 carries DGC_EN in bit 0
 * (other bits reserved), parameters 2..34 carry the 33-entry red LUT.
 * DGC_G (0xCE) and DGC_B (0xCF) each take a flat 33-entry LUT.
 *
 * This must be called with calib_lock held.
 */
static int st7703_apply_dgc_locked(struct st7703 *ctx)
{
	u8 buf[1 + ST7703_DGC_LEN];
	int ret;

	buf[0] = ctx->dgc_enabled ? 0x01 : 0x00;
	memcpy(buf + 1, ctx->dgc_r, ST7703_DGC_LEN);
	ret = st7703_dcs_write(ctx, ST7703_CMD_DGC_R, buf, sizeof(buf));
	if (ret < 0) {
		dev_err(ctx->dev, "DGC_R write failed: %d\n", ret);
		return ret;
	}

	ret = st7703_dcs_write(ctx, ST7703_CMD_DGC_G,
			       ctx->dgc_g, ST7703_DGC_LEN);
	if (ret < 0) {
		dev_err(ctx->dev, "DGC_G write failed: %d\n", ret);
		return ret;
	}

	ret = st7703_dcs_write(ctx, ST7703_CMD_DGC_B,
			       ctx->dgc_b, ST7703_DGC_LEN);
	if (ret < 0) {
		dev_err(ctx->dev, "DGC_B write failed: %d\n", ret);
		return ret;
	}

	return 0;
}

/*
 * Stage a new calibration buffer from userspace into the corresponding
 * field of struct st7703 and re-apply the full calibration sequence.
 */
static ssize_t st7703_calib_write(struct st7703 *ctx, void *field,
				  size_t expected,
				  const char __user *ubuf, size_t count)
{
	u8 staged[ST7703_GAMMA_LEN];
	int ret;

	if (expected > sizeof(staged))
		return -EINVAL;
	if (count != expected)
		return -EINVAL;

	if (copy_from_user(staged, ubuf, expected))
		return -EFAULT;

	mutex_lock(&ctx->calib_lock);
	memcpy(field, staged, expected);
	ret = st7703_apply_calibration_locked(ctx);
	mutex_unlock(&ctx->calib_lock);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t gamma_write(struct file *f, const char __user *ubuf,
			   size_t count, loff_t *ppos)
{
	struct st7703 *ctx = f->private_data;

	return st7703_calib_write(ctx, ctx->gamma, sizeof(ctx->gamma),
				  ubuf, count);
}

static ssize_t bgp_write(struct file *f, const char __user *ubuf,
			 size_t count, loff_t *ppos)
{
	struct st7703 *ctx = f->private_data;

	return st7703_calib_write(ctx, ctx->bgp, sizeof(ctx->bgp),
				  ubuf, count);
}

static ssize_t vcom_write(struct file *f, const char __user *ubuf,
			  size_t count, loff_t *ppos)
{
	struct st7703 *ctx = f->private_data;

	return st7703_calib_write(ctx, ctx->vcom, sizeof(ctx->vcom),
				  ubuf, count);
}

/*
 * Stage a new DGC LUT for one channel and re-upload all three LUTs plus
 * the enable bit. Each write must be exactly ST7703_DGC_LEN (33) bytes.
 */
static ssize_t st7703_dgc_write(struct st7703 *ctx, u8 *field,
				const char __user *ubuf, size_t count)
{
	u8 staged[ST7703_DGC_LEN];
	int ret;

	if (count != ST7703_DGC_LEN)
		return -EINVAL;

	if (copy_from_user(staged, ubuf, ST7703_DGC_LEN))
		return -EFAULT;

	mutex_lock(&ctx->calib_lock);
	memcpy(field, staged, ST7703_DGC_LEN);
	ctx->dgc_configured = true;
	ret = st7703_apply_dgc_locked(ctx);
	mutex_unlock(&ctx->calib_lock);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t dgc_r_write(struct file *f, const char __user *ubuf,
			   size_t count, loff_t *ppos)
{
	struct st7703 *ctx = f->private_data;

	return st7703_dgc_write(ctx, ctx->dgc_r, ubuf, count);
}

static ssize_t dgc_g_write(struct file *f, const char __user *ubuf,
			   size_t count, loff_t *ppos)
{
	struct st7703 *ctx = f->private_data;

	return st7703_dgc_write(ctx, ctx->dgc_g, ubuf, count);
}

static ssize_t dgc_b_write(struct file *f, const char __user *ubuf,
			   size_t count, loff_t *ppos)
{
	struct st7703 *ctx = f->private_data;

	return st7703_dgc_write(ctx, ctx->dgc_b, ubuf, count);
}

static int dgc_enable_get(void *data, u64 *val)
{
	struct st7703 *ctx = data;

	*val = ctx->dgc_enabled ? 1 : 0;
	return 0;
}

static int dgc_enable_set(void *data, u64 val)
{
	struct st7703 *ctx = data;
	int ret;

	mutex_lock(&ctx->calib_lock);
	ctx->dgc_enabled = !!val;
	ctx->dgc_configured = true;
	ret = st7703_apply_dgc_locked(ctx);
	mutex_unlock(&ctx->calib_lock);

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(dgc_enable_fops, dgc_enable_get, dgc_enable_set,
			"%llu\n");

static const struct file_operations gamma_fops = {
	.owner = THIS_MODULE,
	.open  = simple_open,
	.write = gamma_write,
	.llseek = noop_llseek,
};

static const struct file_operations bgp_fops = {
	.owner = THIS_MODULE,
	.open  = simple_open,
	.write = bgp_write,
	.llseek = noop_llseek,
};

static const struct file_operations vcom_fops = {
	.owner = THIS_MODULE,
	.open  = simple_open,
	.write = vcom_write,
	.llseek = noop_llseek,
};

static const struct file_operations dgc_r_fops = {
	.owner = THIS_MODULE,
	.open  = simple_open,
	.write = dgc_r_write,
	.llseek = noop_llseek,
};

static const struct file_operations dgc_g_fops = {
	.owner = THIS_MODULE,
	.open  = simple_open,
	.write = dgc_g_write,
	.llseek = noop_llseek,
};

static const struct file_operations dgc_b_fops = {
	.owner = THIS_MODULE,
	.open  = simple_open,
	.write = dgc_b_write,
	.llseek = noop_llseek,
};

static void st7703_debugfs_init(struct st7703 *ctx)
{
	ctx->debugfs = debugfs_create_dir(DRV_NAME, NULL);

	debugfs_create_file("allpixelson", 0600, ctx->debugfs, ctx,
			    &allpixelson_fops);
	debugfs_create_file("gamma", 0200, ctx->debugfs, ctx, &gamma_fops);
	debugfs_create_file("bgp",   0200, ctx->debugfs, ctx, &bgp_fops);
	debugfs_create_file("vcom",  0200, ctx->debugfs, ctx, &vcom_fops);
	debugfs_create_file("dgc_r", 0200, ctx->debugfs, ctx, &dgc_r_fops);
	debugfs_create_file("dgc_g", 0200, ctx->debugfs, ctx, &dgc_g_fops);
	debugfs_create_file("dgc_b", 0200, ctx->debugfs, ctx, &dgc_b_fops);
	debugfs_create_file("dgc_enable", 0600, ctx->debugfs, ctx,
			    &dgc_enable_fops);
}

static void st7703_debugfs_remove(struct st7703 *ctx)
{
	debugfs_remove_recursive(ctx->debugfs);
	ctx->debugfs = NULL;
}

static int st7703_read_u8_override(struct device *dev, const char *name,
				   u8 *buf, size_t expected)
{
	int ret;

	if (!of_find_property(dev->of_node, name, NULL))
		return 0;

	ret = of_property_read_variable_u8_array(dev->of_node, name, buf,
						 expected, expected);
	if (ret < 0) {
		dev_err(dev, "DT property '%s' must be exactly %zu bytes (%d)\n",
			name, expected, ret);
		return ret;
	}

	dev_info(dev, "Using DT override for %s (%zu bytes)\n",
		 name, expected);
	return 0;
}

static int st7703_load_calibration(struct st7703 *ctx)
{
	int ret;

	/* Start from the panel descriptor's defaults. */
	memcpy(ctx->gamma, ctx->desc->gamma, sizeof(ctx->gamma));
	memcpy(ctx->bgp,   ctx->desc->bgp,   sizeof(ctx->bgp));
	memcpy(ctx->vcom,  ctx->desc->vcom,  sizeof(ctx->vcom));

	/* DGC defaults: descriptor table if provided. Otherwise leave the
	 * LUTs zero-initialised; they will not be uploaded unless
	 * dgc_configured becomes true via DT or debugfs. */
	ctx->dgc_enabled = false;
	ctx->dgc_configured = false;
	if (ctx->desc->has_dgc) {
		memcpy(ctx->dgc_r, ctx->desc->dgc_r, ST7703_DGC_LEN);
		memcpy(ctx->dgc_g, ctx->desc->dgc_g, ST7703_DGC_LEN);
		memcpy(ctx->dgc_b, ctx->desc->dgc_b, ST7703_DGC_LEN);
		ctx->dgc_configured = true;
	}

	/* Apply DT overrides on top, if any. */
	ret = st7703_read_u8_override(ctx->dev, "panel,gamma",
				      ctx->gamma, sizeof(ctx->gamma));
	if (ret < 0)
		return ret;

	ret = st7703_read_u8_override(ctx->dev, "panel,bgp",
				      ctx->bgp, sizeof(ctx->bgp));
	if (ret < 0)
		return ret;

	ret = st7703_read_u8_override(ctx->dev, "panel,vcom",
				      ctx->vcom, sizeof(ctx->vcom));
	if (ret < 0)
		return ret;

	if (of_find_property(ctx->dev->of_node, "panel,dgc-r", NULL)) {
		ret = st7703_read_u8_override(ctx->dev, "panel,dgc-r",
					      ctx->dgc_r, ST7703_DGC_LEN);
		if (ret < 0)
			return ret;
		ctx->dgc_configured = true;
	}

	if (of_find_property(ctx->dev->of_node, "panel,dgc-g", NULL)) {
		ret = st7703_read_u8_override(ctx->dev, "panel,dgc-g",
					      ctx->dgc_g, ST7703_DGC_LEN);
		if (ret < 0)
			return ret;
		ctx->dgc_configured = true;
	}

	if (of_find_property(ctx->dev->of_node, "panel,dgc-b", NULL)) {
		ret = st7703_read_u8_override(ctx->dev, "panel,dgc-b",
					      ctx->dgc_b, ST7703_DGC_LEN);
		if (ret < 0)
			return ret;
		ctx->dgc_configured = true;
	}

	if (of_property_read_bool(ctx->dev->of_node, "panel,dgc-enable")) {
		ctx->dgc_enabled = true;
		ctx->dgc_configured = true;
	}

	return 0;
}

static int st7703_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct st7703 *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio), "Failed to get reset gpio\n");

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	ctx->desc = of_device_get_match_data(dev);
	mutex_init(&ctx->calib_lock);

	dsi->mode_flags = ctx->desc->mode_flags;
	dsi->format = ctx->desc->format;
	dsi->lanes = ctx->desc->lanes;

	ctx->vcc = devm_regulator_get(dev, "vcc");
	if (IS_ERR(ctx->vcc))
		return dev_err_probe(dev, PTR_ERR(ctx->vcc), "Failed to request vcc regulator\n");

	ctx->iovcc = devm_regulator_get(dev, "iovcc");
	if (IS_ERR(ctx->iovcc))
		return dev_err_probe(dev, PTR_ERR(ctx->iovcc),
				     "Failed to request iovcc regulator\n");

	ret = st7703_load_calibration(ctx);
	if (ret < 0)
		return ret;

	drm_panel_init(&ctx->panel, dev, &st7703_drm_funcs,
		       DRM_MODE_CONNECTOR_DSI);

#ifdef RPI_KERNEL
	/* Raspberry Pi downstream kernels require this to be set for the
	   DSI interface to be initialized. */
	ctx->panel.prepare_upstream_first = true;
#endif

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return ret;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "mipi_dsi_attach failed (%d). Is host ready?\n", ret);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	dev_info(dev, "%ux%u@%u %ubpp dsi %udl - ready\n",
		 ctx->desc->mode->hdisplay, ctx->desc->mode->vdisplay,
		 drm_mode_vrefresh(ctx->desc->mode),
		 mipi_dsi_pixel_format_to_bpp(dsi->format), dsi->lanes);

	st7703_debugfs_init(ctx);
	return 0;
}

static void st7703_shutdown(struct mipi_dsi_device *dsi)
{
	struct st7703 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = drm_panel_unprepare(&ctx->panel);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to unprepare panel: %d\n", ret);

	ret = drm_panel_disable(&ctx->panel);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to disable panel: %d\n", ret);
}

static void st7703_remove(struct mipi_dsi_device *dsi)
{
	struct st7703 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	st7703_shutdown(dsi);

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);

	st7703_debugfs_remove(ctx);
}

static const struct of_device_id st7703_of_match[] = {
	{ .compatible = "dxwy,d500c2523v0", .data = &d500c2523v0_desc },
	{ .compatible = "rocktech,jh057n00900", .data = &jh057n00900_panel_desc },
	{ .compatible = "xingbangda,xbd599", .data = &xbd599_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, st7703_of_match);

static struct mipi_dsi_driver st7703_driver = {
	.probe	= st7703_probe,
	.remove = st7703_remove,
	.shutdown = st7703_shutdown,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = st7703_of_match,
	},
};
module_mipi_dsi_driver(st7703_driver);

MODULE_AUTHOR("Guido Günther <agx@sigxcpu.org>");
MODULE_DESCRIPTION("DRM driver for Sitronix ST7703 based MIPI DSI panels");
MODULE_LICENSE("GPL v2");
