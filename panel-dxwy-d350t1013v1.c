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

struct d350t1013v1_panel_desc {
	const struct drm_display_mode *mode;
	unsigned int lanes;
	unsigned long flags;
	enum mipi_dsi_pixel_format format;
	const char *const *supply_names;
	unsigned int num_supplies;
};

struct d350t1013v1 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	const struct d350t1013v1_panel_desc *desc;
	struct regulator_bulk_data *supplies;
	struct gpio_desc *reset;
};

static inline struct d350t1013v1 *panel_to_d350t1013v1(struct drm_panel *panel)
{
	return container_of(panel, struct d350t1013v1, panel);
}

static inline int d350t1013v1_dsi_write(struct d350t1013v1 *d350t1013v1, const void *seq,
				   size_t len)
{
	return mipi_dsi_dcs_write_buffer(d350t1013v1->dsi, seq, len);
}

#define D350T1013V1_DSI(d350t1013v1, seq...) \
	{ \
		const u8 d[] = { seq };	\
		d350t1013v1_dsi_write(d350t1013v1, d, ARRAY_SIZE(d)); \
		msleep(10); \
	}

static void d350t1013v1_init_sequence(struct d350t1013v1 *d350t1013v1)
{
	/* Init sequence provided by manufacturer. */

	D350T1013V1_DSI(d350t1013v1, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x13);
	D350T1013V1_DSI(d350t1013v1, 0xEF, 0x08);
	D350T1013V1_DSI(d350t1013v1, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x10);
	D350T1013V1_DSI(d350t1013v1, 0xC0, 0x63, 0x00);
	D350T1013V1_DSI(d350t1013v1, 0xC1, 0x10, 0x02);
	D350T1013V1_DSI(d350t1013v1, 0xC2, 0x31, 0x02);
	D350T1013V1_DSI(d350t1013v1, 0xCC, 0x10);
	D350T1013V1_DSI(d350t1013v1, 0xB0, 0xC0, 0x0C, 0x92, 0x0C, 0x10, 0x05,
		0x02, 0x0D, 0x07, 0x21, 0x04, 0x53, 0x11, 0x6A, 0x32, 0x1F);
	D350T1013V1_DSI(d350t1013v1, 0xB1, 0xC0, 0x87, 0xCF, 0x0C, 0x10, 0x06,
		0x00, 0x03, 0x08, 0x1D, 0x06, 0x54, 0x12, 0xE6, 0xEC, 0x0F);
	D350T1013V1_DSI(d350t1013v1, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x11);
	D350T1013V1_DSI(d350t1013v1, 0xB0, 0x5D);
	D350T1013V1_DSI(d350t1013v1, 0xB1, 0x62);
	D350T1013V1_DSI(d350t1013v1, 0xB2, 0x82);
	D350T1013V1_DSI(d350t1013v1, 0xB3, 0x80);
	D350T1013V1_DSI(d350t1013v1, 0xB5, 0x42);
	D350T1013V1_DSI(d350t1013v1, 0xB7, 0x85);
	D350T1013V1_DSI(d350t1013v1, 0xB8, 0x20);
	D350T1013V1_DSI(d350t1013v1, 0xC0, 0x09);
	D350T1013V1_DSI(d350t1013v1, 0xC1, 0x78);
	D350T1013V1_DSI(d350t1013v1, 0xC2, 0x78);
	D350T1013V1_DSI(d350t1013v1, 0xD0, 0x88);
	D350T1013V1_DSI(d350t1013v1, 0xEE, 0x42);
	msleep(200);

	D350T1013V1_DSI(d350t1013v1, 0xE0, 0x00, 0x00, 0x02);
	D350T1013V1_DSI(d350t1013v1, 0xE1, 0x04, 0xA0, 0x06, 0xA0, 0x05, 0xA0,
		0x07, 0xA0, 0x00, 0x44, 0x44);
	D350T1013V1_DSI(d350t1013v1, 0xE2, 0x00, 0x00, 0x33, 0x33, 0x01, 0xA0,
		0x00, 0x00, 0x01, 0xA0, 0x00, 0x00);
	D350T1013V1_DSI(d350t1013v1, 0xE3, 0x00, 0x00, 0x33, 0x33);
	D350T1013V1_DSI(d350t1013v1, 0xE4, 0x44, 0x44);
	D350T1013V1_DSI(d350t1013v1, 0xE5, 0x0C, 0x30, 0xA0, 0xA0, 0x0E, 0x32,
		0xA0, 0xA0, 0x08, 0x2C, 0xA0, 0xA0, 0x0A, 0x2E, 0xA0, 0xA0);
	D350T1013V1_DSI(d350t1013v1, 0xE6, 0x00, 0x00, 0x33, 0x33);
	D350T1013V1_DSI(d350t1013v1, 0xE7, 0x44, 0x44);
	D350T1013V1_DSI(d350t1013v1, 0xE8, 0x0D, 0x31, 0xA0, 0xA0, 0x0F, 0x33,
		0xA0, 0xA0, 0x09, 0x2D, 0xA0, 0xA0, 0x0B, 0x2F, 0xA0, 0xA0);
	D350T1013V1_DSI(d350t1013v1, 0xEB, 0x00, 0x01, 0xE4, 0xE4, 0x44, 0x88,
		0x00);
	D350T1013V1_DSI(d350t1013v1, 0xED, 0xFF, 0xF5, 0x47, 0x6F, 0x0B, 0xA1,
		0xA2, 0xBF, 0xFB, 0x2A, 0x1A, 0xB0, 0xF6, 0x74, 0x5F, 0xFF);
	D350T1013V1_DSI(d350t1013v1, 0xEF, 0x08, 0x08, 0x08, 0x40, 0x3F, 0x64);
	D350T1013V1_DSI(d350t1013v1, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x13);
	D350T1013V1_DSI(d350t1013v1, 0xE8, 0x00, 0x0E);
	D350T1013V1_DSI(d350t1013v1, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x00);
	D350T1013V1_DSI(d350t1013v1, 0x11);
	msleep(200);

	D350T1013V1_DSI(d350t1013v1, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x13);
	D350T1013V1_DSI(d350t1013v1, 0xE8, 0x00, 0x0C);
	msleep(200);

	D350T1013V1_DSI(d350t1013v1, 0xE8, 0x00, 0x00);
	D350T1013V1_DSI(d350t1013v1, 0xFF, 0x77, 0x01, 0x00, 0x00, 0x00);
	D350T1013V1_DSI(d350t1013v1, 0x3A, 0x50);
	msleep(200);
}

static int d350t1013v1_prepare(struct drm_panel *panel)
{
	struct d350t1013v1 *d350t1013v1 = panel_to_d350t1013v1(panel);
	int ret;
	u8 ids[3];

	ret = regulator_bulk_enable(d350t1013v1->desc->num_supplies,
				    d350t1013v1->supplies);
	if (ret < 0)
		return ret;

	gpiod_set_value(d350t1013v1->reset, 0);
	msleep(150);
	gpiod_set_value(d350t1013v1->reset, 1);
	msleep(150);

	ret = mipi_dsi_dcs_soft_reset(d350t1013v1->dsi);
	if (ret < 0)
		return ret;
	msleep(150);

	ret = mipi_dsi_dcs_exit_sleep_mode(d350t1013v1->dsi);
	if (ret < 0)
		return ret;
	msleep(150);

	/* Reading the display id ensures that the DSI link is working. */
	ret = mipi_dsi_dcs_read(d350t1013v1->dsi, MIPI_DCS_GET_DISPLAY_ID,
							ids, sizeof(ids));
	if (ret < 0)
		return ret;
	dev_info(&d350t1013v1->dsi->dev, "display id: %02x %02x %02x\n",
			ids[0], ids[1], ids[2]);

	d350t1013v1_init_sequence(d350t1013v1);

	return 0;
}

static int d350t1013v1_enable(struct drm_panel *panel)
{
	struct d350t1013v1 *d350t1013v1 = panel_to_d350t1013v1(panel);

	return mipi_dsi_dcs_set_display_on(d350t1013v1->dsi);
}

static int d350t1013v1_disable(struct drm_panel *panel)
{
	struct d350t1013v1 *d350t1013v1 = panel_to_d350t1013v1(panel);

	return mipi_dsi_dcs_set_display_off(d350t1013v1->dsi);
}

static int d350t1013v1_unprepare(struct drm_panel *panel)
{
	struct d350t1013v1 *d350t1013v1 = panel_to_d350t1013v1(panel);
	int ret;

	ret = mipi_dsi_dcs_enter_sleep_mode(d350t1013v1->dsi);
	if (ret < 0)
		return ret;
	msleep(150);

	regulator_bulk_disable(d350t1013v1->desc->num_supplies, d350t1013v1->supplies);

	return 0;
}

static int d350t1013v1_get_modes(struct drm_panel *panel,
			    struct drm_connector *connector)
{
	struct d350t1013v1 *d350t1013v1 = panel_to_d350t1013v1(panel);
	const struct drm_display_mode *desc_mode = d350t1013v1->desc->mode;
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, desc_mode);
	if (!mode) {
		dev_err(&d350t1013v1->dsi->dev, "failed to add mode %ux%u@%u\n",
			desc_mode->hdisplay, desc_mode->vdisplay,
			drm_mode_vrefresh(desc_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = desc_mode->width_mm;
	connector->display_info.height_mm = desc_mode->height_mm;

	return 1;
}

static const struct drm_panel_funcs d350t1013v1_funcs = {
	.disable	= d350t1013v1_disable,
	.unprepare	= d350t1013v1_unprepare,
	.prepare	= d350t1013v1_prepare,
	.enable		= d350t1013v1_enable,
	.get_modes	= d350t1013v1_get_modes,
};

static struct drm_display_mode d350t1013v1_mode = {
	.clock 			= 25000,

	.hdisplay		= 480,
	.hsync_start	= 480 + 50,
	.hsync_end		= 480 + 50 + 16,
	.htotal			= 480 + 50 + 16 + 2,

	.vdisplay		= 800,
	.vsync_start	= 800 + 16,
	.vsync_end		= 800 + 16 + 14,
	.vtotal			= 800 + 16 + 14 + 2,

	.width_mm		= 45,
	.height_mm		= 76,

	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static uint clock = 25000;
module_param(clock, uint, 0);

static uint hsync_start = 480 + 50;
module_param(hsync_start, uint, 0);
static uint hsync_end = 480 + 50 + 16;
module_param(hsync_end, uint, 0);
static uint htotal = 480 + 50 + 16 + 2;
module_param(htotal, uint, 0);

static uint vsync_start = 800 + 16;
module_param(vsync_start, uint, 0);
static uint vsync_end = 800 + 16 + 14;
module_param(vsync_end, uint, 0);
static uint vtotal = 800 + 16 + 14 + 2;
module_param(vtotal, uint, 0);

static const char * const d350t1013v1_supply_names[] = {
	"vcc",
};

static const struct d350t1013v1_panel_desc d350t1013v1_desc = {
	.mode = &d350t1013v1_mode,
	.lanes = 2,
	.flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS |
			MIPI_DSI_MSG_USE_LPM,
	.format = MIPI_DSI_FMT_RGB888,
	.supply_names = d350t1013v1_supply_names,
	.num_supplies = ARRAY_SIZE(d350t1013v1_supply_names),
};

static int d350t1013v1_dsi_probe(struct mipi_dsi_device *dsi)
{
	const struct d350t1013v1_panel_desc *desc;
	struct d350t1013v1 *d350t1013v1;
	int ret, i;

	d350t1013v1 = devm_kzalloc(&dsi->dev, sizeof(*d350t1013v1), GFP_KERNEL);
	if (!d350t1013v1)
		return -ENOMEM;

	d350t1013v1_mode.clock = clock;
	d350t1013v1_mode.hsync_start = hsync_start;
	d350t1013v1_mode.hsync_end = hsync_end;
	d350t1013v1_mode.htotal = htotal;
	d350t1013v1_mode.vsync_start = vsync_start;
	d350t1013v1_mode.vsync_end = vsync_end;
	d350t1013v1_mode.vtotal = vtotal;

	dev_info(&dsi->dev, "clock=%d hsync_start=%d hsync_end=%d htotal=%d vsync_start=%d vsync_end=%d vtotal=%d\n",
		d350t1013v1_mode.clock,
		d350t1013v1_mode.hsync_start, d350t1013v1_mode.hsync_end, d350t1013v1_mode.htotal,
		d350t1013v1_mode.vsync_start, d350t1013v1_mode.vsync_end, d350t1013v1_mode.vtotal
	);

	desc = of_device_get_match_data(&dsi->dev);
	dsi->mode_flags = desc->flags;
	dsi->format = desc->format;
	dsi->lanes = desc->lanes;

	d350t1013v1->supplies = devm_kcalloc(&dsi->dev, desc->num_supplies,
					sizeof(*d350t1013v1->supplies),
					GFP_KERNEL);
	if (!d350t1013v1->supplies)
		return -ENOMEM;

	for (i = 0; i < desc->num_supplies; i++)
		d350t1013v1->supplies[i].supply = desc->supply_names[i];

	ret = devm_regulator_bulk_get(&dsi->dev, desc->num_supplies,
				      d350t1013v1->supplies);
	if (ret < 0)
		return ret;

	d350t1013v1->reset = devm_gpiod_get(&dsi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(d350t1013v1->reset)) {
		dev_err(&dsi->dev, "Couldn't get our reset GPIO\n");
		return PTR_ERR(d350t1013v1->reset);
	}

	drm_panel_init(&d350t1013v1->panel, &dsi->dev, &d350t1013v1_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ret = drm_panel_of_backlight(&d350t1013v1->panel);
	if (ret)
		return ret;

	drm_panel_add(&d350t1013v1->panel);

	mipi_dsi_set_drvdata(dsi, d350t1013v1);
	d350t1013v1->dsi = dsi;
	d350t1013v1->desc = desc;

	return mipi_dsi_attach(dsi);
}

static int d350t1013v1_dsi_remove(struct mipi_dsi_device *dsi)
{
	struct d350t1013v1 *d350t1013v1 = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&d350t1013v1->panel);

	return 0;
}

static const struct of_device_id d350t1013v1_of_match[] = {
	{ .compatible = "dxwy,d350t1013v1", .data = &d350t1013v1_desc },
	{ }
};
MODULE_DEVICE_TABLE(of, d350t1013v1_of_match);

static struct mipi_dsi_driver d350t1013v1_dsi_driver = {
	.probe		= d350t1013v1_dsi_probe,
	.remove		= d350t1013v1_dsi_remove,
	.driver = {
		.name		= "d350t1013v1",
		.of_match_table	= d350t1013v1_of_match,
	},
};
module_mipi_dsi_driver(d350t1013v1_dsi_driver);

MODULE_AUTHOR("Sebastian Urban <surban@surban.net>");
MODULE_DESCRIPTION("DXWY D350T1013V1 LCD Panel Driver");
MODULE_LICENSE("GPL");
