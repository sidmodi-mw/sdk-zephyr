/*
 * Copyright (c) 2023 Alen Daniel
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT kyo_tn0xxx

#include <zephyr/logging/log.h>

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/init.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include "lvgl.h"
#include "lvgl_display.h"

/* Driver for Kyocera's 2.16" RESOLUTION MEMORY IN PIXEL (MIP) TFT (TN0216ANVNANN)
 * Note:
 * high/1 means white, low/0 means black
 * SPI interface expects LSB first
 * see more notes in boards/shields/tn0xxx/doc/index.rst
 */
LOG_MODULE_REGISTER(tn0xxx, CONFIG_DISPLAY_LOG_LEVEL);

#define TN0XXX_PANEL_WIDTH  DT_INST_PROP(0, width)
#define TN0XXX_PANEL_HEIGHT DT_INST_PROP(0, height)

#define TN0XXX_PIXELS_PER_BYTE 8

#define LCD_ADDRESS_LEN_BITS          8
#define LCD_DUMMY_SPI_CYCLES_LEN_BITS 32
#define DUMMY_BYTE                    0x00

/* Data packet format
 * +-------------------+-------------------+--------------------+
 * | line addr (8 bits) | data (8 WIDTH bits) | dummy (32 bits) |
 * +-------------------+-------------------+--------------------+
 */

struct tn0xxx_config_s {
	struct spi_dt_spec bus;
};

struct tn0xxx_data_s {
	enum display_orientation orientation;
	enum display_pixel_format pixel_format;
};

static int tn0xxx_blanking_off(const struct device *dev)
{
	LOG_WRN("Unsupported");
	return -ENOTSUP;
}

static int tn0xxx_blanking_on(const struct device *dev)
{
	LOG_WRN("Unsupported");
	return -ENOTSUP;
}

static int tn0xxx_read(const struct device *dev, const uint16_t x, const uint16_t y,
		       const struct display_buffer_descriptor *desc, void *buf)
{
	LOG_ERR("not supported");
	return -ENOTSUP;
}

static void *tn0xxx_get_framebuffer(const struct device *dev)
{
	LOG_ERR("not supported");
	return NULL;
}

static int tn0xxx_set_brightness(const struct device *dev, const uint8_t brightness)
{
	LOG_WRN("not supported");
	return -ENOTSUP;
}

static int tn0xxx_set_contrast(const struct device *dev, uint8_t contrast)
{
	LOG_WRN("not supported");
	return -ENOTSUP;
}

static int tn0xxx_set_orientation(const struct device *dev,
				  const enum display_orientation new_orientation)
{
	LOG_WRN("not supported");
	return -ENOTSUP;
}

static int tn0xxx_set_pixel_format(const struct device *dev, const enum display_pixel_format pf)
{

	lv_disp_t *disp = lv_disp_get_default();
	struct lvgl_disp_data *disp_data = disp->driver->user_data;
	struct display_capabilities *caps = &disp_data->cap;
	struct tn0xxx_data_s *data = dev->data;

	if (pf & caps->supported_pixel_formats) {
		caps->current_pixel_format = pf;
		data->pixel_format = pf;
		lv_disp_drv_update(disp, disp->driver);
		return 0;
	}

	LOG_ERR("specified pixel format %d not supported, supported formats are %d", pf,
		caps->supported_pixel_formats);
	return -ENOTSUP;
}

static int update_display(const struct device *dev, uint16_t start_line, uint16_t num_lines,
			  const uint8_t *bitmap_buffer)
{

	const struct tn0xxx_config_s *config = dev->config;
	uint8_t single_line_buffer[(TN0XXX_PANEL_WIDTH + LCD_DUMMY_SPI_CYCLES_LEN_BITS +
				    LCD_ADDRESS_LEN_BITS) /
				   TN0XXX_PIXELS_PER_BYTE];
	int err = 0;

	uint16_t bitmap_buffer_index = 0;
	for (int column_addr = start_line; column_addr < start_line + num_lines; column_addr++) {
		uint8_t buff_index = 0;

		single_line_buffer[buff_index++] = (uint8_t)column_addr;

		for (int i = 0; i < TN0XXX_PANEL_WIDTH / TN0XXX_PIXELS_PER_BYTE; i++) {
			single_line_buffer[buff_index++] = bitmap_buffer[bitmap_buffer_index++];
		}
		for (int i = 0; i < LCD_DUMMY_SPI_CYCLES_LEN_BITS / TN0XXX_PIXELS_PER_BYTE; i++) {
			single_line_buffer[buff_index++] = DUMMY_BYTE;
		}

		int len = sizeof(single_line_buffer);

		struct spi_buf tx_buf = {.buf = single_line_buffer, .len = len};
		struct spi_buf_set tx_bufs = {.buffers = &tx_buf, .count = 1};

		err |= spi_write_dt(&config->bus, &tx_bufs);
	}

	return err;
}

static int tn0xxx_write(const struct device *dev, const uint16_t x, const uint16_t y,
			const struct display_buffer_descriptor *desc, const void *buf)
{
	const struct tn0xxx_data_s *data = dev->data;

	lv_disp_t *disp = lv_disp_get_default();
	struct lvgl_disp_data *disp_data = disp->driver->user_data;
	struct display_capabilities *caps = &disp_data->cap;

	LOG_DBG("X: %d, Y: %d, W: %d, H: %d, pitch: %d, Buf size: %d", x, y, desc->width,
		desc->height, desc->pitch, desc->buf_size);

	if (buf == NULL) {
		LOG_WRN("Display buffer is not available");
		return -EINVAL;
	}

	if ((y + desc->height) > TN0XXX_PANEL_HEIGHT) {
		LOG_ERR("Buffer out of bounds (height)");
		return -EINVAL;
	}

	if (desc->width != TN0XXX_PANEL_WIDTH) {
		LOG_ERR("Width restricted to panel width %d.. user provided %d", TN0XXX_PANEL_WIDTH,
			desc->width);
		return -EINVAL;
	}

	if (x != 0) {
		LOG_ERR("x-coordinate has to be 0");
		return -EINVAL;
	};

	return update_display(dev, y, desc->height, buf);
}

static void tn0xxx_get_capabilities(const struct device *dev, struct display_capabilities *caps)
{
	memset(caps, 0, sizeof(struct display_capabilities));
	caps->x_resolution = TN0XXX_PANEL_WIDTH;
	caps->y_resolution = TN0XXX_PANEL_HEIGHT;
	caps->supported_pixel_formats = PIXEL_FORMAT_MONO01 | PIXEL_FORMAT_MONO10;
	caps->current_pixel_format = PIXEL_FORMAT_MONO01;
	caps->current_orientation = DISPLAY_ORIENTATION_NORMAL;
	caps->screen_info = SCREEN_INFO_X_ALIGNMENT_WIDTH;
}

static int tn0xxx_init(const struct device *dev)
{
	const struct tn0xxx_config_s *config = dev->config;

	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("SPI bus %s not ready", config->bus.bus->name);
		return -ENODEV;
	}

	return 0;
}

static struct display_driver_api tn0xxx_driver_api = {
	.blanking_on = tn0xxx_blanking_on,
	.blanking_off = tn0xxx_blanking_off,
	.write = tn0xxx_write,
	.read = tn0xxx_read,
	.get_framebuffer = tn0xxx_get_framebuffer,
	.set_brightness = tn0xxx_set_brightness,
	.set_contrast = tn0xxx_set_contrast,
	.get_capabilities = tn0xxx_get_capabilities,
	.set_pixel_format = tn0xxx_set_pixel_format,
	.set_orientation = tn0xxx_set_orientation,
};

static struct tn0xxx_data_s tn0xxx_data = {.orientation = DISPLAY_ORIENTATION_NORMAL};

static const struct tn0xxx_config_s tn0xxx_config = {
	.bus = SPI_DT_SPEC_INST_GET(0,
				    SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_LSB |
					    SPI_CS_ACTIVE_HIGH | SPI_HOLD_ON_CS | SPI_LOCK_ON,
				    2),
};

DEVICE_DT_INST_DEFINE(0, tn0xxx_init, NULL, &tn0xxx_data, &tn0xxx_config, POST_KERNEL,
		      CONFIG_DISPLAY_INIT_PRIORITY, &tn0xxx_driver_api);
