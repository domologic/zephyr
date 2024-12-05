/*
 * Copyright (c) 2022 Domologic Home Automation GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT atmel_maxtouch

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(atmel_maxtouch, CONFIG_INPUT_LOG_LEVEL);

#define MAX_OBJ_INFO_COUNT 36

// generic registers
#define REG_OBJ_NUM				0x06
#define REG_OBJ_INFO				0x07

// GEN_COMMANDPROCESSOR_T6 registers
#define REG_T6_RESET				0x00
#define REG_T6_CALIBRATE			0x02
#define REG_T6_REPORTALL			0x03

// TOUCH_MULTITOUCHSCREEN_T100 registers
#define REG_T100_CTRL				0x00
#define REG_T100_TCHEVENTCFG			0x04

// TOUCH_MULTITOUCHSCREEN_T100 CTRL options
#define T100_CTRL_ENABLE			0x01
#define T100_CTRL_RPTEN				0x02

// TOUCH_MULTITOUCHSCREEN_T100 TCHEVENTCFG options
#define T100_TCHEVENTCFG_DISUNSUP		0x08
#define T100_TCHEVENTCFG_DISSUP			0x10
#define T100_TCHEVENTCFG_DISMOVE		0x20

// TOUCH_MULTITOUCHSCREEN_T100 report ids
#define T100_REPORT_ID_TOUCH_FIRST		2

// TOUCH_MULTITOUCHSCREEN_T100 events
#define T100_EV_MOVE				1
#define T100_EV_DOWN				4
#define T100_EV_UP				5

enum atmel_maxtouch_obj_id_t {
	OBJ_GEN_MESSAGEPROCESSOR_T5 	= 5,
	OBJ_SPT_MESSAGECOUNT_T44	= 44,
	OBJ_TOUCH_MULTITOUCHSCREEN_T100 = 100
};

struct atmel_maxtouch_obj_info {
	uint8_t type;
	uint16_t start_addr;
	uint8_t size;
	uint8_t instances;
	uint8_t num_report_ids;
} __packed;

struct atmel_maxtouch_config {
	struct i2c_dt_spec bus;
	struct gpio_dt_spec int_gpio;
	struct gpio_dt_spec rst_gpio;
};

struct atmel_maxtouch_message {
	uint8_t report_id;
	struct {
		uint8_t event : 4;
		uint8_t type : 3;
		uint8_t detect : 1;
	} tchstatus;
	uint16_t xpos;
	uint16_t ypos;
} __packed;

struct atmel_maxtouch_data {
	const struct device *dev;
	struct k_work work;
	struct gpio_callback int_gpio_cb;

	uint16_t obj_gen_messageprocessor;
	uint16_t obj_spt_messagecount;
	uint16_t obj_touch_multitouchscreen;

	uint8_t obj_gen_messageprocessor_size;

	uint8_t obj_touch_multitouchscreen_id;

	bool is_down;
};

static int atmel_maxtouch_read_data(const struct device *dev, uint16_t addr, void* buf, size_t buf_size)
{
	const struct atmel_maxtouch_config *config = dev->config;

	return i2c_write_read_dt(&config->bus, &addr, sizeof(uint16_t), buf, buf_size);
}

static int atmel_maxtouch_write_byte(const struct device *dev, uint16_t addr, uint8_t value)
{
	const struct atmel_maxtouch_config *config = dev->config;

	struct i2c_msg msg[2];

	msg[0].buf = (uint8_t *)&addr;
	msg[0].len = sizeof(addr);
	msg[0].flags = I2C_MSG_WRITE;

	msg[1].buf = &value;
	msg[1].len = sizeof(uint8_t);
	msg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer_dt(&config->bus, msg, 2);
}

static int atmel_maxtouch_hard_reset(const struct device *dev)
{
	const struct atmel_maxtouch_config *config = dev->config;
	int r;

	// hold RESET pin for at least 90 ns to cause a reset
	r = gpio_pin_set_dt(&config->rst_gpio, 0);
	if (r < 0) {
		LOG_ERR("Could not set reset pin to low");
		return r;
	}

	k_sleep(K_NSEC(90));

	// after releasing the RESET pin the device typically takes
	// 38 ms before it is ready to start communications
	r = gpio_pin_set_dt(&config->rst_gpio, 1);
	if (r < 0) {
		LOG_ERR("Could not set reset pin to high");
		return r;
	}

	k_sleep(K_MSEC(38));

	return 0;
}

static void atmel_maxtouch_work_handler(struct k_work *work)
{
	struct atmel_maxtouch_data *data = CONTAINER_OF(work, struct atmel_maxtouch_data, work);
	int r;
	uint16_t buf_size;
	uint8_t i;
	uint8_t count;
	uint8_t *buf_ptr;

	r = atmel_maxtouch_read_data(data->dev, data->obj_spt_messagecount, &count, sizeof(uint8_t));
	if (r < 0) {
		LOG_ERR("Could not read message count");
		return;
	}

	if (!count) {
		return;
	}

	buf_size = data->obj_gen_messageprocessor_size * count;
	uint8_t buf[buf_size];
	r = atmel_maxtouch_read_data(data->dev, data->obj_gen_messageprocessor, buf, buf_size);
	if (r < 0) {
		LOG_ERR("Could not read message data");
		return;
	}

	buf_ptr = buf;
	for (i = 0; i < count; ++i) {
		struct atmel_maxtouch_message* msg = (struct atmel_maxtouch_message*)buf_ptr;

		if (msg->report_id == data->obj_touch_multitouchscreen_id) {
			switch (msg->tchstatus.event) {
				case T100_EV_MOVE: {
					input_report_abs(data->dev, INPUT_ABS_X, msg->xpos, false, K_FOREVER);
					input_report_abs(data->dev, INPUT_ABS_Y, msg->ypos, false, K_FOREVER);
					input_report_key(data->dev, INPUT_BTN_TOUCH, data->is_down, true, K_FOREVER);
					break;
				}
				case T100_EV_DOWN: {
					data->is_down = true;
					input_report_abs(data->dev, INPUT_ABS_X, msg->xpos, false, K_FOREVER);
					input_report_abs(data->dev, INPUT_ABS_Y, msg->ypos, false, K_FOREVER);
					input_report_key(data->dev, INPUT_BTN_TOUCH, true, true, K_FOREVER);
					break;
				}
				case T100_EV_UP: {
					data->is_down = false;
					input_report_abs(data->dev, INPUT_ABS_X, msg->xpos, false, K_FOREVER);
					input_report_abs(data->dev, INPUT_ABS_Y, msg->ypos, false, K_FOREVER);
					input_report_key(data->dev, INPUT_BTN_TOUCH, false, true, K_FOREVER);
					break;
				}
				default: {
					break;
				}
			}
		}

		buf_ptr += data->obj_gen_messageprocessor_size;
	}
}

static void atmel_maxtouch_isr_handler(const struct device *dev,
					struct gpio_callback *cb, uint32_t pins)
{
	struct atmel_maxtouch_data *data = CONTAINER_OF(cb, struct atmel_maxtouch_data, int_gpio_cb);

	k_work_submit(&data->work);
}

static int atmel_maxtouch_init_obj_table(const struct device *dev)
{
	struct atmel_maxtouch_data *data = dev->data;

	struct atmel_maxtouch_obj_info obj_info_table[MAX_OBJ_INFO_COUNT];
	struct atmel_maxtouch_obj_info *obj_info = NULL;
	int r;
	uint8_t obj_num;
	uint8_t report_id = 1;
	uint8_t i;

	r = atmel_maxtouch_read_data(dev, REG_OBJ_NUM, &obj_num, sizeof(obj_num));
	if (r < 0) {
		LOG_ERR("Could not read device info");
		return r;
	}

	r = atmel_maxtouch_read_data(dev, REG_OBJ_INFO, obj_info_table, sizeof(struct atmel_maxtouch_obj_info) * obj_num);
	if (r < 0) {
		LOG_ERR("Could not read object info table");
		return r;
	}

	for (i = 0; i < obj_num; ++i) {
		obj_info = obj_info_table + i;

		switch (obj_info->type) {
			case OBJ_GEN_MESSAGEPROCESSOR_T5: {
				data->obj_gen_messageprocessor = obj_info->start_addr;
				data->obj_gen_messageprocessor_size = obj_info->size;
				break;
			}
			case OBJ_SPT_MESSAGECOUNT_T44: {
				data->obj_spt_messagecount = obj_info->start_addr;
				break;
			}
			case OBJ_TOUCH_MULTITOUCHSCREEN_T100: {
				data->obj_touch_multitouchscreen = obj_info->start_addr;
				data->obj_touch_multitouchscreen_id = report_id + T100_REPORT_ID_TOUCH_FIRST;
				break;
			}
			default: {
				break;
			}
		}

		if (obj_info->num_report_ids) {
			report_id += obj_info->num_report_ids * (obj_info->instances + 1);
		}
	}

	return 0;
}

static int atmel_maxtouch_init(const struct device *dev)
{
	const struct atmel_maxtouch_config *config = dev->config;
	struct atmel_maxtouch_data *data = dev->data;
	int r;

	data->dev = dev;
	data->is_down = false;

	if (!i2c_is_ready_dt(&config->bus)) {
		LOG_ERR("I2C controller device not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&config->rst_gpio)) {
		LOG_ERR("Reset GPIO controller device not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&config->int_gpio)) {
		LOG_ERR("Interrupt GPIO controller device not ready");
		return -ENODEV;
	}

	r = gpio_pin_configure_dt(&config->rst_gpio, GPIO_OUTPUT);
	if (r < 0) {
		LOG_ERR("Could not configure reset GPIO pin");
		return r;
	}

	r = gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT | GPIO_PULL_UP);
	if (r < 0) {
		LOG_ERR("Could not configure interrupt GPIO pin");
		return r;
	}

	r = gpio_pin_interrupt_configure_dt(&config->int_gpio,
						GPIO_INT_EDGE_FALLING);
	if (r < 0) {
		LOG_ERR("Could not configure interrupt GPIO interrupt.");
		return r;
	}

	r = atmel_maxtouch_hard_reset(dev);
	if (r < 0) {
		LOG_ERR("Could not hardware reset");
		return r;
	}

	r = atmel_maxtouch_init_obj_table(dev);
	if (r < 0) {
		LOG_ERR("Could not init object table");
		return r;
	}

	r = atmel_maxtouch_write_byte(dev, data->obj_touch_multitouchscreen + REG_T100_TCHEVENTCFG,
				T100_TCHEVENTCFG_DISSUP  |
				T100_TCHEVENTCFG_DISUNSUP);
	if (r < 0) {
		LOG_ERR("Could not write TOUCH_MULTITOUCHSCREEN_T100 CTRL");
		return r;
	}

	r = atmel_maxtouch_write_byte(dev, data->obj_touch_multitouchscreen + REG_T100_CTRL,
				T100_CTRL_ENABLE | T100_CTRL_RPTEN);
	if (r < 0) {
		LOG_ERR("Could not write TOUCH_MULTITOUCHSCREEN_T100 CTRL");
		return r;
	}

	r = gpio_add_callback_dt(&config->int_gpio, &data->int_gpio_cb);
	if (r) {
		return r;
	}

	k_work_submit(&data->work);
	return 0;
}

#define ATMEL_MAXTOUCH_INIT(index)								\
	static const struct atmel_maxtouch_config atmel_maxtouch_config_##index = {		\
		.bus		= I2C_DT_SPEC_INST_GET(index),					\
		.int_gpio	= GPIO_DT_SPEC_INST_GET(index, int_gpios),			\
		.rst_gpio	= GPIO_DT_SPEC_INST_GET(index, rst_gpios),			\
	};											\
	static struct atmel_maxtouch_data atmel_maxtouch_data_##index = {			\
		.work        = Z_WORK_INITIALIZER(atmel_maxtouch_work_handler),			\
		.int_gpio_cb = {								\
				.handler  = atmel_maxtouch_isr_handler,				\
				.pin_mask = BIT(atmel_maxtouch_config_##index.int_gpio.pin)	\
        	}										\
	};											\
	DEVICE_DT_INST_DEFINE(index, atmel_maxtouch_init, NULL,					\
				&atmel_maxtouch_data_##index, &atmel_maxtouch_config_##index,	\
				POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY,			\
				NULL);

DT_INST_FOREACH_STATUS_OKAY(ATMEL_MAXTOUCH_INIT)
