/*
 * Copyright (c) 2025 Alex Bucknall
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT plantower_pmsa003i

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(PMSA003I, CONFIG_SENSOR_LOG_LEVEL);

// PMSA003I registers
#define PMSA003I_REG_PM_1_0     0x04
#define PMSA003I_REG_PM_2_5     0x06
#define PMSA003I_REG_PM_10      0x08

// PMSA003I start bytes
#define PMSA003I_REG_START_BYTE_1 0x42
#define PMSA003I_REG_START_BYTE_2 0x4d

// PMSA003I serial timeout
#define CFG_PMSA003I_SERIAL_TIMEOUT 1000

struct pmsa003i_data {
	uint16_t pm_1_0;
	uint16_t pm_2_5;
	uint16_t pm_10;
};

struct pmsa003i_config {
	struct i2c_dt_spec i2c;
};

static int pmsa003i_init(const struct device *dev)
{
	const struct pmsa003i_config *cfg = dev->config;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	return 0;
}

static uint8_t pmsa003i_calculate_checksum(uint8_t *buf) {
    uint16_t sum = 0;
    for (size_t i = 0; i < 30; i++) {
        sum += buf[i];
    }
    return (uint8_t)sum;
}

static int pmsa003i_read_with_timeout(const struct pmsa003i_config *cfg, uint8_t *buf, size_t len) {
    int64_t start_time = k_uptime_get();
    int64_t timeout_ms = CFG_PMSA003I_SERIAL_TIMEOUT;
	int rc;

    while (k_uptime_get() - start_time < timeout_ms) {
        rc = i2c_read_dt(&cfg->i2c, buf, len);
        if (rc < 0) {
            return rc;
        }

        if (buf[0] == PMSA003I_REG_START_BYTE_1 &&
            buf[1] == PMSA003I_REG_START_BYTE_2) {
            return 0;
        }

        k_sleep(K_MSEC(10));
    }

    return -ETIMEDOUT;
}

static int pmsa003i_sample_fetch(const struct device *dev,
								enum sensor_channel chan)
{
	const struct pmsa003i_config *cfg = dev->config;
	struct pmsa003i_data *drv_data = dev->data;

	uint8_t pmsa003i_read_buffer[32] = {0};
	uint8_t calculated_checksum;
	int rc;

	/* sample output */
	/* 42 4D 00 1C 00 01 00 01 00 01 00 01 00 01 00 01 01 92
	 * 00 4E 00 03 00 00 00 00 00 00 71 00 02 06
	 */

    rc = pmsa003i_read_with_timeout(cfg, pmsa003i_read_buffer, sizeof(pmsa003i_read_buffer));
    if (rc < 0) {
        LOG_WRN("Failed to read valid data within timeout (err: %d)", rc);
        return rc;
    }

	// Check checksum
    calculated_checksum = pmsa003i_calculate_checksum(pmsa003i_read_buffer);
    if (calculated_checksum != pmsa003i_read_buffer[31]) {
        LOG_WRN("checksum mismatch (calc: 0x%02X, recv: 0x%02X)",
                calculated_checksum, pmsa003i_read_buffer[31]);
        return -EIO;
    }

	drv_data->pm_1_0 =
	    (pmsa003i_read_buffer[PMSA003I_REG_PM_1_0] << 8) + pmsa003i_read_buffer[PMSA003I_REG_PM_1_0 + 1];
	drv_data->pm_2_5 =
	    (pmsa003i_read_buffer[PMSA003I_REG_PM_2_5] << 8) + pmsa003i_read_buffer[PMSA003I_REG_PM_2_5 + 1];
	drv_data->pm_10 =
	    (pmsa003i_read_buffer[PMSA003I_REG_PM_10] << 8) + pmsa003i_read_buffer[PMSA003I_REG_PM_10 + 1];

	return 0;
}

static int pmsa003i_channel_get(const struct device *dev,
							   enum sensor_channel chan,
			    			   struct sensor_value *val)
{
	const struct pmsa003i_data *data = dev->data;

    switch (chan) {
        case SENSOR_CHAN_PM_1_0:
            val->val1 = data->pm_1_0;
            val->val2 = 0;
            break;
        case SENSOR_CHAN_PM_2_5:
            val->val1 = data->pm_2_5;
            val->val2 = 0;
            break;
        case SENSOR_CHAN_PM_10:
            val->val1 = data->pm_10;
            val->val2 = 0;
            break;
        default:
            LOG_ERR("Unsupported channel: %d", chan);
            return -ENOTSUP;
    }

	return 0;
}

static DEVICE_API(sensor, pmsa003i_api) = {
	.sample_fetch = pmsa003i_sample_fetch,
	.channel_get = pmsa003i_channel_get,
};

#define PMSA003I_DEFINE(inst)                                        		   \
																			   \
	static struct pmsa003i_data pmsa003i_data_##inst;                 		   \
                                                                    		   \
	static const struct pmsa003i_config pmsa003i_config_##inst = {			   \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                          		   \
	};        																   \
                                                                    		   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, 										   \
								 pmsa003i_init, 							   \
								 NULL, 										   \
								 &pmsa003i_data_##inst,						   \
				     			 &pmsa003i_config_##inst, 					   \
								 POST_KERNEL,                       		   \
				     			 CONFIG_SENSOR_INIT_PRIORITY, 				   \
								 &pmsa003i_api);						       \

DT_INST_FOREACH_STATUS_OKAY(PMSA003I_DEFINE)