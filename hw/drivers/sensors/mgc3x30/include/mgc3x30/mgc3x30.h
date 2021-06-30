/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#ifndef __DRIVERS_MGC3X30_H_
#define __DRIVERS_MGC3X30_H_

#include <stdint.h>
#include <hal/hal_i2c.h>
#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
#include <bus/drivers/i2c_common.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

struct mgc3x30_dev;

struct mgc3x30_cfg {
#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    struct bus_i2c_node_cfg node_cfg;
#else
    struct hal_i2c_settings i2c_settings;
    int i2c_num;
    uint8_t i2c_addr;
#endif
    int8_t eio0_ts_pin;
    int8_t eio1_pin;
    int8_t eio2_pin;
    int8_t eio3_pin;
    int8_t eio4_pin;
    int8_t mclr_pin;
    uint16_t north_rx: 3;
    uint16_t south_rx: 3;
    uint16_t west_rx: 3;
    uint16_t east_rx: 3;
    uint16_t center_rx: 3;
};

struct mgc3x30_fw_version_info;

struct mgc3x30_client {
    void (*ready)(struct mgc3x30_dev *dev, const struct mgc3x30_fw_version_info *info);
    void (*gesture)(struct mgc3x30_dev *dev, uint32_t gesture_bits, uint16_t x, uint16_t y, uint16_t z);
    void (*touch)(struct mgc3x30_dev *dev, uint32_t touch_bits, uint16_t x, uint16_t y, uint16_t z);
    void (*air_wheel)(struct mgc3x30_dev *dev, uint16_t air_wheel_bits);
    void (*noise_power)(struct mgc3x30_dev *dev, uint32_t power);
    void (*cic_data)(struct mgc3x30_dev *dev, const uint8_t *data);
    void (*sd_data)(struct mgc3x30_dev *dev, const uint8_t *data);
};

/**
 * Set client data to already opened MGC3X30 device.
 *
 * @param dev    - device to set client to
 * @param client - client structure (callbacks) can be NULL to stop receiving
 *                 notifications.
 * @return 0 on success, non-zero on failure
 */
int mgc3x30_set_client(struct mgc3x30_dev *dev, const struct mgc3x30_client *client);

/**
 * Open MGC3X30 device with specified client data
 *
 * @param name   - device name
 * @param client - client structure (callbacks)
 * @return pointer to device if device can be opened, otherwise NULL.
 */
struct mgc3x30_dev *mgc3x30_open(const char *name, const struct mgc3x30_client *client);

/**
 * Create MGC3X30 device.
 *
 * @param mgc3x30_dev - device structure to initialized
 * @param name        - name of mgc3x30 device
 * @param cfg         - device configuration
 * @return 0 on success, non-zero on failure
 */
int mgc3x30_dev_create(struct mgc3x30_dev *mgc3x30_dev, const char *name,
                       const struct mgc3x30_cfg *cfg);

/**
 *
 * @param dev
 * @param paramter_id
 * @param arg0
 * @param arg1
 * @return
 */
int mgc3x30_get_runtime_parameter(struct mgc3x30_dev *dev, uint16_t paramter_id, uint32_t *arg0,
                                  uint32_t *arg1);

int mgc3x30_get_fw_info(struct mgc3x30_dev *dev, const struct mgc3x30_fw_version_info *info);

void mgc3x30_reset(struct mgc3x30_dev *dev);

#include "../../src/mgc3x30_priv.h"

#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_MGC3X30_H_ */
