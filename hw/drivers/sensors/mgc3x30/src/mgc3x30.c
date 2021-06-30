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

#include <assert.h>
#include <string.h>

#include <os/mynewt.h>
#include <bsp/bsp.h>
#include <hal/hal_gpio.h>
#include <hal/hal_uart.h>

#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
#include <bus/drivers/spi_common.h>
#endif
#include <mgc3x30/mgc3x30.h>
#include "mgc3x30_priv.h"

#include <uart/uart.h>
#include <console/console.h>

static inline int
mgc3x30_lock(struct mgc3x30_dev *dev)
{
    return os_error_to_sys(os_mutex_pend(&dev->lock,
                                         os_time_ms_to_ticks32(MYNEWT_VAL(MGC3X30_LOCK_TIMEOUT))));
}

static inline void
mgc3x30_unlock(struct mgc3x30_dev *dev)
{
    int rc = os_mutex_release(&dev->lock);
    assert(rc == 0);
}

#if MYNEWT_VAL(BUS_DRIVER_PRESENT)

static int
mgc3x30_i2c_write(struct mgc3x30_dev *dev, mgc3x30_msg_t *msg, uint8_t msg_size)
{
    int rc;

    rc = bus_node_lock((struct os_dev *)&dev->dev, 1000);
    if (rc == 0) {
        rc = bus_node_simple_write((struct os_dev *)&dev->dev, msg, msg_size);
        (void)bus_node_unlock((struct os_dev *)&dev->dev);
    }

    return rc;
}

int
mgc3x30_i2c_read(struct mgc3x30_dev *dev, mgc3x30_msg_t *msg, int length)
{
    int rc;

    rc = bus_node_lock((struct os_dev *)&dev->dev, 1000);
    if (rc == 0) {
        rc = bus_node_read((struct os_dev *)&dev->dev, msg, length, 1000, BUS_F_NONE);
    }
    (void)bus_node_unlock((struct os_dev *)&dev->dev);
    return rc;
}

#else

static int
mgc3x30_i2c_write(struct mgc3x30_dev *dev, mgc3x30_msg_t *msg, uint8_t msg_size)
{
    struct hal_i2c_master_data data;
    data.address = dev->cfg.i2c_addr;

    /* Send the register to read from */
    data.buffer = (uint8_t *)&msg;
    data.len = msg_size;

    return hal_i2c_master_write(dev->cfg.i2c_num, &data, 1000, 1);
}

static int
mgc3x30_i2c_read(struct mgc3x30_dev *dev, mgc3x30_msg_t *msg, int length)
{
    int rc;
    struct hal_i2c_master_data data = {
        .address = dev->cfg.i2c_addr,
        .buffer = (uint8_t *)msg,
        .len = length,
    };

    rc = hal_i2c_master_read(dev->cfg.i2c_num, &data, 1000, 1);
//    if (rc == 0 && msg->hdr.size > 4) {
//        data.len = msg->hdr.size - 4;
//        rc = hal_i2c_master_read(dev->cfg.i2c_num, &data, 1000, 1);
//    }

    return rc;
}

#endif

static void
mgc3x30_isr(struct mgc3x30_dev *dev)
{
    dev->irq_pending = true;
    os_eventq_put(dev->event_queue, &dev->irq_event);
}

static void
mgc3x30_enable_ts_int(struct mgc3x30_dev *dev)
{
    hal_gpio_irq_init(dev->cfg.eio0_ts_pin, (hal_gpio_irq_handler_t)mgc3x30_isr,
                      dev, HAL_GPIO_TRIG_FALLING, HAL_GPIO_PULL_NONE);
    hal_gpio_irq_enable(dev->cfg.eio0_ts_pin);
}

static void
mgc3x30_ts_down(struct mgc3x30_dev *dev)
{
    hal_gpio_irq_release(dev->cfg.eio0_ts_pin);
    hal_gpio_init_out_od(dev->cfg.eio0_ts_pin, 0);
}

int
mgc3x30_read_message(struct mgc3x30_dev *dev, mgc3x30_msg_t *msg, uint8_t len)
{
    int rc;
    bool locked;

    rc = mgc3x30_lock(dev);
    locked = rc == 0;

    while (hal_gpio_read(dev->cfg.eio0_ts_pin)) {
        os_cputime_delay_usecs(200);
    }
    mgc3x30_ts_down(dev);

    rc = mgc3x30_i2c_read(dev, msg, len);

    mgc3x30_enable_ts_int(dev);

    if (locked) {
        mgc3x30_unlock(dev);
    }

    return rc;
}

int
mgc3x30_write_message(struct mgc3x30_dev *dev, mgc3x30_msg_t *msg)
{
    int rc;
    bool locked;

    rc = mgc3x30_lock(dev);
    locked = rc == 0;

    if (locked) {
        mgc3x30_ts_down(dev);
        rc = mgc3x30_i2c_write(dev, msg, msg->hdr.size);
        mgc3x30_enable_ts_int(dev);
    }
    if (locked) {
        mgc3x30_unlock(dev);
    }

    return rc;
}

static const uint8_t *
extract2(const uint8_t *p, uint16_t *data)
{
    *data = (p[1] << 8) | p[0];
    return p + 2;
}

static const uint8_t *
extract4(const uint8_t *p, uint32_t *data)
{
    *data = (p[3] << 24) | (p[2] << 16) | (p[1] << 8) | p[0];
    return p + 4;
}

static void
mgc3x30_sensor_data_output(struct mgc3x30_dev *dev, const struct mgc3x30_data_output *data_output)
{
    const uint8_t *p = data_output->data_buffer;
    uint16_t dsp_status;
    uint32_t gesture_info;
    uint32_t touch_info;
    uint16_t air_wheel_info = 0;
    uint16_t xyz[3] = {0};
    uint32_t noise_power;

    if (data_output->data_output_config_mask & DATA_OUTPUT_DSP_STATUS) {
        p = extract2(p, &dsp_status);
    }
    if (data_output->data_output_config_mask & DATA_OUTPUT_GESTURE_INFO) {
        p = extract4(p, &gesture_info);
    }
    if (data_output->data_output_config_mask & DATA_OUTPUT_TOUCH_INFO) {
        p = extract4(p, &touch_info);
    }
    if (data_output->data_output_config_mask & DATA_OUTPUT_AIR_WHEEL_INFO) {
        p = extract2(p, &air_wheel_info);
    }
    if (data_output->data_output_config_mask & DATA_OUTPUT_XYZ_POSITION) {
        p = extract2(p, &xyz[0]);
        p = extract2(p, &xyz[1]);
        p = extract2(p, &xyz[2]);
    }
    if (data_output->data_output_config_mask & DATA_OUTPUT_NOISE_POWER) {
        p = extract4(p, &noise_power);
    }
    if (data_output->data_output_config_mask & DATA_OUTPUT_GESTURE_INFO && dev->client->gesture) {
        dev->client->gesture(dev, gesture_info, xyz[0], xyz[1], xyz[2]);
    }
    if (data_output->data_output_config_mask & DATA_OUTPUT_TOUCH_INFO && dev->client->touch) {
        dev->client->touch(dev, touch_info, xyz[0], xyz[1], xyz[2]);
    }
    if (data_output->data_output_config_mask & DATA_OUTPUT_AIR_WHEEL_INFO &&
        dev->client->air_wheel && data_output->system_info & SYSTEM_INFO_AIR_WHEEL_VALID) {
        dev->client->air_wheel(dev, air_wheel_info);
    }
    if (data_output->data_output_config_mask & DATA_OUTPUT_NOISE_POWER &&
        dev->client->noise_power && data_output->system_info & SYSTEM_INFO_NOISE_POWER_VALID) {
        dev->client->air_wheel(dev, noise_power);
    }
    if (data_output->data_output_config_mask & DATA_OUTPUT_CIC_DATA) {
        if (dev->client->cic_data && data_output->system_info & SYSTEM_INFO_RAW_DATA_VALID) {
            dev->client->cic_data(dev, p);
        }
        p += 20;
    }
    if (data_output->data_output_config_mask & DATA_OUTPUT_SD_DATA) {
        if (dev->client->sd_data && data_output->system_info & SYSTEM_INFO_RAW_DATA_VALID) {
            dev->client->sd_data(dev, p);
        }
        p += 20;
    }
    if ((p - (uint8_t *)data_output) != data_output->hdr.size) {
        console_printf("MGC3x30 data mismatch\n");
        const uint8_t *pp = (const uint8_t *)&data_output;
        for (int i = 0; i < data_output->hdr.size; i += 16) {
            for (int j = 0; j < 16 && i + j < data_output->hdr.size; ++j, ++pp) {
                console_printf("%02x ", *pp);
            }
            console_out('\n');
        }
    }
}

static void
mgc3x30_isr_cb(struct mgc3x30_dev *dev)
{
    int rc;
    mgc3x30_msg_t msg;

    rc = mgc3x30_lock(dev);
    if (rc == 0) {
        if (hal_gpio_read(dev->cfg.eio0_ts_pin) == 0)
        {
            mgc3x30_read_message(dev, &msg, dev->expected_msg_size);
            switch(msg.hdr.id) {
            case MGC3X30_MSG_SYSTEM_STATUS:
                break;
            case MGC3X30_MSG_FW_VERSION_INFO:
                dev->ready = true;
                if (dev->client && dev->client->ready) {
                    dev->client->ready(dev, &msg.fw_version);
                }
                dev->expected_msg_size = dev->data_out_size;
                break;
            case MGC3X30_MSG_SENSOR_DATA_OUTPUT:
                mgc3x30_sensor_data_output(dev, &msg.data_output);
                break;
            }
        }
        mgc3x30_unlock(dev);
    }
}

static void
mgc3x30_isr_event_cb(struct os_event *event)
{
    mgc3x30_isr_cb((struct mgc3x30_dev *)event->ev_arg);
}

void
mgc3x30_reset(struct mgc3x30_dev *dev)
{
    if (dev->cfg.mclr_pin >= 0) {
        hal_gpio_write(dev->cfg.mclr_pin, 0);
        mgc3x30_enable_ts_int(dev);
        dev->expected_msg_size = 0x84;
        dev->reset_done = true;
        dev->ready = false;
        os_cputime_delay_usecs(1000);
        hal_gpio_write(dev->cfg.mclr_pin, 1);
        os_cputime_delay_usecs(3000);
    }
}

int
mgc3x30_get_runtime_parameter(struct mgc3x30_dev *dev, uint16_t paramter_id, uint32_t *arg0,
                              uint32_t *arg1)
{
    int rc;
    struct mgc3x30_request msg = {
        .hdr.size = sizeof(msg),
        .hdr.id = MGC3X30_MSG_REQUEST_MESSAGE,
        .hdr.seq = dev->seq++,
        .message_id = MGC3X30_MSG_SET_RUNTIME_PARAMETER,
        .parameter = htole32(paramter_id),
    };
    union {
        struct mgc3x30_msg_header hdr;
        struct mgc3x30_set_runtime_parameter param;
        struct mgc3x30_system_status status;
    } result;

    rc = mgc3x30_write_message(dev, (mgc3x30_msg_t *)&msg);
    if (rc == 0) {
        rc = mgc3x30_read_message(dev, (mgc3x30_msg_t *)&result, sizeof(result));
        if (rc == 0) {
            if (result.hdr.size == sizeof(result.param) &&
                result.param.hdr.id == MGC3X30_MSG_SET_RUNTIME_PARAMETER) {
                *arg0 = result.param.arg0;
                *arg1 = result.param.arg1;
                (void)mgc3x30_read_message(dev, (mgc3x30_msg_t *)&result, sizeof(result.status));
            } else if (result.hdr.size == sizeof(result.status) &&
                       result.status.hdr.id == MGC3X30_MSG_SYSTEM_STATUS) {
                rc = -10000 - result.status.error_code;
            }
        }
    }
    return rc;
}

int
mgc3x30_get_fw_info(struct mgc3x30_dev *dev, const struct mgc3x30_fw_version_info *info)
{
    int rc;
    struct mgc3x30_request msg = {
        .hdr.size = sizeof(msg),
        .hdr.id = MGC3X30_MSG_REQUEST_MESSAGE,
        .hdr.seq = dev->seq++,
        .message_id = MGC3X30_MSG_FW_VERSION_INFO,
    };
    struct mgc3x30_system_status status;

    rc = mgc3x30_write_message(dev, (mgc3x30_msg_t *)&msg);
    if (rc == 0) {
        rc = mgc3x30_read_message(dev, (mgc3x30_msg_t *)&status, sizeof(status));
        if (rc == 0 && status.hdr.size == sizeof(status) && status.hdr.id == MGC3X30_MSG_SYSTEM_STATUS &&
            status.id == MGC3X30_MSG_FW_VERSION_INFO && status.error_code == 0) {
            rc = mgc3x30_read_message(dev, (mgc3x30_msg_t *)info, sizeof(*info));
            if (rc == 0) {
                if (info->hdr.size != 0x10 ||
                    info->hdr.id != MGC3X30_MSG_FW_VERSION_INFO)
                    rc = -1;
            }
        }
    }
    return rc;
}

int
mgc3x30_set_runtime_parameter(struct mgc3x30_dev *dev, uint16_t paramter_id, uint32_t arg0, uint32_t arg1)
{
    int rc;
    struct mgc3x30_system_status status;
    struct mgc3x30_set_runtime_parameter msg = {
        .hdr.size = sizeof(msg),
        .hdr.id = MGC3X30_MSG_SET_RUNTIME_PARAMETER,
        .runtime_paramter_id = paramter_id,
        .arg0 = arg0,
        .arg1 = arg1,
    };

    rc = mgc3x30_write_message(dev, (mgc3x30_msg_t *)&msg);
    if (rc) {
        rc = mgc3x30_read_message(dev, (mgc3x30_msg_t *)&status, sizeof(status));
        if (rc == 0) {
            if (status.hdr.size != 0x10 ||
                status.hdr.id != MGC3X30_MSG_SET_RUNTIME_PARAMETER ||
                status.error_code != 0) {
                    rc = -1;
            }
        }
    }
    return rc;
}

int
mgc3x30_enable_approach_detection(struct mgc3x30_dev *dev, bool enable)
{
    return mgc3x30_set_runtime_parameter(dev, MGC3X30_PARAM_APPROACH_DETECTION,
                                         enable ? MGC3X30_APPROACH_DETECTION_MASK : 0,
                                         MGC3X30_APPROACH_DETECTION_MASK);
}

int
mgc3x30_enable_touch_detection(struct mgc3x30_dev *dev, bool enable)
{
    return mgc3x30_set_runtime_parameter(dev, MGC3X30_PARAM_TOUCH_DETECTION,
                                         enable ? MGC3X30_TOUCH_DETECTION_MASK : 0,
                                         MGC3X30_TOUCH_DETECTION_MASK);
}

int
mgc3x30_set_client(struct mgc3x30_dev *dev, const struct mgc3x30_client *client)
{
    dev->client = client;

    return 0;
}

static void
mgc3x30_init_int(struct mgc3x30_dev *dev)
{
}

static int
mgc3x30_dev_open(struct mgc3x30_dev *dev)
{
    if (dev->cfg.mclr_pin >= 0 && !dev->reset_done) {
        mgc3x30_reset(dev);
    } else {
        mgc3x30_enable_ts_int(dev);
    }

    return 0;
}

static void
mgc3x30_dev_close(struct mgc3x30_dev *dev)
{
    hal_gpio_irq_disable(dev->cfg.eio0_ts_pin);
    hal_gpio_deinit(dev->cfg.eio0_ts_pin);
    mgc3x30_set_client(dev, NULL);
}

struct mgc3x30_dev *
mgc3x30_open(const char *name, const struct mgc3x30_client *client)
{
    struct mgc3x30_dev *dev = (struct mgc3x30_dev *)os_dev_open(name, 1000, NULL);
    if (dev) {
        mgc3x30_set_client(dev, client);
        if (dev->ready && client != NULL && client->ready) {
            client->ready(dev, NULL);
        }
    }

    return dev;
}

#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
static void
mgc3x30_init_node_cb(struct bus_node *node, void *arg)
{
    struct mgc3x30_dev *dev = (struct mgc3x30_dev *)node;

    (void)arg;

    mgc3x30_init_int(dev);
}

void mgc3x30_node_open(struct bus_node *node)
{
    mgc3x30_dev_open((struct mgc3x30_dev *)node);
}

void mgc3x30_node_close(struct bus_node *node)
{
    mgc3x30_dev_close((struct mgc3x30_dev *)node);
}

int
mgc3x30_dev_create(struct mgc3x30_dev *dev, const char *name,
                   const struct mgc3x30_cfg *cfg)
{
    struct bus_node_callbacks cbs = {
        .init = mgc3x30_init_node_cb,
        .open = mgc3x30_node_open,
        .close = mgc3x30_node_close,
    };
    int rc;

    dev->cfg = *cfg;
    dev->event_queue = os_eventq_dflt_get();
    dev->irq_event.ev_cb = mgc3x30_isr_event_cb;
    dev->irq_event.ev_arg = dev;
    os_mutex_init(&dev->lock);
    dev->data_out_size = 26;
    if (dev->cfg.mclr_pin >= 0) {
        hal_gpio_init_out(dev->cfg.mclr_pin, 1);
    }

    bus_node_set_callbacks((struct os_dev *)dev, &cbs);

    rc = bus_i2c_node_create(name, &dev->dev, &cfg->node_cfg, NULL);

    return rc;
}

#else

static int
mgc3x30_open_handler(struct os_dev *odev, uint32_t timeout, void *arg)
{
    (void)timeout;
    (void)arg;

    return mgc3x30_dev_open((struct mgc3x30_dev *)odev);
}

static int
mgc3x30_close_handler(struct os_dev *odev)
{
    mgc3x30_dev_close((struct mgc3x30_dev *)odev);

    return 0;
}

static int
mgc3x30_init_func(struct os_dev *odev, void *arg)
{
    struct mgc3x30_dev *dev = (struct mgc3x30_dev *)odev;

    (void)arg;

    OS_DEV_SETHANDLERS(odev, mgc3x30_open_handler, mgc3x30_close_handler);

    mgc3x30_init_int(dev);

    return 0;
}

int
mgc3x30_dev_create(struct mgc3x30_dev *dev, const char *name,
                   const struct mgc3x30_cfg *cfg)
{
    int rc;

    dev->cfg = *cfg;
    assert(cfg->eio0_ts_pin >= 0);
    hal_gpio_init_in(cfg->eio0_ts_pin, 0);
    dev->event_queue = os_eventq_dflt_get();
    dev->irq_event.ev_cb = mgc3x30_isr_event_cb;
    dev->irq_event.ev_arg = dev;
    os_mutex_init(&dev->lock);

    rc = os_dev_create(&dev->dev, name, OS_DEV_INIT_SECONDARY, 0, mgc3x30_init_func, NULL);

    return rc;
}


#endif
