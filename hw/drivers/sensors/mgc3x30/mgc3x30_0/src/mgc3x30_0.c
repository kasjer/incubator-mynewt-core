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
#include <os/mynewt.h>
#include <bsp/bsp.h>
#include <mgc3x30/mgc3x30.h>

struct mgc3x30_dev mgc3x30_0;

struct mgc3x30_cfg mgc3x30_0_cfg = {
#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    .node_cfg = {
        .node_cfg.lock_timeout_ms = MYNEWT_VAL(MGC3X30_0_LOCK_TIMEOUT),
        .node_cfg.bus_name = MYNEWT_VAL(MGC3X30_0_I2C_BUS),
        .addr = MYNEWT_VAL(MGC3X30_0_I2C_ADDR),
        .freq = MYNEWT_VAL(MGC3X30_0_I2C_BAUDRATE),
    },
#else
    .i2c_settings = {
        .frequency = MYNEWT_VAL(MGC3X30_0_I2C_BAUDRATE),
    },
    .i2c_num = MYNEWT_VAL(MGC3X30_0_I2C_NUM),
    .i2c_addr = MYNEWT_VAL(MGC3X30_0_I2C_ADDR),
#endif
    .eio0_ts_pin = MYNEWT_VAL(MGC3X30_0_EIO0_TS_PIN),
    .eio1_pin = MYNEWT_VAL(MGC3X30_0_EIO1_PIN),
    .eio2_pin = MYNEWT_VAL(MGC3X30_0_EIO2_PIN),
    .eio3_pin = MYNEWT_VAL(MGC3X30_0_EIO3_PIN),
    .eio4_pin = MYNEWT_VAL(MGC3X30_0_EIO4_PIN),
    .mclr_pin = MYNEWT_VAL(MGC3X30_0_MCLR_PIN),
    .north_rx = MYNEWT_VAL(MGC3X30_0_NORTH_RX),
    .south_rx = MYNEWT_VAL(MGC3X30_0_SOUTH_RX),
    .west_rx = MYNEWT_VAL(MGC3X30_0_WEST_RX),
    .east_rx = MYNEWT_VAL(MGC3X30_0_EAST_RX),
    .center_rx = MYNEWT_VAL(MGC3X30_0_CENTER_RX),
};

void
mgc3x30_0_init(void)
{
    int rc;

    rc = mgc3x30_dev_create(&mgc3x30_0,
                            MYNEWT_VAL(MGC3X30_0_NAME),
                            &mgc3x30_0_cfg);

    assert(rc == 0);
}
