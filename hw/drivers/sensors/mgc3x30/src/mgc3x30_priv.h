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

#ifndef __DRIVERS_MAX3107_PRIV_H_
#define __DRIVERS_MAX3107_PRIV_H_

#include <stdint.h>
#include <uart/uart.h>
#include <hal/hal_spi.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MGC3X30_MSG_SYSTEM_STATUS           0x15
#define MGC3X30_MSG_REQUEST_MESSAGE         0x06
#define MGC3X30_MSG_FW_VERSION_INFO         0x83
#define MGC3X30_MSG_SET_RUNTIME_PARAMETER   0xA2
    
#define MGC3X30_MSG_SENSOR_DATA_OUTPUT      0x91

#define MGC3X30_MSG_FW_UPDATE_START         0x80
#define MGC3X30_MSG_FW_UPDATE_BLOCK         0x81
#define MGC3X30_MSG_FW_UPDATE_COMPLETED     0x82

#define MGC3X30_PARAM_TRIGGER                       0x1000
#define MGC3X30_PARAM_MAKE_PERSISTENT               0xFF00
#define MGC3X30_PARAM_AFE_RX_ATT_S                  0x0050
#define MGC3X30_PARAM_AFE_RX_ATT_W                  0x0051
#define MGC3X30_PARAM_AFE_RX_ATT_N                  0x0052
#define MGC3X30_PARAM_AFE_RX_ATT_E                  0x0053
#define MGC3X30_PARAM_AFE_RX_ATT_C                  0x0054
#define MGC3X30_PARAM_CH_MAP_S                      0x0065
#define MGC3X30_PARAM_CH_MAP_W                      0x0066
#define MGC3X30_PARAM_CH_MAP_N                      0x0067
#define MGC3X30_PARAM_CH_MAP_E                      0x0068
#define MGC3X30_PARAM_CH_MAP_C                      0x0069
#define MGC3X30_PARAM_TX_FREQ_SEL                   0x0082
#define MGC3X30_PARAM_TOUCH_DETECTION               0x0097
#define MGC3X30_PARAM_APPROACH_DETECTION            0x0097
#define MGC3X30_PARAM_AIR_WHEEL                     0x0090
#define MGC3X30_PARAM_GESTURE_PROCESSING            0x0085
#define MGC3X30_PARAM_CALIBRATION                   0x0080
#define MGC3X30_PARAM_DATA_OUTPUT                   0x00A0
#define MGC3X30_PARAM_DATA_OUTPUT_LOCK_MASK         0x00A1
#define MGC3X30_PARAM_DATA_OUTPUT_REQ_MASK          0x00A2

#define MGC3X30_TOUCH_DETECTION_MASK                0x0008
#define MGC3X30_APPROACH_DETECTION_MASK             0x0001
#define MGC3X30_AIR_WHEEL_DETECTION_MASK            0x0020

#define MGC3X30_GESTURE_MASK_GARBAGE_MODEL          (1u << 0)
#define MGC3X30_GESTURE_MASK_FLICK_W_2_E            (1u << 1)
#define MGC3X30_GESTURE_MASK_FLICK_E_2_W            (1u << 2)
#define MGC3X30_GESTURE_MASK_FLICK_S_2_N            (1u << 3)
#define MGC3X30_GESTURE_MASK_FLICK_N_2_S            (1u << 4)
#define MGC3X30_GESTURE_MASK_CIRCLE                 (1u << 5)
#define MGC3X30_GESTURE_MASK_CIRCLE_BACK            (1u << 6)
#define MGC3X30_GESTURE_MASK_WAVE_X                 (1u << 7)
#define MGC3X30_GESTURE_MASK_WAVE_Y                 (1u << 8)
#define MGC3X30_GESTURE_MASK_HOLD                   (1u << 22)
#define MGC3X30_GESTURE_MASK_PRESENCE               (1u << 23)
#define MGC3X30_GESTURE_MASK_EDGE_FLICK_W_2_E       (1u << 24)
#define MGC3X30_GESTURE_MASK_EDGE_FLICK_E_2_W       (1u << 25)
#define MGC3X30_GESTURE_MASK_EDGE_FLICK_S_2_N       (1u << 26)
#define MGC3X30_GESTURE_MASK_EDGE_FLICK_N_2_S       (1u << 27)
#define MGC3X30_GESTURE_MASK_DOUBLE_FLICK_W_2_E     (1u << 28)
#define MGC3X30_GESTURE_MASK_DOUBLE_FLICK_E_2_W     (1u << 29)
#define MGC3X30_GESTURE_MASK_DOUBLE_FLICK_S_2_N     (1u << 30)
#define MGC3X30_GESTURE_MASK_DOUBLE_FLICK_N_2_S     (1u << 31)

#define MGC3X30_TX_FREQ_115_KHZ                     0
#define MGC3X30_TX_FREQ_103_KHZ                     1
#define MGC3X30_TX_FREQ_88_KHZ                      2
#define MGC3X30_TX_FREQ_67_KHZ                      3
#define MGC3X30_TX_FREQ_44_KHZ                      4


#define SYSTEM_INFO_POSITION_VALID                  0x01
#define SYSTEM_INFO_AIR_WHEEL_VALID                 0x02
#define SYSTEM_INFO_RAW_DATA_VALID                  0x04
#define SYSTEM_INFO_NOISE_POWER_VALID               0x08
#define SYSTEM_INFO_ENVIRONMENTAL_NOISE             0x10
#define SYSTEM_INFO_CLIPPING                        0x20
#define SYSTEM_INFO_DSP_RUNNING                     0x80

#define DATA_OUTPUT_DSP_STATUS                      0x0001
#define DATA_OUTPUT_GESTURE_INFO                    0x0002
#define DATA_OUTPUT_TOUCH_INFO                      0x0004
#define DATA_OUTPUT_AIR_WHEEL_INFO                  0x0008
#define DATA_OUTPUT_XYZ_POSITION                    0x0010
#define DATA_OUTPUT_NOISE_POWER                     0x0020
#define DATA_OUTPUT_CIC_DATA                        0x0800
#define DATA_OUTPUT_SD_DATA                         0x1000

#define GESTURE_NO_GESTURE                          0
#define GESTURE_GARBAGE_MODEL                       1
#define GESTURE_FLICK_WEST_TO_EAST                  2
#define GESTURE_FLICK_EAST_TO_WEST                  3
#define GESTURE_FLICK_SOUTH_TO_NORTH                4
#define GESTURE_FLICK_NORTH_TO_SOUTH                5
#define GESTURE_CIRCLE_CLOCKWISE                    6
#define GESTURE_CIRCLE_COUNTER_CLOCKWISE            7
#define GESTURE_WAVE_X                              8
#define GESTURE_WAVE_Y                              9
#define GESTURE_HOLD                                64
#define GESTURE_PRESENCE                            73
#define GESTURE_EDGE_FLICK_WEST_TO_EAST             65
#define GESTURE_EDGE_FLICK_EAST_TO_WEST             66
#define GESTURE_EDGE_FLICK_SOUTH_TO_NORTH           67
#define GESTURE_EDGE_FLICK_NORTH_TO_SOUTH           68
#define GESTURE_DOUBLE_FLICK_WEST_TO_EAST           69
#define GESTURE_DOUBLE_FLICK_EAST_TO_WEST           70
#define GESTURE_DOUBLE_FLICK_SOUTH_TO_NORTH         71
#define GESTURE_DOUBLE_FLICK_NORTH_TO_SOUTH         72

#define GESTURE_CLASS_GARBAGE_MODEL                 (0u << 12)
#define GESTURE_CLASS_FLICK_GESTURE                 (1u << 12)
#define GESTURE_CLASS_CIRCULAR_GESTURE              (2u << 12)

#define GESTURE_EDGE_FLICK                          (1u << 16)

#define GESTURE_HAND_PRESENCE                       (1u << 27)
#define GESTURE_HAND_HOLD                           (1u << 28)
#define GESTURE_HAND_INSIDE                         (1u << 29)

#define GESTURE_RECOGNITION_IN_PROGRESS     (1u << 31)

struct mgc3x30_msg_header {
    uint8_t size;
    uint8_t flags;
    uint8_t seq;
    uint8_t id;
} __attribute__((packed));

struct mgc3x30_request {
    struct mgc3x30_msg_header hdr;
    uint8_t message_id;
    uint8_t reserved[3];
    uint32_t parameter;
} __attribute__((packed));

struct mgc3x30_fw_version_info {
    struct mgc3x30_msg_header hdr;
    uint8_t fw_valid;
    uint16_t hw_rev;
    uint8_t parameter_start_addr;
    uint16_t library_loader_version;
    uint8_t library_loader_platform;
    uint8_t fw_start_addr;
    uint8_t fw_version[120];
} __attribute__((packed));

struct mgc3x30_set_runtime_parameter {
    struct mgc3x30_msg_header hdr;
    uint16_t runtime_paramter_id;
    uint16_t reserved;
    uint32_t arg0;
    uint32_t arg1;
} __attribute__((packed));

struct mgc3x30_system_status {
    struct mgc3x30_msg_header hdr;
    uint8_t id;
    uint8_t max_cmd_size;
    uint16_t error_code;
    uint32_t reserved0;
    uint32_t reserved1;
} __attribute__((packed));

struct mgc3x30_data_output {
    struct mgc3x30_msg_header hdr;
    uint16_t data_output_config_mask;
    uint8_t time_stamp;
    uint8_t system_info;
    uint8_t data_buffer[248];
} __attribute__((packed));

typedef union mgc3x30_msg {
    struct mgc3x30_msg_header hdr;
    struct mgc3x30_request request;
    struct mgc3x30_fw_version_info fw_version;
    struct mgc3x30_set_runtime_parameter set_runtime_parameter;
    struct mgc3x30_data_output data_output;
} mgc3x30_msg_t;

struct mgc3x30_dev {
#if MYNEWT_VAL(BUS_DRIVER_PRESENT)
    struct bus_i2c_node dev;
#else
    struct os_dev dev;
#endif
    struct os_mutex lock;

    struct mgc3x30_cfg cfg;

    const struct mgc3x30_client *client;

    bool irq_pending;
    bool reset_done;
    bool ready;
    uint8_t expected_msg_size;
    uint8_t data_out_size;
    uint8_t seq;
    struct os_event irq_event;
    struct os_eventq *event_queue;
};

#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_MAX3107_PRIV_H_ */
