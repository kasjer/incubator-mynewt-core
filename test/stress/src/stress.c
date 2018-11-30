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

#include <inttypes.h>
#include "os/mynewt.h"
#include <shell/shell.h>
#include <console/console.h>
#include <parse/parse.h>
#include <bsp/bsp.h>
#include <errno.h>

static volatile bool busy_loop = MYNEWT_VAL(STRESS_TASK_DEFAULT_BUSY);
struct os_sem stress_semaphore;

static os_stack_t stress_task_stack[MYNEWT_VAL(STRESS_TASK_STACK_SIZE)] __attribute__ ((aligned (8)));
static struct os_task stress_task;

static int stress_cli_cmd(int argc, char **argv);

static struct shell_cmd stress_cmd_struct = {
    .sc_cmd = "stress",
    .sc_cmd_func = stress_cli_cmd
};

#include "adc/adc.h"

extern nrfx_saadc_config_t bsp_adc0_cfg;

const nrf_saadc_channel_config_t bsp_adc0_chan_cfg_vdd =
{
    .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
    .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
    .gain = NRF_SAADC_GAIN1_6,
    .reference = NRF_SAADC_REFERENCE_INTERNAL,
    .acq_time = NRF_SAADC_ACQTIME_5US,
    .mode = NRF_SAADC_MODE_SINGLE_ENDED,
    .burst = NRF_SAADC_BURST_ENABLED,
    .pin_p = NRF_SAADC_INPUT_VDD,
    .pin_n = NRF_SAADC_INPUT_DISABLED,
};

int bat_val;
int vdd_val;
static int
stress_cli_cmd(int argc, char **argv)
{
    if (argc < 2) {
        return 0;
    }
    if (strcmp(argv[1], "on") == 0) {
        busy_loop = true;
        os_sem_release(&stress_semaphore);
    } else if (strcmp(argv[1], "off") == 0) {
        busy_loop = false;
        os_sem_release(&stress_semaphore);
    } else if (strcmp(argv[1], "test") == 0) {
        struct adc_dev *adc = (struct adc_dev *)os_dev_open("adc0", 1000, &bsp_adc0_cfg);
        if (adc) {
            vote_meas_en_high(0x10);
            bat_val = 0x1234567;
            adc_chan_config(adc, 5, (nrf_saadc_channel_config_t *)&bsp_adc0_chan_cfg_battery);
            adc_chan_read(adc, 5, &bat_val);
            if (bat_val == 0x1234567) {
                console_printf("adc did not work\n");
            } else {
                bat_val = adc_result_mv(adc, 5, bat_val) *
                                    36 / 20;
                console_printf("VBat = %d\n", bat_val);
            }
            vote_meas_en_low(0x10);
            os_dev_close(&adc->ad_dev);
        }
        adc = (struct adc_dev *)os_dev_open("adc0", 1000, &bsp_adc0_cfg);
        if (adc) {
            vdd_val = 0x1234567;
            adc_chan_config(adc, 6, (nrf_saadc_channel_config_t *)&bsp_adc0_chan_cfg_vdd);
            adc_chan_read(adc, 6, &vdd_val);
            if (vdd_val == 0x1234567) {
                console_printf("adc did not work\n");
            } else {
                vdd_val = adc_result_mv(adc, 6, vdd_val);
                console_printf("VDD = %d\n", vdd_val);
            }
            os_dev_close(&adc->ad_dev);
        }
    }
    return 0;
}

static void
stress_task_f(void *arg)
{
    volatile int i = 0;

    /* Main log handling loop */
    while (1)
    {
        os_time_t limit = os_time_get() + os_time_ms_to_ticks32(1000);
        while (busy_loop && limit > os_time_get()) {
            i = i * 4 / 3;
            i++;
        }
        if (busy_loop) {
            /* Release a bit */
            os_sem_pend(&stress_semaphore, 3);
        } else {
            os_sem_pend(&stress_semaphore, OS_TIMEOUT_NEVER);
        }
    }
}

void
stress_init(void)
{
#if MYNEWT_VAL(STRESS_TASK_ENABLE)
    int rc;

    os_sem_init(&stress_semaphore, 0);

    rc = os_task_init(&stress_task, "stress", stress_task_f, NULL,
                      MYNEWT_VAL(STRESS_TASK_PRIORITY), OS_WAIT_FOREVER,
                      stress_task_stack, MYNEWT_VAL(STRESS_TASK_STACK_SIZE));
    assert(rc == 0);
    shell_cmd_register(&stress_cmd_struct);
#endif
}
