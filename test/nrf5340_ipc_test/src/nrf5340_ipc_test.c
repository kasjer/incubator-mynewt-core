
#include <os/mynewt.h>
#include <os/os_time.h>
#include <os/os_task.h>
#include <console/console.h>
#include <shell/shell.h>
#include <ipc_nrf5340/ipc_nrf5340.h>
#include <hal/hal_gpio.h>
#include <bsp/bsp.h>

uint8_t app_receive_buf[256];
uint16_t app_len;
uint16_t app_p;
struct os_sem done;

uint8_t app_write_buf[256];
uint8_t net_write_buf[256];

struct os_callout ipc_callout;

void
net_rcv(int channel, void *user_data)
{
    uint32_t n = os_time_get() & 63;
    uint32_t p = 0;
    uint32_t m;
    if (n == 0) { n = 2; }

    while (ipc_nrf5340_available(channel) > 0) {
        p += ipc_nrf5340_read(channel, net_write_buf + p, n);
    }
    m = p;
    p = 0;
    n += 2;
    while (p < m) {
        if (n > (m - p)) {
            n = m - p;
        }
        ipc_nrf5340_send(channel + 1, net_write_buf + p, n);
        p += n;
    }
}

void
app_rcv(int channel, void *user_data)
{
    uint32_t n = os_time_get() & 63;
    if (n == 0) { n = 2; }

    while (ipc_nrf5340_available(channel) > 0) {
        app_p += ipc_nrf5340_read(channel, app_receive_buf + app_p, n);
    }

    if (app_p == app_len) {
        hal_gpio_write(ARDUINO_PIN_D11, 1);
        app_p = 0;
        os_sem_release(&done);
        hal_gpio_write(ARDUINO_PIN_D11, 0);
    } if (app_p > app_len) {
        HAL_DEBUG_BREAK();
    }
}

void
nrf5340_ipc_net_core_init(void)
{
    ipc_nrf5340_recv(2, net_rcv, NULL);
}

static int
ipc_test(os_time_t ticks)
{
    int i;
    os_time_t limit = os_time_get() + ticks;
    uint32_t v = 1;
    int n, p;
    os_sem_init(&done, 0);

    hal_gpio_init_out(ARDUINO_PIN_D8, 0);
    hal_gpio_init_out(ARDUINO_PIN_D9, 0);

    while (limit > os_time_get()) {
        app_len = ((os_time_get() & 31) + 1) << 2;

        for (i = 0; i < app_len; ++i) {
            app_write_buf[i] = (uint8_t)v;
            ++v;
        }
        n = (app_write_buf[0] & 31) + 1;
        p = 0;
        app_p = 0;
        while (p < app_len) {
            if (n > app_len - p) n = app_len - p;
            ipc_nrf5340_send(2, app_write_buf + p, n);
            p += n;
        }
        hal_gpio_write(ARDUINO_PIN_D8, 1);
        if (os_sem_pend(&done, OS_TICKS_PER_SEC)) {
            HAL_DEBUG_BREAK();
            hal_gpio_write(ARDUINO_PIN_D8, 0);
            console_printf("Timeout\n");
            return -1;
        }
        hal_gpio_write(ARDUINO_PIN_D8, 0);
        if (memcmp(app_write_buf, app_receive_buf, app_len) != 0) {
            HAL_DEBUG_BREAK();
            console_printf("Data comparison failed\n");
            return -2;
        }
    }
    return 0;
}

static int
ipctest_cmd(int argc, char **argv)
{
    console_printf("Starting ipctest\n");

    if (0 == ipc_test(OS_TICKS_PER_SEC * 5)) {
        console_printf("Test ok\n");
    }

    return 0;
}

void ipc_callout_cb(struct os_event *e)
{
    ipc_test(OS_TICKS_PER_SEC);
    os_callout_reset(&ipc_callout, OS_TICKS_PER_SEC + 10);
}

static int
calloutipc_cmd(int argc, char **argv)
{
    if (os_callout_queued(&ipc_callout)) {
        console_printf("Stopping IPC test in callout\n");
        os_callout_stop(&ipc_callout);
    } else {
        console_printf("Starting IPC test in callout\n");
        os_callout_reset(&ipc_callout, OS_TICKS_PER_SEC);
    }
    return 0;
}

static const struct shell_cmd ipctest_cmd_struct = {
    .sc_cmd = "ipctest",
    .sc_cmd_func = ipctest_cmd
};

static const struct shell_cmd calloutipc_cmd_struct = {
    .sc_cmd = "calloutipc",
    .sc_cmd_func = calloutipc_cmd
};

void
nrf5340_ipc_app_core_init(void)
{
    ipc_nrf5340_recv(3, app_rcv, NULL);
    os_callout_init(&ipc_callout, os_eventq_dflt_get(), ipc_callout_cb, NULL);
    shell_cmd_register(&ipctest_cmd_struct);
    shell_cmd_register(&calloutipc_cmd_struct);
}
