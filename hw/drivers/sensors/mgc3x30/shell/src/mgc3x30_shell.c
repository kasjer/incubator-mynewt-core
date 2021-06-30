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

#include <os/mynewt.h>

#include <console/console.h>
#include <shell/shell.h>
#include <parse/parse.h>
#include <stdio.h>
#include <mcu/mcu.h>
#include <mgc3x30/mgc3x30.h>

static struct mgc3x30_dev *mgc3x30_dev;
static struct streamer *shell_stream;
static uint32_t last_gesture = 123123;
static uint32_t last_touch = 123123;

#if MYNEWT_VAL(SHELL_CMD_HELP)
#define HELP(a) &(mgc3x30_##a##_help)
static const struct shell_param mgc3x30_open_param_help = {
    .param_name = "<device>",
    .help = "device to open",
};
static const struct shell_cmd_help mgc3x30_open_help = {
    .summary = "Opens device",
    .usage = "open [name]",
    .params = &mgc3x30_open_param_help,
};

static const struct shell_cmd_help mgc3x30_close_help = {
    .summary = "Closes device",
    .usage = NULL,
    .params = NULL,
};

static const struct shell_cmd_help mgc3x30_reset_help = {
    .summary = "Reset device with MCLR line",
    .usage = NULL,
    .params = NULL,
};

static const struct shell_cmd_help mgc3x30_get_help = {
    .summary = "Get ...",
    .usage = NULL,
    .params = NULL,
};

#else
#define HELP(a) NULL
#endif

static const char *gesture_str(uint32_t gesture)
{
    switch (gesture & 0xFF) {
    case GESTURE_NO_GESTURE:
        return "No gesture";
    case GESTURE_GARBAGE_MODEL:
        return "Garbage model";
    case GESTURE_FLICK_WEST_TO_EAST:
        return "flick west to east";
    case GESTURE_FLICK_EAST_TO_WEST:
        return "flick east to west";
    case GESTURE_FLICK_SOUTH_TO_NORTH:
        return "flick south to north";
    case GESTURE_FLICK_NORTH_TO_SOUTH:
        return "flock north to south";
    case GESTURE_CIRCLE_CLOCKWISE:
        return "circle";
    case GESTURE_CIRCLE_COUNTER_CLOCKWISE:
        return "back circle";
    case GESTURE_WAVE_X:
        return "wave x";
    case GESTURE_WAVE_Y:
        return "wave y";
    case GESTURE_HOLD:
        return "hold";
    case GESTURE_PRESENCE:
        return "presence";
    case GESTURE_EDGE_FLICK_WEST_TO_EAST:
        return "edge flick west to east";
    case GESTURE_EDGE_FLICK_EAST_TO_WEST:
        return "edge flick east to west";
    case GESTURE_EDGE_FLICK_SOUTH_TO_NORTH:
        return "edge flick south to north";
    case GESTURE_EDGE_FLICK_NORTH_TO_SOUTH:
        return "edge flick north to south";
    case GESTURE_DOUBLE_FLICK_WEST_TO_EAST:
        return "double flick west to east";;
    case GESTURE_DOUBLE_FLICK_EAST_TO_WEST:
        return "double flick east to west";
    case GESTURE_DOUBLE_FLICK_SOUTH_TO_NORTH:
        return "double flick south to north";
    case GESTURE_DOUBLE_FLICK_NORTH_TO_SOUTH:
        return "double flick north to south";
    default:
        return "???";
    }
}

static const char *
gesture_class_str(uint32_t gesture)
{
    switch (gesture & (3u << 12)) {
    case GESTURE_CLASS_GARBAGE_MODEL:
        return ", garbage model";
    case GESTURE_CLASS_FLICK_GESTURE:
        return ", flick gesture";
    case GESTURE_CLASS_CIRCULAR_GESTURE:
        return ", circular gesture";
    default:
        return ", ???";
    }
}

static const char *
gesture_edge_flick(uint32_t gesture)
{
    return (gesture & GESTURE_EDGE_FLICK) ? ", edge flick " : "";
}

static const char *
gesture_hand_presence_str(uint32_t gesture)
{
    return (gesture & GESTURE_HAND_PRESENCE) ? ", hand presence " : "";
}

static const char *
gesture_hand_hold_str(uint32_t gesture)
{
    return (gesture & GESTURE_HAND_HOLD) ? ", hand hold" : "";
}

static const char *
gesture_hand_inside_str(uint32_t gesture)
{
    return (gesture & GESTURE_HAND_INSIDE) ? ", hand inside" : "";
}

static const char *touch_str[] =
{
    "touch S",
    "touch W",
    "touch N",
    "touch E",
    "touch C",
    "tap S",
    "tap W",
    "tap N",
    "tap E",
    "tap C",
    "double tap S",
    "double tap W",
    "double tap N",
    "double tap E",
    "double tap C",
};

static void
mgc_gesture(struct mgc3x30_dev *dev, uint32_t gesture, uint16_t x, uint16_t y, uint16_t z)
{
    if (gesture != last_gesture) {
        if (gesture != GESTURE_NO_GESTURE && gesture != GESTURE_GARBAGE_MODEL) {
            console_printf("gesture 0x%08" PRIx32 " %s%s%s%s%s%s (%d, %d, %d)\n", gesture,
                           gesture_str(gesture), gesture_class_str(gesture),
                           gesture_edge_flick(gesture), gesture_hand_presence_str(gesture),
                           gesture_hand_hold_str(gesture), gesture_hand_inside_str(gesture),
                           x, y, z);
        }
        last_gesture = gesture;
    }
}

static void
mgc_touch(struct mgc3x30_dev *dev, uint32_t touch, uint16_t x, uint16_t y, uint16_t z)
{
    int i;

    if (touch != last_touch) {
        streamer_printf(shell_stream, "touch 0x%08" PRIx32, touch);
        for (i = 0; i < 15; ++i) {
            if (touch & (1 << i)) {
                streamer_printf(shell_stream, ", %s", touch_str[i]);
            }
        }
        streamer_printf(shell_stream, ", (%d, %d, %d)\n", x, y, z);
        last_touch = touch;
    }
}

static void
mgc_ready(struct mgc3x30_dev *dev, const struct mgc3x30_fw_version_info *info)
{
    console_printf("MGC3130 ready %s\n", info ? (const char *)info->fw_version : "");
    int rc;
//    uint32_t arg0, arg1;
//    rc = mgc3x30_get_runtime_parameter(dev, MGC3X30_PARAM_AFE_RX_ATT_S, &arg0, &arg1);
//    if (rc == 0) {
//        console_printf("Failed to get runtime parameter %d\n", rc);
//    } else {
//        console_printf("AFE_RX_ATT_S %d\n", (int)arg0);
//    }
//    rc = mgc3x30_get_runtime_parameter(dev, MGC3X30_PARAM_APPROACH_DETECTION, &arg0, &arg1);
    struct mgc3x30_fw_version_info inf;
    rc = mgc3x30_get_fw_info(dev, &inf);
    if (rc == 0) {
        console_printf("Failed to get firmware version %d\n", rc);
    } else {
        console_printf("Firmware %s\n", inf.fw_version);
    }
}

static struct mgc3x30_client mgc_client = {
    .ready = mgc_ready,
    .gesture = mgc_gesture,
    .touch = mgc_touch,
};

static int
mgc3x30_shell_cmd_close(const struct shell_cmd *cmd, int argc, char **argv, struct streamer *str)
{
    (void)cmd;
    (void)argc;
    (void)argv;
    (void)str;

    if (mgc3x30_dev) {
        os_dev_close(&mgc3x30_dev->dev.bnode.odev);
    }
    mgc3x30_dev = NULL;

    return 0;
}

static int
mgc3x30_shell_cmd_open(const struct shell_cmd *cmd, int argc, char **argv, struct streamer *str)
{
    const char *name = "mgc3130_0";

    (void)cmd;

    if (argc > 1) {
        name = argv[1];
    }
    if (mgc3x30_dev) {
        if (strcmp(name, mgc3x30_dev->dev.bnode.odev.od_name) != 0) {
            os_dev_close(&mgc3x30_dev->dev.bnode.odev);
        }
    }

    shell_stream = str;

    mgc3x30_dev = mgc3x30_open(name, &mgc_client);
    if (mgc3x30_dev == 0) {
        streamer_printf(str, "Failed to open device %s\n", name);
    }

    return 0;
}

static int
mgc3x30_shell_cmd_reset(const struct shell_cmd *cmd, int argc, char **argv, struct streamer *str)
{
    (void)cmd;
    (void)argc;
    (void)argv;
    (void)str;

    if (mgc3x30_dev) {
        mgc3x30_reset(mgc3x30_dev);
    }

    return 0;
}

static int
mgc3x30_shell_cmd_get(const struct shell_cmd *cmd, int argc, char **argv, struct streamer *str)
{
    (void)cmd;
    (void)argc;
    (void)argv;
    (void)str;
    int rc;
    uint16_t param_id = 0xA0;
    uint32_t arg0;
    uint32_t arg1;

    if (argc > 1) {
        param_id = (uint16_t)parse_ll_bounds(argv[1], 0, 0x1000, &rc);
        if (rc) {
            streamer_printf(str, "Invalid parameter_id %s\n", argv[1]);
            return 0;
        }
    }
    if (mgc3x30_dev) {
        rc = mgc3x30_get_runtime_parameter(mgc3x30_dev, param_id, &arg0, &arg1);
        if (rc) {
            streamer_printf(str, "Failed to read runtime param\n");
        } else {
            streamer_printf(str, "arg0 = 0x%08X, arg1 = 0x%08X\n", arg0, arg1);
        }
    }

    return 0;
}

static const struct shell_cmd mgc3x30_cmds[] = {
    SHELL_CMD_EXT("open", mgc3x30_shell_cmd_open, HELP(open)),
    SHELL_CMD_EXT("close", mgc3x30_shell_cmd_close, HELP(close)),
    SHELL_CMD_EXT("reset", mgc3x30_shell_cmd_reset, HELP(reset)),
    SHELL_CMD_EXT("get", mgc3x30_shell_cmd_get, HELP(get)),
    SHELL_CMD_EXT(NULL, NULL, NULL)
};

#if MYNEWT_VAL(SHELL_COMPAT)
static const struct shell_cmd mgc3x30_shell_cmd_struct;

static int
mgc3x30_help(void)
{
    console_printf("%s cmd\n", mgc3x30_shell_cmd_struct.sc_cmd);
    console_printf("cmd:\n");
    console_printf("\thelp\n");
    console_printf("\tfw\n");
    console_printf("\topen\n");
    console_printf("\tclose\n");
    console_printf("\treset\n");
    return 0;
}

static int
mgc3x30_shell_cmd(int argc, char **argv)
{
    const struct shell_cmd *cmd = mgc3x30_cmds;

    argv++;
    argc--;
    if (argc == 0 || strcmp(argv[0], "help") == 0) {
        return mgc3x30_help();
    }

    for (; cmd->sc_cmd; ++cmd) {
        if (strcmp(cmd->sc_cmd, argv[0]) == 0) {
            return cmd->sc_cmd_func(argc, argv);
        }
    }

    console_printf("Unknown command %s\n", argv[1]);
    return 0;
}

static const struct shell_cmd mgc3x30_shell_cmd_struct = {
    .sc_cmd = "mgc3x30",
    .sc_cmd_func = mgc3x30_shell_cmd
};
#endif

int
mgc3x30_shell_init(void)
{
    int rc;

#if MYNEWT_VAL(SHELL_COMPAT)
    rc = shell_cmd_register(&mgc3x30_shell_cmd_struct);
    SYSINIT_PANIC_ASSERT(rc == 0);
#endif

    rc = shell_register("mgc3x30", mgc3x30_cmds);
    return rc;
}
