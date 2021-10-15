/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <settings/settings.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/matrix.h>
#include <zmk/kscan.h>
#include <zmk/display.h>
#include <drivers/ext_power.h>

#include <drivers/sensor.h>

#define ZMK_KSCAN_DEV DT_LABEL(ZMK_MATRIX_NODE_ID)

static void slider_trigger_handler(const struct device *dev, struct sensor_trigger *trigger) {
    struct sensor_value position;

    sensor_sample_fetch(dev);
    sensor_channel_get(dev, SENSOR_CHAN_POS_DX, &position);

    LOG_INF("Slider position: %d\n", position.val1);
}

void main(void) {
    LOG_INF("Welcome to ZMK!\n");

    if (zmk_kscan_init(ZMK_KSCAN_DEV) != 0) {
        return;
    }

#ifdef CONFIG_ZMK_DISPLAY
    zmk_display_init();
#endif /* CONFIG_ZMK_DISPLAY */

    const struct device *dev = device_get_binding("CY8CMBR3106S");
    struct sensor_trigger trigger;

    trigger.type = SENSOR_TRIG_DATA_READY;
    trigger.chan = SENSOR_CHAN_POS_DX;

    sensor_trigger_set(dev, &trigger, slider_trigger_handler);
}
