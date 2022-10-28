/**
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_touch_slider

#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>
#include <logging/log.h>

#include <zmk/endpoints.h>
#include <zmk/hid.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct touch_slider_config {
    const struct device *sensor;
    sensor_trigger_handler_t trigger_handler;
};

struct touch_slider_data {
    struct sensor_trigger trigger;
};

static void touch_slider_on_value_change(const struct device *dev) {
    const struct touch_slider_config *config = dev->config;

    struct sensor_value position;

    sensor_sample_fetch(config->sensor);
    sensor_channel_get(config->sensor, SENSOR_CHAN_POS_DX, &position);

    LOG_INF("Slider position: %d", position.val1);

    // if (position.val1 == 0xff) {
    //     zmk_hid_consumer_volume_reset();
    //     zmk_endpoints_send_report(HID_USAGE_CONSUMER);
    //     return;
    // }

    // zmk_hid_consumer_volume_set((uint8_t)position.val1);
    // zmk_endpoints_send_report(HID_USAGE_CONSUMER);
}

static int touch_slider_init(const struct device *dev) {
    const struct touch_slider_config *config = dev->config;
    struct touch_slider_data *data = dev->data;

    sensor_trigger_set(config->sensor, &data->trigger, config->trigger_handler);

    return 0;
}

#define TOUCH_SLIDER_DEVICE(idx)                                                                   \
                                                                                                   \
    static void touch_slider_##idx##_trigger_handler(const struct device *sensor,                  \
                                                     struct sensor_trigger *trigger) {             \
        const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(idx));                                \
                                                                                                   \
        touch_slider_on_value_change(dev);                                                         \
    }                                                                                              \
                                                                                                   \
    static struct touch_slider_config touch_slider_##idx##_config = {                              \
        .sensor = DEVICE_DT_GET(DT_PHANDLE_BY_IDX(DT_DRV_INST(idx), sensor, 0)),                   \
        .trigger_handler = touch_slider_##idx##_trigger_handler,                                   \
    };                                                                                             \
                                                                                                   \
    static struct touch_slider_data touch_slider_##idx##_data = {                                  \
        .trigger = {                                                                               \
            .type = SENSOR_TRIG_DATA_READY,                                                        \
            .chan = SENSOR_CHAN_POS_DX,                                                            \
        },                                                                                         \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(idx, &touch_slider_init, NULL, &touch_slider_##idx##_data,               \
                          &touch_slider_##idx##_config, POST_KERNEL,                               \
                          CONFIG_APPLICATION_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TOUCH_SLIDER_DEVICE);
