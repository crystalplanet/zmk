/**
 *
 * Copyright (c) 2023 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_animation_wpm

#define WPM_CALC_BUFFER_LENGTH 15
#define WPM_CALC_INTERVAL 300
#define WPM_CALC_SIGMA 15
#define WPM_CALC_STROKES_PER_WORD 5

#include <math.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/animation.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct animation_wpm_config {
    size_t *pixel_map;
    size_t pixel_map_size;
    struct zmk_color_hsl *colors;
    bool is_multi_color;
    uint8_t bounds_min;
    uint8_t bounds_max;
    bool is_horizontal;
    uint8_t max_wpm;
    uint8_t blending_mode;
};

/**
 * Holds the number of keystrokes between subsequent WPM re-calculations.
 * At 300ms intervals, a 15 element buffer covers the last 4.5s.
 */
static uint8_t keystrokes[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/**
 *
 */
static uint8_t keystrokes_index = 0;

/**
 * The second-latest WPM measurement value.
 */
static uint8_t last_wpm = 0;

/**
 * The latest WPM measurement value.
 */
static uint8_t current_wpm = 0;

/**
 * The timestamp for the last WPM measurement.
 */
static uint64_t current_wpm_timestamp = 0;

static void animation_wpm_calc_value(struct k_work *work);

K_WORK_DEFINE(animation_wpm_work, animation_wpm_calc_value);

static void animation_wpm_tick_handler(struct k_timer *timer) {
    k_work_submit(&animation_wpm_work);
}

K_TIMER_DEFINE(animation_wpm_tick, animation_wpm_tick_handler, NULL);

/**
 * Calculates a real-time WPM value.
 */
static void animation_wpm_calc_value(struct k_work *work) {
    uint8_t total_keystrokes = 0;

    float averages = 0.0;
    float weights = 0.0;

    for (size_t i = 0; i < WPM_CALC_BUFFER_LENGTH; ++i) {
        float distance = keystrokes_index - i + (i > keystrokes_index ? WPM_CALC_BUFFER_LENGTH : 0);
        float weight = exp(-((distance * distance) / (2 * WPM_CALC_SIGMA)));

        weights += weight;
        averages += weight * keystrokes[i];

        total_keystrokes += keystrokes[i];
    }

    last_wpm = current_wpm;
    current_wpm =
        ((averages / weights) * ((1000 * 60) / WPM_CALC_INTERVAL)) / WPM_CALC_STROKES_PER_WORD;

    current_wpm_timestamp = k_uptime_get();

    if (current_wpm > 0) {
        zmk_animation_request_frames(1);
    }

    if (++keystrokes_index == WPM_CALC_BUFFER_LENGTH) {
        keystrokes_index = 0;
    }

    keystrokes[keystrokes_index] = 0;

    if (total_keystrokes == 0) {
        k_timer_stop(&animation_wpm_tick);
    }
}

static int animation_wpm_on_key_press(const zmk_event_t *event) {
    const struct zmk_position_state_changed *pos_event;

    if ((pos_event = as_zmk_position_state_changed(event)) == NULL) {
        // Event not supported.
        return -ENOTSUP;
    }

    if (!pos_event->state) {
        // Don't track key releases.
        return 0;
    }

    keystrokes[keystrokes_index] += 1;

    if (k_timer_remaining_get(&animation_wpm_tick) == 0) {
        k_timer_start(&animation_wpm_tick, K_MSEC(WPM_CALC_INTERVAL), K_MSEC(WPM_CALC_INTERVAL));
    }

    return 0;
}

ZMK_LISTENER(animation_wpm, animation_wpm_on_key_press);
ZMK_SUBSCRIPTION(animation_wpm, zmk_position_state_changed);

static struct zmk_color_rgb animation_wpm_get_frame_color(const struct device *dev,
                                                          float animation_step) {
    const struct animation_wpm_config *config = dev->config;

    struct zmk_color_rgb rgb;
    struct zmk_color_hsl hsl;

    if (!config->is_multi_color) {
        zmk_hsl_to_rgb(&config->colors[0], &rgb);
        return rgb;
    }

    zmk_interpolate_hsl(&config->colors[0], &config->colors[1], &hsl, animation_step);
    zmk_hsl_to_rgb(&hsl, &rgb);

    return rgb;
}

static void animation_wpm_render_frame(const struct device *dev, struct animation_pixel *pixels,
                                       size_t num_pixels) {
    const struct animation_wpm_config *config = dev->config;

    size_t *pixel_map = config->pixel_map;

    float timestamp_delta =
        ((float)(k_uptime_get() - current_wpm_timestamp)) / ((float)WPM_CALC_INTERVAL);

    if (timestamp_delta > 1.0) {
        timestamp_delta = 1.0;
    }

    float wpm_delta = (float)last_wpm + (((float)current_wpm - (float)last_wpm) * timestamp_delta);

    float animation_step = wpm_delta / ((float)config->max_wpm);

    if (animation_step > 1.0) {
        animation_step = 1.0;
    }

    struct zmk_color_rgb color = animation_wpm_get_frame_color(dev, animation_step);

    for (size_t i = 0; i < config->pixel_map_size; ++i) {
        const uint8_t position = config->is_horizontal ? pixels[pixel_map[i]].position_x
                                                       : pixels[pixel_map[i]].position_y;
        const uint8_t upper_bound =
            config->bounds_min + (config->bounds_max - config->bounds_min) * animation_step;

        if (position < config->bounds_min || upper_bound < position) {
            continue;
        }

        pixels[pixel_map[i]].value =
            zmk_apply_blending_mode(pixels[pixel_map[i]].value, color, config->blending_mode);
    }

    if (last_wpm == 0 && current_wpm == 0) {
        return;
    }

    zmk_animation_request_frames(1);
}

static void animation_wpm_start(const struct device *dev) { zmk_animation_request_frames(1); }

static void animation_wpm_stop(const struct device *dev) {}

static int animation_wpm_init(const struct device *dev) { return 0; };

static const struct animation_api animation_wpm_api = {
    .on_start = animation_wpm_start,
    .on_stop = animation_wpm_stop,
    .render_frame = animation_wpm_render_frame,
};

#define ANIMATION_WPM_DEVICE(idx)                                                                  \
                                                                                                   \
    static size_t animation_wpm_##idx##_pixel_map[] = DT_INST_PROP(idx, pixels);                   \
                                                                                                   \
    static uint32_t animation_wpm_##idx##_colors[] = DT_INST_PROP(idx, colors);                    \
                                                                                                   \
    static struct animation_wpm_config animation_wpm_##idx##_config = {                            \
        .pixel_map = &animation_wpm_##idx##_pixel_map[0],                                          \
        .pixel_map_size = DT_INST_PROP_LEN(idx, pixels),                                           \
        .colors = (struct zmk_color_hsl *)animation_wpm_##idx##_colors,                            \
        .is_multi_color = DT_INST_PROP_LEN(idx, colors) > 1,                                       \
        .bounds_min = DT_INST_PROP_BY_IDX(idx, bounds, 0),                                         \
        .bounds_max = DT_INST_PROP_BY_IDX(idx, bounds, 1),                                         \
        .is_horizontal = 0 == (DT_INST_ENUM_IDX(idx, bounds_axis)),                                \
        .max_wpm = DT_INST_PROP(idx, max_wpm),                                                     \
        .blending_mode = DT_INST_ENUM_IDX(idx, blending_mode),                                     \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(idx, animation_wpm_init, NULL, NULL, &animation_wpm_##idx##_config,      \
                          POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY, &animation_wpm_api);

DT_INST_FOREACH_STATUS_OKAY(ANIMATION_WPM_DEVICE);
