/**
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT cypress_cy8cmbr3106s

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <logging/log.h>
#include <sys/math_extras.h>
#include <sys/util.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define CY8CMBR3106S_REG_CTRL_CMD 0x86
#define CY8CMBR3106S_REG_SPO 0x4c

#define CY8CMBR3106S_REG_SLIDER_CONFIG 0x5d
#define CY8CMBR3106S_REG_SLIDER_1_CONFIG 0x61
#define CY8CMBR3106S_REG_SLIDER_1_RESOLUTION 0x62
#define CY8CMBR3106S_REG_SLIDER_1_THRESHOLD 0x63
#define CY8CMBR3106S_REG_SLIDER_2_CONFIG 0x67
#define CY8CMBR3106S_REG_SLIDER_2_RESOLUTION 0x68
#define CY8CMBR3106S_REG_SLIDER_2_THRESHOLD 0x69

#define CY8CMBR3106S_REG_BASELINE 0x71
#define CY8CMBR3106S_REG_NEGATIVE_NOISE 0x72
#define CY8CMBR3106S_REG_NOISE 0x73

#define CY8CMBR3106S_REG_SLIDER_1_POSITION 0xb0
#define CY8CMBR3106S_REG_SLIDER_1_LIFTOFF 0xb1
#define CY8CMBR3106S_REG_SLIDER_2_POSITION 0xb2
#define CY8CMBR3106S_REG_SLIDER_2_LIFTOFF 0xb3

struct cy8cmbr3106s_config {
    char *bus;
    int reg;
    char *label;

    char *trigger_port;
    gpio_pin_t trigger_pin;
    gpio_dt_flags_t trigger_flags;
};

struct cy8cmbr3106s_data {
    const struct device *i2c;
    const struct device *gpio;

    uint8_t slider_position;

    struct gpio_callback trigger_callback;

    const struct sensor_trigger *trigger;
    sensor_trigger_handler_t handler;

    struct k_work work;
};

static int cy8cmbr3106s_wake(const struct device *dev) {
    const struct cy8cmbr3106s_data *data = dev->data;
    const struct cy8cmbr3106s_config *config = dev->config;

    // i2c_burst_write(data->i2c, config->reg, 0x00, NULL, 0);
    // i2c_burst_write(data->i2c, config->reg, 0x00, NULL, 0);
    // i2c_burst_write(data->i2c, config->reg, 0x00, NULL, 0);

    // return 0;

    for (int i=0; i<10; ++i) {
        // TODO: check i2c_burst_write again
        if (!i2c_burst_write(data->i2c, config->reg, 0x00, NULL, 0)) {
            return 0;
        }
    }

    LOG_ERR("CY8CMBR3106s failed to wake on I2C ping.");
    return -EIO;
}

static int cy8cmbr3106s_reg_write(const struct device *dev, uint8_t addr, uint8_t value) {
    const struct cy8cmbr3106s_data *data = dev->data;
    const struct cy8cmbr3106s_config *config = dev->config;

    // if (cy8cmbr3106s_wake(dev)) {
    //     return -EIO;
    // }

    // if (i2c_reg_write_byte(data->i2c, config->reg, addr, value)) {
    //     // CY8CMBR3106S can will NACK the first transation when in deep sleep
    //     // in which case we need to retry twice within the next 340ms

    //     k_msleep(100);
    //     i2c_reg_write_byte(data->i2c, config->reg, addr, value);
    //     k_msleep(100);
    for (int i=0; i<10; ++i) {
        if (!i2c_reg_write_byte(data->i2c, config->reg, addr, value)) {
            return 0;
        }

        k_msleep(50);
    }

    LOG_ERR("Failed writing value 0x%x to register address 0x%x on device 0x%x.", value, addr,
        config->reg);
    return -EIO;
}

static int cy8cmbr3106s_reg_read(const struct device *dev, uint8_t addr, uint8_t *value) {
    const struct cy8cmbr3106s_data *data = dev->data;
    const struct cy8cmbr3106s_config *config = dev->config;

    // if (cy8cmbr3106s_wake(dev)) {
    //     return -EIO;
    // }

    if (i2c_reg_read_byte(data->i2c, config->reg, addr, value)) {
        LOG_ERR("Failed reading from register address %x on device %x.", addr, config->reg);
        return -EIO;
    }

    return 0;
}

static int cy8cmbr3106s_init_i2c(const struct device *dev) {
    struct cy8cmbr3106s_data *data = dev->data;
    const struct cy8cmbr3106s_config *config = dev->config;

    data->i2c = device_get_binding(config->bus);

    if (data->i2c == NULL) {
        LOG_ERR("I2C device %s not found", config->bus);
        return -ENODEV;
    }

    // Set up interrupt handler on the GPIO!

    // Make sure the device is active before writing to it
    // cy8cmbr3106s_wake(dev);

    // Reset the device
    // cy8cmbr3106s_reg_write(dev, CY8CMBR3106S_REG_CTRL_CMD, 0xff);

    // k_msleep(5);

    // Enable shield electrode on SPO1 and host interrupt on SPO0.
    cy8cmbr3106s_reg_write(dev, CY8CMBR3106S_REG_SPO, 0x24);

    // Set slider configuration
    cy8cmbr3106s_reg_write(dev, CY8CMBR3106S_REG_SLIDER_CONFIG, 0x02);

    // Configure slider one
    cy8cmbr3106s_reg_write(dev, CY8CMBR3106S_REG_SLIDER_1_CONFIG, 0x18);
    cy8cmbr3106s_reg_write(dev, CY8CMBR3106S_REG_SLIDER_1_RESOLUTION, 0xff);
    cy8cmbr3106s_reg_write(dev, CY8CMBR3106S_REG_SLIDER_1_THRESHOLD, 0x0a); // 10 counts to start with

    // Configure slider two
    cy8cmbr3106s_reg_write(dev, CY8CMBR3106S_REG_SLIDER_2_CONFIG, 0x1d);
    cy8cmbr3106s_reg_write(dev, CY8CMBR3106S_REG_SLIDER_2_RESOLUTION, 0x00); // Ignored if both active
    cy8cmbr3106s_reg_write(dev, CY8CMBR3106S_REG_SLIDER_2_THRESHOLD, 0x00); // Ignored if both active

    // Other settings go here, stick with the default initially
    // cy8cmbr3106s_reg_write(dev, CY8CMBR3106S_REG_BASELINE, 0x40);
    // cy8cmbr3106s_reg_write(dev, CY8CMBR3106S_REG_NEGATIVE_NOISE, 0x40);
    // cy8cmbr3106s_reg_write(dev, CY8CMBR3106S_REG_NOISE, 0x40);

    return 0;
}

static void cy8cmbr3106s_trigger_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    struct cy8cmbr3106s_data *data = CONTAINER_OF(cb, struct cy8cmbr3106s_data, trigger_callback);

    // Handle in global thread?
    k_work_submit(&data->work);
}

static int cy8cmbr3106s_init_interrupt(const struct device *dev) {
    struct cy8cmbr3106s_data *data = dev->data;
    const struct cy8cmbr3106s_config *config = dev->config;

    data->gpio = device_get_binding(config->trigger_port);

    if (data->gpio == NULL) {
        LOG_ERR("GPIO device %s not found", config->trigger_port);
        return -ENODEV;
    }

    // INTERRUPT IS ACTIVE LOW, SO GPIO NEEDS TO BE DRIVEN HIGH EXTERNALLY!!!

    // TODO: whatever this is
    //       should this actually be an input? Or do we need to generate some current on here?
    gpio_pin_configure(data->gpio, config->trigger_pin, (GPIO_INPUT | config->trigger_flags));

    gpio_init_callback(&data->trigger_callback, cy8cmbr3106s_trigger_callback, BIT(config->trigger_pin));

    if (gpio_add_callback(data->gpio, &data->trigger_callback)) {
        LOG_ERR("Failed to set the trigger callback on pin %d (%s).", config->trigger_pin, config->trigger_port);
        return -EIO;
    }

    return 0;
}

/**
 * Initiates a driver instance for CY8CMBR3106S.
 */
static int cy8cmbr3106s_init(const struct device *dev) {
    cy8cmbr3106s_init_i2c(dev);
    cy8cmbr3106s_init_interrupt(dev);

    return 0;
}

static int cy8cmbr3106s_trigger_set(const struct device *dev, const struct sensor_trigger *trigger, sensor_trigger_handler_t handler) {
    struct cy8cmbr3106s_data *data = dev->data;
    const struct cy8cmbr3106s_config *config = dev->config;

    // this is where the interrups is actually configured!!!

    // maybe need to turn this on and off before setting the two below?
    if (gpio_pin_interrupt_configure(data->gpio, config->trigger_pin, GPIO_INT_EDGE_FALLING)) {
        // failed!
    }

    data->trigger = trigger;
    data->handler = handler;

    return 0;
}

static int cy8cmbr3106s_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct cy8cmbr3106s_data *data = dev->data;

    if (cy8cmbr3106s_wake(dev)) {
        return -EIO;
    }

    cy8cmbr3106s_reg_read(data->i2c, CY8CMBR3106S_REG_SLIDER_1_POSITION, &data->slider_position);

    return 0;
}

static int cy8cmbr3106s_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val) {
    const struct cy8cmbr3106s_data *data = dev->data;

    if (chan != SENSOR_CHAN_POS_DX) {
        return -ENOTSUP;
    }

    val->val1 = data->slider_position;
    val->val2 = 0;

    return 0;
}

static const struct sensor_driver_api cy8cmbr3106s_api = {
    .trigger_set = cy8cmbr3106s_trigger_set,
    .sample_fetch = cy8cmbr3106s_sample_fetch,
    .channel_get = cy8cmbr3106s_channel_get,
};

#define CY8CMBR3106S_DEVICE(idx)                                                               \
    static struct cy8cmbr3106s_data cy8cmbr3106s_##idx##_data;                                 \
                                                                                               \
    static const struct cy8cmbr3106s_config cy8cmbr3106s_##idx##_config = {                    \
        .bus = DT_INST_BUS_LABEL(idx),                                                         \
        .reg = DT_INST_REG_ADDR(idx),                                                          \
        .label = DT_INST_LABEL(idx),                                                           \
        .trigger_port = DT_INST_GPIO_LABEL(idx, trigger_gpios),                                \
        .trigger_pin = DT_INST_GPIO_PIN(idx, trigger_gpios),                                   \
        .trigger_flags = DT_INST_GPIO_FLAGS(idx, trigger_gpios),                               \
    };                                                                                         \
                                                                                               \
    DEVICE_AND_API_INIT(cy8cmbr3106s_##idx, DT_INST_LABEL(idx), &cy8cmbr3106s_init,            \
                        &cy8cmbr3106s_##idx##_data, &cy8cmbr3106s_##idx##_config, POST_KERNEL, \
                        CONFIG_SENSOR_INIT_PRIORITY, &cy8cmbr3106s_api);

DT_INST_FOREACH_STATUS_OKAY(CY8CMBR3106S_DEVICE);
