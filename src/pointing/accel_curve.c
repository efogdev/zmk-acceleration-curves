#include <stdlib.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT zmk_accel_curve
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct point {
    int16_t x;
    int16_t y;
};

struct curve {
    struct point start, end, cp1, cp2;
};

struct zip_accel_curve_config {
    const uint8_t max_curves, points;
};

struct zip_accel_curve_data {
    const struct device *dev;
    bool initialized;
    struct curve* curves;
    struct point* points;
};

static struct curve* parse_curve(const char* data) {
    // data format: start end cp1 cp2
    // point format: X Y
    // example data: 0 0 10 10 0 5 5 0

    return NULL;
}

static void data_init(const struct device *dev) {
    struct zip_accel_curve_data *data = dev->data;
    const struct zip_accel_curve_config *config = dev->config;

    data->curves = malloc(sizeof(struct curve) * config->max_curves);
    data->points = malloc(sizeof(struct point) * config->points);

    // parse curves, check that they are continuous, meaning that every next curve starts at the end of the previous
    // approximate into config->points number of points
    // verify that for each X there is single Y and that every next X is larger than previous

    data->initialized = true;
}

// ReSharper disable once CppParameterMayBeConstPtrOrRef
static int sy_handle_event(const struct device *dev, struct input_event *event, const uint32_t p1,
                           const uint32_t p2, struct zmk_input_processor_state *s) {
    struct zip_accel_curve_data *data = dev->data;
    const struct zip_accel_curve_config *config = dev->config;

    if (!data->initialized) {
        data_init(dev);
    }

    // if (event->code == INPUT_REL_X || event->code == INPUT_REL_Y) {
    //
    // } else if (event->code == INPUT_REL_WHEEL) {
    //
    // } else {
    //     return 0;
    // }
    
    return 0;
}

static int sy_init(const struct device *dev) {
    struct zip_accel_curve_data *data = dev->data;

    data->dev = dev;
    return 0;
}

static struct zmk_input_processor_driver_api sy_driver_api = {
    .handle_event = sy_handle_event,
};

static struct zip_accel_curve_data data = {
    .initialized = false,
};

static const struct zip_accel_curve_config config = {
    .max_curves = DT_INST_PROP_OR(0, max_curves, 8),
    .points = DT_INST_PROP_OR(0, points, 64),
};

DEVICE_DT_INST_DEFINE(0, &sy_init, NULL, &data, &config, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &sy_driver_api);
