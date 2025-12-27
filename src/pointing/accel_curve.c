#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <drivers/behavior_accel_curves_runtime.h>
#include "zephyr/shell/shell.h"

#define DT_DRV_COMPAT zmk_accel_curve
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

static const struct device* devices[DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT)];
static uint8_t num_dev = 0;

static struct k_work_delayable load_curves_work;
static bool work_initialized = false;

static int16_t bezier_eval(const int16_t p0, const int16_t p1, const int16_t p2, const int16_t p3, const float t) {
    const float u = 1.0f - t;
    const float tt = t * t;
    const float uu = u * u;
    const float uuu = uu * u;
    const float ttt = tt * t;
    return (int16_t) (uuu * p0 + 3 * uu * t * p1 + 3 * u * tt * p2 + ttt * p3);
}

static int set_curves(const struct device* dev, const char* datastring) {
    const struct zip_accel_curve_data *data = dev->data;
    const struct zip_accel_curve_config *config = dev->config;

    if (!datastring || !data->curves) {
        return -EINVAL;
    }

    uint8_t curve_count = 0;
    const char* ptr = datastring;
    int16_t values[8];
    while (*ptr && curve_count < config->max_curves) {
        const int parsed = sscanf(ptr, "%hd %hd %hd %hd %hd %hd %hd %hd",
            &values[0], &values[1], &values[2], &values[3],
            &values[4], &values[5], &values[6], &values[7]);

        if (parsed != 8) break;

        if (curve_count == 0) {
            data->curves[curve_count] = (struct curve){
                .start = {0, 10},
                .end = {values[2], values[3]},
                .cp1 = {values[4], values[5]},
                .cp2 = {values[6], values[7]}
            };
        } else {
            data->curves[curve_count] = (struct curve){
                .start = {values[0], values[1]},
                .end = {values[2], values[3]},
                .cp1 = {values[4], values[5]},
                .cp2 = {values[6], values[7]}
            };
        }

        curve_count++;

        for (int i = 0; i < 8 && *ptr; i++) {
            while (*ptr && (*ptr == ' ' || *ptr == '\t')) ptr++;
            while (*ptr && *ptr != ' ' && *ptr != '\t') ptr++;
        }
    }

    if (curve_count < 1 || curve_count > config->max_curves) {
        LOG_ERR("Invalid number of curves or parsing failed: %d", curve_count);
        return -EINVAL;
    }

    for (uint8_t i = 1; i < curve_count; i++) {
        if (data->curves[i].start.x != data->curves[i-1].end.x || data->curves[i].start.y != data->curves[i-1].end.y) {
            LOG_ERR("Curves are not continuous at index %d", i);
            return -EINVAL;
        }
    }

    const uint32_t points_per_curve = curve_count > 0 ? config->points / curve_count : 0;
    uint32_t point_idx = 0;

    for (uint8_t curve_idx = 0; curve_idx < curve_count && point_idx < config->points; curve_idx++) {
        const struct curve *c = &data->curves[curve_idx];
        const uint32_t num_points = (curve_idx == curve_count - 1)
            ? (config->points - point_idx)
            : points_per_curve;

        for (uint32_t i = 0; i < num_points && point_idx < config->points; i++) {
            const float t = (float) i / (float) (num_points - 1);
            data->points[point_idx].x = bezier_eval(c->start.x, c->cp1.x, c->cp2.x, c->end.x, t);
            data->points[point_idx].y = bezier_eval(c->start.y, c->cp1.y, c->cp2.y, c->end.y, t);
            point_idx++;
        }
    }

    for (uint32_t i = 1; i < point_idx; i++) {
        if (data->points[i].x < data->points[i-1].x && !(points_per_curve % i == 0 && data->points[i].x == data->points[i-1].x)) {
            LOG_ERR("now: %d prev: %d per: %d", (int)(data->points[i].x * 1000), (int)(data->points[i-1].x * 1000), points_per_curve);
            LOG_ERR("Invalid point sequence: X values must be strictly increasing at index %d/%d", i, point_idx);
            return -EINVAL;
        }
    }

    return curve_count;
}

static int save_curves_to_nvs(const struct device* dev, const char* datastring) {
    const struct zip_accel_curve_config *config = dev->config;

    char setting_name[64];
    snprintf(setting_name, sizeof(setting_name), "%s/%s", ACCEL_CURVE_NVS_PREFIX, config->device_name);

    const int rc = settings_save_one(setting_name, datastring, strlen(datastring) + 1);
    if (rc != 0) {
        LOG_ERR("Failed to save curves to NVS for %s: %d", config->device_name, rc);
        return rc;
    }
    
    LOG_DBG("Saved curves to NVS for %s", config->device_name);
    return 0;
}

static int load_cb(const char *key, const size_t len, const settings_read_cb read_cb, void *cb_arg, void *param) {
    const struct device* dev = param;
    char data[len];
    const int read = read_cb(cb_arg, &data, len);
    if (read == 0) {
        LOG_ERR("Failed to read curve: no data read");
        return -EACCES;
    }

    return data_import(dev, data);
}

static int load_curves_from_nvs(const struct device* dev) {
    const struct zip_accel_curve_config *config = dev->config;
    char setting_name[32];
    sprintf(setting_name, "%s/%s", ACCEL_CURVE_NVS_PREFIX, config->device_name);
    return settings_load_subtree_direct(setting_name, load_cb, (struct device*) dev);
}

static void load_curves_work_handler(struct k_work *work) {
    LOG_INF("Loading curves from NVS for all %d devices", num_dev);
    
    for (uint8_t i = 0; i < num_dev; i++) {
        if (devices[i] != NULL) {
            load_curves_from_nvs(devices[i]);
        }
    }
}

static int dump_cb(const char *key, const size_t len, const settings_read_cb read_cb, void *cb_arg, void *param) {
    char data[len];
    const int read = read_cb(cb_arg, &data, len);
    if (read == 0) {
        LOG_ERR("Failed to read curve data for key: %s", key);
        return 0;
    }

    char _data[read + 1];
    sprintf(_data, "%s", data);

    if (param == NULL) {
        if (key != NULL) {
            LOG_INF("Device: %s", key);
        }

        LOG_INF("Curve: %s", _data);
    } else {
        const struct shell *sh = param;

        if (key != NULL) {
            shell_print(sh, "Device: %s", key);
        }

        shell_print(sh, "Curve: %s", _data);
    }

    return 0;
}

int dump_curves(const struct shell *sh, const char* name) {
    char setting_name[32];
    if (name == NULL) {
        sprintf(setting_name, "%s", ACCEL_CURVE_NVS_PREFIX);
    } else {
        sprintf(setting_name, "%s/%s", ACCEL_CURVE_NVS_PREFIX, name);
    }

    const int rc = settings_load_subtree_direct(setting_name, dump_cb, (struct shell *) sh);
    if (rc != 0) {
        LOG_ERR("Failed to dump curves: %d", rc);
        return rc;
    }

    return 0;
}

int data_import(const struct device* dev, const char* datastring) {
    if (dev == NULL) {
        LOG_ERR("Device not initialized");
        return -EINVAL;
    }

    struct zip_accel_curve_data *data = dev->data;
    const struct zip_accel_curve_config *config = dev->config;

    data->curves = malloc(sizeof(struct curve) * config->max_curves);
    data->points = malloc(sizeof(struct point) * config->points);
    
    if (!data->remainders) {
        data->remainders = malloc(sizeof(float) * config->event_codes_len);
        if (data->remainders) {
            for (uint8_t i = 0; i < config->event_codes_len; i++) {
                data->remainders[i] = 0.0f;
            }
        }
    }

    if (!data->curves || !data->points) {
        LOG_ERR("Failed to allocate memory for curves or points");
        return -EINVAL;
    }

    const int curve_count = set_curves(dev, datastring);
    LOG_INF("%d curves found", curve_count);

    if (curve_count > 0) {
        data->initialized = true;
        data->num_curves = curve_count;
        
        if (data->stored_datastring) {
            free(data->stored_datastring);
        }
        data->stored_datastring = strdup(datastring);
        save_curves_to_nvs(dev, datastring);
    } else {
        data->initialized = false;
    }
    
    free(data->curves);
    return curve_count;
}

// ReSharper disable once CppParameterMayBeConstPtrOrRef
static int sy_handle_event(const struct device *dev, struct input_event *event, const uint32_t p1,
                           const uint32_t p2, struct zmk_input_processor_state *s) {
    const struct zip_accel_curve_data *data = dev->data;
    const struct zip_accel_curve_config *config = dev->config;

    if (!data->initialized) {
        return 0;
    }

    uint8_t event_idx = 0;
    bool relevant = false;
    for (uint8_t i = 0; i < config->event_codes_len; i++) {
        if (event->code == config->event_codes[i]) {
            relevant = true;
            event_idx = i;
            break;
        }
    }

    if (!relevant) {
        return 0;
    }

    const int32_t input_val = event->value;
    const int32_t abs_input = abs(input_val);
    const int32_t sign = (input_val >= 0) ? 1 : -1;

    if (config->points == 0 || !data->points || !data->remainders) {
        return 0;
    }

    float coef;
    if (abs_input >= data->points[config->points - 1].x) {
        coef = data->points[config->points - 1].y / 100.0f;
    } else if (abs_input <= data->points[0].x) {
        coef = data->points[0].y / 100.0f;
    } else {
        uint32_t i = 0;
        for (i = 0; i < config->points - 1; i++) {
            if (abs_input >= data->points[i].x && abs_input < data->points[i + 1].x) {
                break;
            }
        }

        const struct point *point0 = &data->points[i];
        const struct point *point1 = &data->points[i + 1];
        const float t = (float) (abs_input - point0->x) / (float) (point1->x - point0->x);
        const float interpolated_y = point0->y + t * (point1->y - point0->y);
        coef = interpolated_y / 100.0f;
    }

    LOG_DBG("event value: %d, resulting coef: %.02f", input_val, coef);

    const float result_with_remainder = (float) abs_input * coef + data->remainders[event_idx];
    const int32_t result_int = (int32_t) result_with_remainder;
    data->remainders[event_idx] = result_with_remainder - (float) result_int;

    event->value = result_int * sign;
    return 0;
}

static int sy_init(const struct device *dev) {
    if (!dev) {
        LOG_ERR("Unexpected NULL ptr");
        return -EINVAL;
    }

    const struct zip_accel_curve_config *config = dev->config;
    struct zip_accel_curve_data *data = dev->data;
    data->dev = dev;

    for (uint8_t i = 0; i < num_dev; i++) {
        if (devices[i] == NULL) continue;

        const struct zip_accel_curve_config *_config = devices[i]->config;
        if (strcmp(config->device_name, _config->device_name) == 0) {
            LOG_ERR("Duplicate device name: %s", config->device_name);
            return -EINVAL;
        }
    }

    if (num_dev < DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT)) {
        devices[num_dev] = dev;
        num_dev++;
    } else {
        LOG_ERR("Too many devices");
        return -EINVAL;
    }
    
    if (!work_initialized) {
        k_work_init_delayable(&load_curves_work, load_curves_work_handler);
        work_initialized = true;
    }
    
    k_work_cancel_delayable(&load_curves_work);
    k_work_reschedule(&load_curves_work, K_MSEC(1350));
    return 0;
}

const struct device* device_by_name(const char* name) {
    if (name == NULL) return NULL;
    for (uint8_t i = 0; i < num_dev; i++) {
        if (devices[i] == NULL) continue;

        const struct zip_accel_curve_config *config = devices[i]->config;
        if (strcmp(config->device_name, name) == 0) {
            return devices[i];
        }
    }

    return NULL;
}

int list_devices(char*** names) {
    if (num_dev == 0) {
        return 0;
    }

    *names = NULL;
    char** collected = malloc(sizeof(char*) * num_dev);
    for (uint8_t i = 0; i < num_dev; i++) {
        if (devices[i] == NULL) {
            LOG_ERR("Unexpected NULL ptr");
            free(collected);
            return -EINVAL;
        };

        const struct zip_accel_curve_config *config = devices[i]->config;
        collected[i] = strdup(config->device_name);
    }

    *names = collected;
    return num_dev;
}

static struct zmk_input_processor_driver_api sy_driver_api = { .handle_event = sy_handle_event };

#define ACCEL_CURVE_INST(n)                                                                       \
    static struct zip_accel_curve_data data_##n = { 0 };                                          \
    static const struct zip_accel_curve_config config_##n = {                                     \
        .max_curves = DT_INST_PROP_OR(n, max_curves, 8),                                          \
        .points = DT_INST_PROP_OR(n, points, 64),                                                 \
        .device_name = DT_INST_PROP_OR(n, device_name, "unknown"),                                \
        .event_codes_len = DT_INST_PROP_LEN(n, event_codes),                                      \
        .event_codes = DT_INST_PROP(n, event_codes)                                               \
    };                                                                                            \
    DEVICE_DT_INST_DEFINE(n, &sy_init, NULL, &data_##n, &config_##n, POST_KERNEL,                 \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &sy_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ACCEL_CURVE_INST)
