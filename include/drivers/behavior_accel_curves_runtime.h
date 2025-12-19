#pragma once
#include "zephyr/shell/shell.h"
#define ACCEL_CURVE_NVS_PREFIX "curves"

struct point {
    int16_t x;
    int16_t y;
};

struct curve {
    struct point start, end, cp1, cp2;
};

struct zip_accel_curve_config {
    const uint8_t max_curves, points;
    const uint8_t event_codes_len;
    const char* device_name;
    const uint16_t event_codes[];
};

struct zip_accel_curve_data {
    const struct device *dev;
    bool initialized;
    struct curve* curves;
    struct point* points;
    uint8_t num_curves;
    char* stored_datastring;
};

void curves_init();
int data_import(const struct device* dev, const char* datastring);
const struct device* device_by_name(const char* name);
int dump_curves(const struct shell *, const char* name);
int list_devices(char*** names);
