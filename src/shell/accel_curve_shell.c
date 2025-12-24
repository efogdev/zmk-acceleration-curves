#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>
#include <zephyr/settings/settings.h>
#include "drivers/behavior_accel_curves_runtime.h"

#define DT_DRV_COMPAT zmk_curve_shell
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if IS_ENABLED(CONFIG_SHELL) && IS_ENABLED(CONFIG_ZMK_RRL_SHELL)
#define shprint(_sh, _fmt, ...) \
do { \
  if ((_sh) != NULL) \
    shell_print((_sh), _fmt, ##__VA_ARGS__); \
} while (0)

static int cmd_status(const struct shell *sh, const size_t argc, char **argv) {
    if (strcmp(argv[0], "status") == 0 && argc == 1) {
        char** names = NULL;
        const int available = list_devices(&names);
        if (available <= 0) {
            shprint(sh, "No devices found.");
        } else {
            shprint(sh, "Devices available: ");
            for (size_t i = 0; i < available; i++) {
                const struct device* dev = device_by_name(names[i]);
                if (dev == NULL) {
                    shprint(sh, "Unexpected: device not found.");
                    return -EBUSY;
                }
                const struct zip_accel_curve_config *config = dev->config;
                shprint(sh, "  %s (up to %d curve(s), %d point(s) interpolation)", names[i], config->max_curves, config->points);
            }

            shprint(sh, "");
        }
    }

    return dump_curves(sh, argc == 1 ? NULL : argv[1]);
}

static int cmd_destroy(const struct shell *sh, const size_t argc, char **argv) {
    if (argc < 2) {
        shprint(sh, "Usage: curve destroy [name]");
        return -EINVAL;
    }

    const struct device* dev = device_by_name(argv[1]);
    if (dev == NULL) {
        shprint(sh, "Device not found.");
        return -EINVAL;
    }

    char buf[24];
    sprintf(buf, "%s/%s", ACCEL_CURVE_NVS_PREFIX, argv[1]);
    const int err = settings_delete(buf);
    if (err < 0) {
        shprint(sh, "Could not delete settings.");
        return err;
    }

    shprint(sh, "Success.");
    return 0;
}

static int cmd_set(const struct shell *sh, const size_t argc, char **argv) {
    if (argc < 3) {
        shprint(sh, "Usage: curve set [name] [...values...]");
        return -EINVAL;
    }

    const struct device* dev = device_by_name(argv[1]);
    if (dev == NULL) {
        shprint(sh, "Device not found.");
        return -EINVAL;
    }

    size_t total_len = 0;
    for (size_t i = 2; i < argc; i++) {
        total_len += strlen(argv[i]);
        if (i < argc - 1) {
            total_len++; 
        }
    }
    total_len++;

    char* datastring = malloc(total_len);
    if (datastring == NULL) {
        shprint(sh, "Memory allocation failed.");
        return -ENOMEM;
    }

    datastring[0] = '\0';
    for (size_t i = 2; i < argc; i++) {
        strcat(datastring, argv[i]);
        if (i < argc - 1) {
            strcat(datastring, " ");
        }
    }

    const int ret = data_import(dev, datastring);
    free(datastring);

    if (ret == 0) {
        shprint(sh, "Done!");
    }
    
    return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_curve,
    SHELL_CMD(status, NULL, "Get current status", cmd_status),
    SHELL_CMD(dump, NULL, "Dump curve(s)", cmd_status),
    SHELL_CMD(set, NULL, "Write curve", cmd_set),
    SHELL_CMD(destroy, NULL, "Clear device", cmd_destroy),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(curve, &sub_curve, "Acceleration curves", NULL);
#endif
