# Copyright (c) 2023 The ZMK Contributors
# SPDX-License-Identifier: MIT

target_sources_ifdef(CONFIG_ZMK_ACCEL_CURVE app PRIVATE accel_curve.c)
