# ZMK Acceleration Curves

Input processor that applies velocity-based acceleration to pointer or scroll input using cubic Bézier splines. Maps input speed to a multiplier, so slow movement stays precise and fast flicks cover more ground.

## Usage

Include the provided dtsi and add the processor to your input listener:

```dts
#include <input/processors/accel_curve.dtsi>

/ {
    trackball_listener: trackball_listener {
        compatible = "zmk,input-listener";
        device = <&trackball>;
        input-processors = <&zip_pointer_accel>;
    };
};
```

`zip_pointer_accel` targets `INPUT_REL_X` / `INPUT_REL_Y`. `zip_scroll_accel` targets `INPUT_REL_WHEEL` / `INPUT_REL_HWHEEL`. Neither does anything until you load a curve — without one, input passes through unmodified.

If you need different limits:

```dts
&zip_pointer_accel {
    max-curves = <8>;
    points = <64>;
};
```

- `max-curves`: max number of Bézier segments the curve can have
- `points`: resolution of the interpolated lookup table — more points means smoother transitions between segments

## Loading a curve

Curves are defined as space-separated integers via the shell and persisted to flash. Each segment is: `x0 y0 x1 y1 cp1x cp1y cp2x cp2y`.

X is input speed (×100 internally), Y is the acceleration coefficient (×100 internally). Segments must connect end-to-end.

Example — a gentle S-curve that starts at 1× and ramps up to ~3×:

```
curve set pointer 0 100 500 150 100 100 400 130 500 150 2000 300 700 160 1800 290
```

Check what's loaded:

```
curve status
curve status pointer
```

Remove a curve (reverts to pass-through):

```
curve destroy pointer
```

Curves are loaded from flash ~1.3s after boot. There's a brief window at startup where input is unaccelerated.

## Configuration

```kconfig
CONFIG_ZMK_ACCEL_CURVE=y
CONFIG_ZMK_ACCEL_CURVE_SHELL=y
```

## Curve format details

The first segment's start point is always implicitly `(0, 100)` (i.e., 1× at zero speed). Coordinates are integers scaled ×100 — so Y=150 means a 1.5× multiplier. Segments must be continuous: the end point of segment N must equal the start point of segment N+1.

Input values are sign-preserved: the lookup uses the absolute value, and the sign is reapplied to the output. Fractional output is accumulated across events to avoid cumulative rounding error.
