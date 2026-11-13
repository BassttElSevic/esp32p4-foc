#pragma once

#include <stdbool.h>

/* M2 bench hardware. Initialization leaves the gate-driver enable low. */
void bench_io_init(void);
void bench_io_disable(void);
void bench_io_enable(void);

/* Single writer for PWM and sensor peripherals. Errors deassert EN.
 * vd/vq are signed modulation (0 is neutral); theta is electrical radians.
 * ADC values include the INA199 offsets. angle_code is raw mechanical 0..4095.
 */
bool bench_io_voltage(float theta, float vd, float vq);
bool bench_io_sample(float *u_mv, float *v_mv, int *angle_code);
