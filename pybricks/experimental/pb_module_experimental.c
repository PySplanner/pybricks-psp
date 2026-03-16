// SPDX-License-Identifier: MIT
// Copyright (c) 2018-2026 The Pybricks Authors

#include "py/mpconfig.h"

#if PYBRICKS_PY_EXPERIMENTAL

#include "py/mphal.h"
#include "py/obj.h"
#include "py/runtime.h"
#include <math.h>

// Direct Hardware Access Headers
#include <pbio/tacho.h>
#include <pbio/port.h>

// Architecture Detection & Optimization Macros
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
    #define IS_CORTEX_M 1
    #define ACCEL_RAM __attribute__((section(".data"), noinline))
#else
    #define IS_CORTEX_M 0
    #define ACCEL_RAM 
#endif

// Constants
static const float PI_F          = 3.141592653589793f;
static const float TWO_PI_F      = 6.283185307179586f;
static const float HALF_PI_F     = 1.570796326794896f;
static const float INV_TWO_PI_F  = 0.159154943091895f;

// -----------------------------------------------------------------------------
// Core Math Engines
// -----------------------------------------------------------------------------

ACCEL_RAM static float fast_sin_internal(float theta) {
    float x = theta * INV_TWO_PI_F;
    x = theta - (float)((int)(x + (x > 0 ? 0.5f : -0.5f))) * TWO_PI_F;

    if (x > HALF_PI_F) { x = PI_F - x; }
    else if (x < -HALF_PI_F) { x = -PI_F - x; }

    float x2 = x * x;
    float res;

    #if IS_CORTEX_M
        res = -0.0001848814f; 
        res = 0.0083119000f + (x2 * res);
        res = -0.1666555409f + (x2 * res);
        res = 1.0f + (x2 * res); // Note: 1.0f is faster for absolute error Minimax
    #else
        res = 0.9999990609f + x2 * (-0.1666555409f + x2 * (0.0083119000f + x2 * -0.0001848814f));
    #endif
    
    return x * res;
}

// -----------------------------------------------------------------------------
// Hardware Benchmarks
// -----------------------------------------------------------------------------

static mp_obj_t experimental_odometry_benchmark(mp_obj_t num_iters_in, mp_obj_t wheel_circ_in) {
    int num_iters = mp_obj_get_int(num_iters_in);
    float wheel_circ = mp_obj_get_float(wheel_circ_in);
    float deg_to_mm = wheel_circ / 360.0f;

    float robot_x = 0.0f, robot_y = 0.0f;
    int32_t last_left = 0, last_right = 0;
    float heading = 0.0f;
    
    // FIX 1: Updated to pbio_tacho_get_angle
    pbio_tacho_get_angle(pbio_tacho_get_tacho(PBIO_PORT_ID_A), &last_left);
    pbio_tacho_get_angle(pbio_tacho_get_tacho(PBIO_PORT_ID_D), &last_right);

    uint32_t start_time = mp_hal_ticks_ms();

    for (int i = 0; i < num_iters; i++) {
        int32_t cur_left, cur_right;
        pbio_tacho_get_angle(pbio_tacho_get_tacho(PBIO_PORT_ID_A), &cur_left);
        pbio_tacho_get_angle(pbio_tacho_get_tacho(PBIO_PORT_ID_D), &cur_right);

        float d_left = (float)(cur_left - last_left) * deg_to_mm;
        float d_right = (float)(cur_right - last_right) * deg_to_mm;
        float linear_delta = (d_left + d_right) / 2.0f;
        float heading_delta = (d_right - d_left) / 96.0f; // Axle track 96mm

        if (fabsf(linear_delta) > 0.0001f) {
            float avg_h = heading + (heading_delta / 2.0f);
            robot_x += linear_delta * fast_sin_internal(avg_h + HALF_PI_F);
            robot_y += linear_delta * fast_sin_internal(avg_h);
        }

        heading += heading_delta;
        last_left = cur_left;
        last_right = cur_right;

        // FIX 2 & 3: Semicolon and proper event polling for Pybricks bricks
        if ((i % 1000) == 0) { 
            mp_handle_pending(true); 
        }
    }

    uint32_t duration_ms = mp_hal_ticks_ms() - start_time;
    float duration = (float)duration_ms / 1000.0f;

    mp_obj_t tuple[5] = {
        mp_obj_new_float_from_f(duration),
        mp_obj_new_int(num_iters),
        mp_obj_new_float_from_f((float)num_iters / duration),
        mp_obj_new_float_from_f(robot_x),
        mp_obj_new_float_from_f(robot_y)
    };
    return mp_obj_new_tuple(5, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_2(experimental_odometry_benchmark_obj, experimental_odometry_benchmark);

// Module Globals
static const mp_rom_map_elem_t experimental_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),             MP_ROM_QSTR(MP_QSTR_experimental) },
    { MP_ROM_QSTR(MP_QSTR_odometry_benchmark),   MP_ROM_PTR(&experimental_odometry_benchmark_obj) },
};
static MP_DEFINE_CONST_DICT(pb_module_experimental_globals, experimental_globals_table);

const mp_obj_module_t pb_module_experimental = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&pb_module_experimental_globals,
};

#if !MICROPY_MODULE_BUILTIN_SUBPACKAGES
MP_REGISTER_MODULE(MP_QSTR_pybricks_dot_experimental, pb_module_experimental);
#endif

#endif // PYBRICKS_PY_EXPERIMENTAL