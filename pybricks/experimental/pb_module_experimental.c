// SPDX-License-Identifier: MIT
// Copyright (c) 2018-2021 The Pybricks Authors

#include "py/mpconfig.h"

#if PYBRICKS_PY_EXPERIMENTAL

#include "py/mphal.h"
#include "py/obj.h"
#include "py/runtime.h"
#include <math.h>

// Architecture Detection & Optimization Macros
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
    #define IS_CORTEX_M 1
    #define ACCEL_RAM __attribute__((section(".data"), noinline))
    #define DWT_CONTROL  (*((volatile uint32_t*)0xE0001000))
    #define DWT_CYCCNT   (*((volatile uint32_t*)0xE0001004))
    #define DEMCR        (*((volatile uint32_t*)0xE000EDFC))
#else
    #define IS_CORTEX_M 0
    #define ACCEL_RAM   // EV3/Linux handles RAM loading automatically
#endif

static const float PI_F          = 3.141592653589793f;
static const float TWO_PI_F      = 6.283185307179586f;
static const float HALF_PI_F     = 1.570796326794896f;
static const float INV_TWO_PI_F  = 0.159154943091895f;

// -----------------------------------------------------------------------------
// Core Math Engines (RAM Accelerated on Spike)
// -----------------------------------------------------------------------------

ACCEL_RAM static float fast_sin_internal(float theta) {
    float x = theta * INV_TWO_PI_F;
    x = theta - (float)((int)(x + (x > 0 ? 0.5f : -0.5f))) * TWO_PI_F;

    if (x > HALF_PI_F) { x = PI_F - x; }
    else if (x < -HALF_PI_F) { x = -PI_F - x; }

    float x2 = x * x;
    float res;

    #if IS_CORTEX_M
        res = -0.000195152f;
        res = 0.008332152f + (x2 * res);
        res = -0.166666567f + (x2 * res);
        res = 1.0f + (x2 * res);
    #else
        res = 1.0f + x2 * (-0.166666567f + x2 * (0.008332152f + x2 * -0.000195152f));
    #endif
    
    return x * res;
}

ACCEL_RAM static float fast_atan2_internal(float y, float x) {
    if (x == 0.0f && y == 0.0f) return 0.0f;
    float abs_y = fabsf(y) + 1e-10f;
    float abs_x = fabsf(x);
    float angle;
    float r = (abs_x >= abs_y) ? (y / x) : (x / y);
    float den = 1.0f + (r * r * 0.28086f);
    float res_atan = r * (1.0f / den);

    if (abs_x >= abs_y) {
        angle = res_atan;
        if (x < 0.0f) angle += (y >= 0.0f) ? PI_F : -PI_F;
    } else {
        angle = (y > 0.0f ? HALF_PI_F : -HALF_PI_F) - res_atan;
    }
    return angle;
}

// -----------------------------------------------------------------------------
// Wrappers & Hardware Benchmarks
// -----------------------------------------------------------------------------

static mp_obj_t experimental_sin(mp_obj_t theta_in) {
    return mp_obj_new_float_from_f(fast_sin_internal(mp_obj_get_float(theta_in)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(experimental_sin_obj, experimental_sin);

static mp_obj_t experimental_cos(mp_obj_t theta_in) {
    return mp_obj_new_float_from_f(fast_sin_internal(mp_obj_get_float(theta_in) + HALF_PI_F));
}
static MP_DEFINE_CONST_FUN_OBJ_1(experimental_cos_obj, experimental_cos);

static mp_obj_t experimental_atan2(mp_obj_t y_in, mp_obj_t x_in) {
    return mp_obj_new_float_from_f(fast_atan2_internal(mp_obj_get_float(y_in), mp_obj_get_float(x_in)));
}
static MP_DEFINE_CONST_FUN_OBJ_2(experimental_atan2_obj, experimental_atan2);

static mp_obj_t experimental_benchmark_hardware(mp_obj_t seed_in) {
    float seed = mp_obj_get_float(seed_in);
    
    #if IS_CORTEX_M
        // Spike Prime High-Res Cycle Counter
        DEMCR |= 0x01000000; DWT_CONTROL |= 1;
        DWT_CYCCNT = 0;
        uint32_t start = DWT_CYCCNT;
        volatile float res = fast_sin_internal(seed);
        res = fast_sin_internal(res + 0.01f); 
        __asm volatile ("dsb"); 
        return mp_obj_new_int((DWT_CYCCNT - start) / 2);
    #else
        // EV3 / Generic: Microsecond-based average
        uint32_t t0 = mp_hal_ticks_ms();
        volatile float res = seed;
        int loops = 50000;
        for(int i=0; i<loops; i++) {
            res = fast_sin_internal(res + 0.0001f);
        }
        uint32_t dt_ms = mp_hal_ticks_ms() - t0;
        // Return result scaled as "nanoseconds" to match Python expected format
        return mp_obj_new_int((dt_ms * 1000000) / loops);
    #endif
}
static MP_DEFINE_CONST_FUN_OBJ_1(experimental_benchmark_hardware_obj, experimental_benchmark_hardware);

static const mp_rom_map_elem_t experimental_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),             MP_ROM_QSTR(MP_QSTR_experimental) },
    { MP_ROM_QSTR(MP_QSTR_sin),                  MP_ROM_PTR(&experimental_sin_obj) },
    { MP_ROM_QSTR(MP_QSTR_cos),                  MP_ROM_PTR(&experimental_cos_obj) },
    { MP_ROM_QSTR(MP_QSTR_atan2),                MP_ROM_PTR(&experimental_atan2_obj) },
    { MP_ROM_QSTR(MP_QSTR_benchmark_hardware),   MP_ROM_PTR(&experimental_benchmark_hardware_obj) },
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