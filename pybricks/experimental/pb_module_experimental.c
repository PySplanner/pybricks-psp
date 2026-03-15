// SPDX-License-Identifier: MIT
// Copyright (c) 2018-2021 The Pybricks Authors

#include "py/mpconfig.h"

#if PYBRICKS_PY_EXPERIMENTAL

#include "py/mphal.h"
#include "py/obj.h"
#include "py/objstr.h"
#include "py/runtime.h"
#include "py/mperrno.h"

#include <pbio/util.h>

#include <pybricks/util_mp/pb_obj_helper.h>
#include <pybricks/util_mp/pb_kwarg_helper.h>
#include <pybricks/util_pb/pb_error.h>
#include <pybricks/robotics.h>

#include <math.h>

// Math Constants
static const float PI_F         = 3.1415926535f;
static const float TWO_PI_F     = 6.2831853071f;
static const float HALF_PI_F    = 1.5707963267f;
static const float INV_TWO_PI_F = 0.1591549431f;

// -----------------------------------------------------------------------------
// Internal Math Engines
// -----------------------------------------------------------------------------

// High-Precision Sine (5th-degree Minimax Polynomial)
// This is the core engine for both sin and cos.
static float fast_sin_internal(float theta) {
    // Range reduction to [-PI, PI]
    float quot = theta * INV_TWO_PI_F;
    float x = theta - (float)((int)(quot + (quot > 0 ? 0.5f : -0.5f))) * TWO_PI_F;

    // Symmetry reduction to [-PI/2, PI/2]
    if (x > HALF_PI_F) x = PI_F - x;
    else if (x < -HALF_PI_F) x = -PI_F - x;

    // 5th-degree Minimax Polynomial
    float x2 = x * x;
    return x * (1.0f + x2 * (-0.1666665f + x2 * 0.0083322f));
}

// Atan2 using the 0.273 parabolic approximation (valid for |z| <= 1)
static float fast_atan2_internal(float y, float x) {
    float ay = fabsf(y) + 1e-10f; 
    float ax = fabsf(x);
    float z, angle;

    // Range reduction to ensure we stay on the stable part of the approximation
    if (ax >= ay) {
        z = y / ax;
        angle = (0.7853982f + 0.273f * (1.0f - fabsf(z))) * z;
    } else {
        z = x / ay;
        angle = 1.5707963f - (0.7853982f + 0.273f * (1.0f - fabsf(z))) * z;
    }

    if (x < 0.0f) {
        angle += (y >= 0.0f) ? PI_F : -PI_F;
    }
    return angle;
}

// -----------------------------------------------------------------------------
// MicroPython Functions
// -----------------------------------------------------------------------------

// pybricks.experimental.sin(radians)
static mp_obj_t experimental_sin(mp_obj_t theta_in) {
    float theta = mp_obj_get_float(theta_in);
    return mp_obj_new_float_from_f(fast_sin_internal(theta));
}
static MP_DEFINE_CONST_FUN_OBJ_1(experimental_sin_obj, experimental_sin);

// pybricks.experimental.cos(radians)
static mp_obj_t experimental_cos(mp_obj_t theta_in) {
    float theta = mp_obj_get_float(theta_in);
    return mp_obj_new_float_from_f(fast_sin_internal(theta + HALF_PI_F));
}
static MP_DEFINE_CONST_FUN_OBJ_1(experimental_cos_obj, experimental_cos);

// pybricks.experimental.atan2(y, x)
static mp_obj_t experimental_atan2(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    PB_PARSE_ARGS_FUNCTION(n_args, pos_args, kw_args,
        PB_ARG_REQUIRED(y),
        PB_ARG_REQUIRED(x));

    float y = mp_obj_get_float(y_in);
    float x = mp_obj_get_float(x_in);

    return mp_obj_new_float_from_f(fast_atan2_internal(y, x));
}
static MP_DEFINE_CONST_FUN_OBJ_KW(experimental_atan2_obj, 2, experimental_atan2);

// -----------------------------------------------------------------------------
// Module Registry
// -----------------------------------------------------------------------------

static const mp_rom_map_elem_t experimental_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_experimental) },
    { MP_ROM_QSTR(MP_QSTR_sin),         MP_ROM_PTR(&experimental_sin_obj) },
    { MP_ROM_QSTR(MP_QSTR_cos),         MP_ROM_PTR(&experimental_cos_obj) },
    { MP_ROM_QSTR(MP_QSTR_atan2),       MP_ROM_PTR(&experimental_atan2_obj) },
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