// SPDX-License-Identifier: MIT
#include "py/mpconfig.h"

#if PYBRICKS_PY_EXPERIMENTAL

#include "py/mphal.h"
#include "py/runtime.h"
#include <math.h>

#include <pybricks/common.h>
#include <pbio/servo.h>
#include "pybricks/experimental/odometry.h"
#include "pybricks/experimental/platform_math.h"

// Hardware Pointers
static pbio_servo_t *left_servo_ptr = NULL;
static pbio_servo_t *right_servo_ptr = NULL;

// Odometry State
volatile bool odom_running = false;
volatile uint32_t last_odom_time_ms = 0;
volatile float global_x = 0.0f, global_y = 0.0f, global_h = 0.0f;
volatile int32_t last_left_angle = 0, last_right_angle = 0;
float odom_deg_to_mm = 1.0f;
float odom_inv_track = 1.0f;

// Pursuit / Spline State
volatile bool pursuit_running = false;
volatile float p_target_speed = 0.0f;
volatile float p_lookahead = 120.0f;

// Spline Coefficients for y = ax^3 + bx^2 + cx + d
volatile float sp_a = 0.0f, sp_b = 0.0f, sp_c = 0.0f, sp_d = 0.0f;
volatile float sp_x_end = 0.0f;

// --- Background Updates ---

void pb_background_odometry_update(void) {
    if (!odom_running) return;

    uint32_t now = mp_hal_ticks_ms();
    if (now - last_odom_time_ms < 5) return;
    last_odom_time_ms = now;

    int32_t cur_l, cur_r, unused_rate;
    pbio_servo_get_state_user(left_servo_ptr, &cur_l, &unused_rate);
    pbio_servo_get_state_user(right_servo_ptr, &cur_r, &unused_rate);

    int32_t delta_l = cur_l - last_left_angle;
    int32_t delta_r = cur_r - last_right_angle;

    if (delta_l != 0 || delta_r != 0) {
        float dL = (float)delta_l * odom_deg_to_mm;
        float dR = (float)delta_r * odom_deg_to_mm;
        float dD = (dR + dL) * 0.5f;
        float dH = (dR - dL) * odom_inv_track;
        float avg_h = global_h + (dH * 0.5f);

        global_x += dD * pb_fast_cos(avg_h);
        global_y += dD * pb_fast_sin(avg_h);
        global_h += dH;

        last_left_angle = cur_l;
        last_right_angle = cur_r;
    }
}

void pb_background_pursuit_update(void) {
    if (!pursuit_running || !left_servo_ptr || !right_servo_ptr) return;

    // 1. PROJECT TARGET POINT
    // Use sp_a, sp_b, sp_c, sp_d and p_lookahead to find target_x and target_y
    //TODO
    //float target_x = 0.0f; 
    //float target_y = 0.0f;

    // 2. CHECK EXIT CONDITION
    // Decide when to set pursuit_running = false (e.g. reaching sp_x_end)

    // 3. PURE PURSUIT MATH
    // Calculate m_left and m_right based on target_x/y relative to global_x/y
    float m_left = 1.0f;
    float m_right = 1.0f;

    // 4. DRIVE
    pbio_servo_run_forever(left_servo_ptr, (int32_t)(p_target_speed * m_left));
    pbio_servo_run_forever(right_servo_ptr, (int32_t)(p_target_speed * m_right));

}

// --- MicroPython API ---

// start_odometry(left_m, right_m, mm_per_deg, track, x, y, h)
mp_obj_t experimental_start_odometry(size_t n_args, const mp_obj_t *args) {
    left_servo_ptr = ((pb_type_Motor_obj_t *)MP_OBJ_TO_PTR(args[0]))->srv;
    right_servo_ptr = ((pb_type_Motor_obj_t *)MP_OBJ_TO_PTR(args[1]))->srv;
    odom_deg_to_mm = mp_obj_get_float(args[2]) / 360.0f;
    odom_inv_track = 1.0f / mp_obj_get_float(args[3]);
    global_x = mp_obj_get_float(args[4]);
    global_y = mp_obj_get_float(args[5]);
    global_h = mp_obj_get_float(args[6]);

    int32_t unused_rate;
    pbio_servo_get_state_user(left_servo_ptr, (int32_t*)&last_left_angle, &unused_rate);
    pbio_servo_get_state_user(right_servo_ptr, (int32_t*)&last_right_angle, &unused_rate);
    odom_running = true;
    return mp_const_none;
}

mp_obj_t experimental_get_odometry(void) {
    mp_obj_t tuple[3] = { mp_obj_new_float_from_f(global_x), mp_obj_new_float_from_f(global_y), mp_obj_new_float_from_f(global_h) };
    return mp_obj_new_tuple(3, tuple);
}

mp_obj_t experimental_stop_odometry(void) {
    odom_running = false;
    return mp_const_none;
}

// NEW API: start_spline(a, b, c, d, x_end, speed, lookahead)
mp_obj_t experimental_start_pursuit(size_t n_args, const mp_obj_t *args) {
    sp_a = mp_obj_get_float(args[0]);
    sp_b = mp_obj_get_float(args[1]);
    sp_c = mp_obj_get_float(args[2]);
    sp_d = mp_obj_get_float(args[3]);
    sp_x_end = mp_obj_get_float(args[4]);
    p_target_speed = mp_obj_get_float(args[5]);
    p_lookahead = mp_obj_get_float(args[6]);

    pursuit_running = true;
    return mp_const_none;
}

mp_obj_t experimental_stop_pursuit(void) {
    pursuit_running = false;
    if (left_servo_ptr && right_servo_ptr) {
        pbio_servo_stop(left_servo_ptr, PBIO_CONTROL_ON_COMPLETION_BRAKE);
        pbio_servo_stop(right_servo_ptr, PBIO_CONTROL_ON_COMPLETION_BRAKE);
    }
    return mp_const_none;
    
}

#endif