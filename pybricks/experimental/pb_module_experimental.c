#include "py/mpconfig.h"
#include "py/obj.h"
#include "py/runtime.h"

#if PYBRICKS_PY_EXPERIMENTAL

// Forward declare the C functions
extern mp_obj_t calculate_odometry(int num_iters, float wheel_circ, float axle_track, mp_obj_t right_motor, mp_obj_t left_motor);
extern mp_obj_t get_pure_pursuit_multipliers(float tx, float ty, float rx_pos, float ry_pos, float rh_ang, float track);

static mp_obj_t experimental_odometry_benchmark(size_t n_args, const mp_obj_t *args) {
    return calculate_odometry(
        mp_obj_get_int(args[0]),
        mp_obj_get_float(args[1]),
        mp_obj_get_float(args[2]),
        args[3], // Raw Python Motor Object (Right)
        args[4]  // Raw Python Motor Object (Left)
    );
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(experimental_odometry_benchmark_obj, 5, 5, experimental_odometry_benchmark);

static mp_obj_t experimental_pure_pursuit_logic(size_t n_args, const mp_obj_t *args) {
    return get_pure_pursuit_multipliers(
        mp_obj_get_float(args[0]), 
        mp_obj_get_float(args[1]), 
        mp_obj_get_float(args[2]), 
        mp_obj_get_float(args[3]), 
        mp_obj_get_float(args[4]), 
        mp_obj_get_float(args[5])
    );
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(experimental_pure_pursuit_logic_obj, 6, 6, experimental_pure_pursuit_logic);

const mp_rom_map_elem_t pb_module_experimental_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),            MP_ROM_QSTR(MP_QSTR_experimental) },
    { MP_ROM_QSTR(MP_QSTR_odometry_benchmark),  MP_ROM_PTR(&experimental_odometry_benchmark_obj) },
    { MP_ROM_QSTR(MP_QSTR_pure_pursuit_logic),  MP_ROM_PTR(&experimental_pure_pursuit_logic_obj) },
};
MP_DEFINE_CONST_DICT(pb_module_experimental_globals, pb_module_experimental_globals_table);

const mp_obj_module_t pb_module_experimental = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&pb_module_experimental_globals,
};

#endif // PYBRICKS_PY_EXPERIMENTAL