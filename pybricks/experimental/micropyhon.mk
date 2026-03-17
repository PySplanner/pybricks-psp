# Always compile the shared Python wrapper
PYBRICKS_SRC_C += pybricks/experimental/pb_module_experimental.c

# Routing logic based on PLATFORM
ifeq ($(PLATFORM), $(filter $(PLATFORM), prime_hub essential_hub technic_hub))
    PYBRICKS_SRC_C += pybricks/experimental/odometry_cortexm4.c
else ifeq ($(PLATFORM), ev3dev)
    PYBRICKS_SRC_C += pybricks/experimental/odometry_arm9.c
endif