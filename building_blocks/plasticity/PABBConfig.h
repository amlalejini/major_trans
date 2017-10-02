#ifndef PABB_CONFIG_H
#define PABB_CONFIG_H

#include "config/config.h"

EMP_BUILD_CONFIG( MajorTransConfig,
  GROUP(DEFAULT_GROUP, "General Settings"),
  VALUE(DEBUG_MODE, bool, false, "Should we output debug information?"),
  VALUE(RANDOM_SEED, int, 2, "Random number seed (0 for based on time)"),
  VALUE(UPDATES, size_t, 100, "Number of generations to run."),
  VALUE(ANCESTOR_FILE, std::string, "ancestor.gp", "File to read for ancestor program."),
  GROUP(ENVIRONMENT_GROUP, "Environment Settings"),
  VALUE(GRID_WIDTH, size_t, 60, "Width of population grid."),
  VALUE(GRID_HEIGHT, size_t, 60, "Height of population grid."),
  VALUE(MAX_MOD, double, 1.0, "Maximum allowed resource modifier."),
  VALUE(MIN_MOD, double, 0.125, "Minimum allowed resource modifier."),
  VALUE(RESOURCES_PER_UPDATE, double, 1.0, "."),
  VALUE(EXPORT_REWARD, double, 32.0, "."),
  VALUE(COST_OF_REPRO, double, 128.0, "."),
  VALUE(FAILED_REPRO_PENALTY, double, 16.0, "."),
  VALUE(HW_MAX_CORES, size_t, 8, "."),
  VALUE(HW_MAX_CALL_DEPTH, size_t, 128, "."),
  VALUE(HW_MIN_BIND_THRESH, double, 0.5, "."),
  VALUE(PROG_MAX_FUNC_CNT, size_t, 4, "."),
  VALUE(PROG_MAX_FUNC_LEN, size_t, 32, "."),
  VALUE(PROG_MAX_ARG_VAL, size_t, 16, "."),
  VALUE(PER_BIT__AFFINITY_FLIP_RATE, double, 0.05, "."),
  VALUE(PER_INST__SUB_RATE, double, 0.005, "."),
  VALUE(PER_FUNC__SLIP_RATE, double, 0.05, "."),
  VALUE(PER_FUNC__FUNC_DUP_RATE, double, 0.05, "."),
  VALUE(PER_FUNC__FUNC_DEL_RATE, double, 0.05, "."),
  VALUE(SYSTEMATICS_INTERVAL, size_t, 100, "."),
  VALUE(POP_SNAPSHOT_INTERVAL, size_t, 100000, "."),
  VALUE(DATA_DIRECTORY, std::string, "./", ".")
)

#endif
