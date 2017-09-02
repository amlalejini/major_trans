#ifndef PABB_CONFIG_H
#define PABB_CONFIG_H

#include "config/config.h"

EMP_BUILD_CONFIG( MajorTransConfig,
  GROUP(DEFAULT_GROUP, "General Settings"),
  VALUE(DEBUG_MODE, bool, false, "Should we output debug information?"),
  VALUE(RANDOM_SEED, int, 0, "Random number seed (0 for based on time)"),
  VALUE(UPDATES, size_t, 100, "Number of generations to run."),
  GROUP(ENVIRONMENT_GROUP, "Environment Settings"),
  VALUE(GRID_WIDTH, size_t, 60, "Width of population grid."),
  VALUE(GRID_HEIGHT, size_t, 60, "Height of population grid."),
  VALUE(MAX_MOD, double, 1.0, "Maximum allowed resource modifier."),
  VALUE(MIN_MOD, double, 0.125, "Minimum allowed resource modifier."),
  VALUE(RESOURCES_PER_UPDATE, double, 1.0, "Maximum allowed resource modifier."),
  VALUE(EXPORT_REWARD, double, 4.0, "Minimum allowed resource modifier.")


)

#endif
