#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include "utils.hpp"
#include <gcem.hpp>

static constexpr double relative_gate = -10.0;
static constexpr double minus_twenty_decibels = gcem::pow(10.0, -20.0 / 10.0);
static constexpr double relative_gate_factor = gcem::pow(10.0, relative_gate / 10.0);
static constexpr double absolute_gate = loudnessToEnergy(-70.0);

#endif // CONSTANTS_HPP
