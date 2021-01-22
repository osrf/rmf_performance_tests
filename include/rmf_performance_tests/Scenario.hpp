/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef RMF_PERFORMANCE_TESTS_SCENARIO_HPP_
#define RMF_PERFORMANCE_TESTS_SCENARIO_HPP_

#include <iostream>

#include <rmf_traffic/agv/VehicleTraits.hpp>

#include <yaml-cpp/yaml.h>

namespace rmf_performance_tests {
namespace scenario {

struct Plan
{
  std::string robot;
  std::size_t initial_time;
  double initial_orientation;
  std::string initial_waypoint, goal;
};

struct Scenario
{
  std::string map_file;
  std::size_t samples;

  std::unordered_map<std::string, rmf_traffic::agv::VehicleTraits> robots;
  std::vector<Plan> obstacles;

  Plan plan;
};

void parse_scenario(std::string scenario_file, Scenario& scenario);

}
}

#endif //RMF_PERFORMANCE_TESTS_SCENARIO_HPP_
