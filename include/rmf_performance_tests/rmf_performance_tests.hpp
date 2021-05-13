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

#ifndef RMF_PERFORMANCE_TESTS_RMF_PERFORMANCE_TESTS
#define RMF_PERFORMANCE_TESTS_RMF_PERFORMANCE_TESTS

#include <rmf_traffic/agv/Planner.hpp>

#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/schedule/Participant.hpp>

namespace rmf_performance_tests {

rmf_traffic::schedule::Participant add_obstacle(
  const rmf_traffic::agv::Planner& planner,
  const std::shared_ptr<rmf_traffic::schedule::Database>& database,
  const rmf_traffic::agv::Plan::Start& start,
  const rmf_traffic::agv::Plan::Goal& goal);

rmf_traffic::schedule::Participant add_obstacle(
  const std::shared_ptr<rmf_traffic::schedule::Database>& database,
  const rmf_traffic::Profile& profile,
  const rmf_traffic::Route& route);

void print_result(
  const std::string& label,
  const std::size_t samples,
  const double total_time,
  const std::size_t node_count);

double test_planner_timing_no_cache(
  const std::string& label,
  const std::size_t samples,
  const rmf_traffic::agv::Planner::Configuration& config,
  const rmf_traffic::agv::Planner::Options& options,
  const rmf_traffic::agv::Plan::Start& start,
  const rmf_traffic::agv::Plan::Goal& goal);

double test_planner_timing_with_cache(
  const std::string& label,
  const std::size_t samples,
  const rmf_traffic::agv::Planner::Configuration& config,
  const rmf_traffic::agv::Planner::Options& options,
  const rmf_traffic::agv::Plan::Start& start,
  const rmf_traffic::agv::Plan::Goal& goal);

void test_planner_timing(
  const std::string& label,
  const std::size_t samples,
  const rmf_traffic::agv::Planner::Configuration& config,
  const rmf_traffic::agv::Planner::Options& options,
  const rmf_traffic::agv::Plan::Start& start,
  const rmf_traffic::agv::Plan::Goal& goal);

void test_planner(
  const std::string& label,
  const std::size_t samples,
  const rmf_traffic::agv::Graph& graph,
  const rmf_traffic::agv::VehicleTraits& traits,
  const std::shared_ptr<rmf_traffic::schedule::Database>& database,
  const rmf_traffic::agv::Plan::Start& start,
  const rmf_traffic::agv::Plan::Goal& goal);

std::string get_map_directory();

}

#endif //RMF_PERFORMANCE_TESTS_RMF_PERFORMANCE_TESTS
