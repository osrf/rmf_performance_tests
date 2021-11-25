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

#include <rmf_performance_tests/rmf_performance_tests.hpp>
#include <rmf_performance_tests/Scenario.hpp>

#include <iostream>

#include <yaml-cpp/yaml.h>

int main(int argc, char* argv[])
{
  if (argc < 2)
  {
    std::cout << "Please provide scenario file name" << std::endl;
    return 1;
  }

  rmf_performance_tests::scenario::Description scenario;
  try
  {
    parse(argv[1], scenario);
  }
  catch (std::runtime_error& e)
  {
    std::cout << e.what() << std::endl;
    return 1;
  }

  using namespace std::chrono_literals;

  const auto& plan_robot = scenario.robots.find(scenario.plan.value().robot);
  if (plan_robot == scenario.robots.end())
  {
    std::cout << "Plan robot [" << scenario.plan.value().robot <<
      "]'s limits and profile missing" << std::endl;
    return 1;
  }

  // We'll make some "obstacles" in the environment by planning routes between
  // various waypoints.

  const auto database = std::make_shared<rmf_traffic::schedule::Database>();
  std::vector<rmf_traffic::schedule::Participant> obstacles;

  for (const auto& obstacle : scenario.obstacle_plans)
  {
    const auto& robot = scenario.robots.find(obstacle.robot);

    if (robot == scenario.robots.end())
    {
      std::cout << "Robot [" << obstacle.robot <<
        "] is missing limits / profile / graph. Using limits, profile and graph of plan_robot."
                << std::endl;

      rmf_traffic::agv::Planner planner = rmf_traffic::agv::Planner{
        plan_robot->second,
        {nullptr}
      };

      obstacles.emplace_back(
        rmf_performance_tests::add_obstacle(
          planner, database,
          obstacle.start,
          obstacle.goal
        )
      );
    }
    else
    {
      rmf_traffic::agv::Planner planner = rmf_traffic::agv::Planner{
        robot->second,
        {nullptr}
      };

      obstacles.emplace_back(
        rmf_performance_tests::add_obstacle(
          planner, database,
          obstacle.start,
          obstacle.goal
        )
      );
    }
  }

  for (const auto& obstacle : scenario.obstacle_routes)
  {
    const auto& robot = scenario.robots.at(obstacle.robot);

    obstacles.emplace_back(
      rmf_performance_tests::add_obstacle(
        database,
        robot.vehicle_traits().profile(),
        obstacle.route));
  }

  const auto& plan = scenario.plan.value();

  rmf_performance_tests::test_planner(
    std::to_string(plan.start.waypoint()) + " -> "
    + std::to_string(plan.goal.waypoint()),
    scenario.samples,
    plan_robot->second.graph(), plan_robot->second.vehicle_traits(), database,
    plan.start,
    plan.goal
  );
}
