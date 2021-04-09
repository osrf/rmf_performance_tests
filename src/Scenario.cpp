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

#include "rmf_performance_tests/Scenario.hpp"

void rmf_performance_tests::scenario::parse_scenario(std::string scenario_file,
  Scenario& scenario)
{
  if (scenario_file.rfind(".yaml") == std::string::npos)
  {
    scenario_file.append(".yaml");
  }

  YAML::Node scenario_config;

  try
  {
    scenario_config = YAML::LoadFile(std::string(
          TEST_SCENARIO_DIR) + scenario_file);
  }
  catch (YAML::BadFile& e)
  {
    throw std::runtime_error(
            "Failed to load scenario file [" + scenario_file + "]");
  }

  if (scenario_config["samples"])
  {
    scenario.samples = scenario_config["samples"].as<std::size_t>(100);
  }
  else
  {
    std::cout <<
      "Scenario file is missing the [samples] key. Using default value [100]" <<
      std::endl;
  }

  const YAML::Node robots = scenario_config["robots"];

  for (auto iter = robots.begin(); iter != robots.end(); ++iter)
  {
    const auto& name = iter->first.as<std::string>();
    const auto& robot = iter->second;

    double linear_velocity, linear_acceleration, angular_velocity,
      angular_acceleration;

    const auto& limits = robot["limits"];
    if (limits)
    {
      const auto& linear = limits["linear"];
      const auto& angular = limits["angular"];

      if (linear)
      {
        const auto& velocity = linear["velocity"];
        const auto& acceleration = linear["acceleration"];

        if (velocity)
        {
          linear_velocity = velocity.as<double>();
        }
        else
        {
          throw YAML::ParserException(
            velocity.Mark(),
            "Robot [" + name + "] is missing key [velocity]");
          std::cout << "Robot [" << name <<
            "] is missing key [limits[linear[velocity]]]. Skipping entry" <<
            std::endl;
          continue;
        }

        if (acceleration)
        {
          linear_acceleration = acceleration.as<double>();
        }
        else
        {
          std::cout << "Robot [" << name <<
            "] is missing key [limits[linear[accleration]]]. Skipping entry" <<
            std::endl;
          continue;
        }
      }
      else
      {
        std::cout << "Robot [" << name <<
          "] is missing key [limits[linear]]. Skipping entry" << std::endl;
      }

      if (angular)
      {
        const auto& velocity = angular["velocity"];
        const auto& acceleration = angular["acceleration"];

        if (velocity)
        {
          angular_velocity = velocity.as<double>();
        }
        else
        {
          std::cout << "Robot [" << name <<
            "] is missing key [limits[angular[velocity]]]. Skipping entry" <<
            std::endl;
          continue;
        }

        if (acceleration)
        {
          angular_acceleration = acceleration.as<double>();
        }
        else
        {
          std::cout << "Robot [" << name <<
            "] is missing key [limits[angular[accleration]]]. Skipping entry" <<
            std::endl;
          continue;
        }
      }
      else
      {
        std::cout << "Robot [" << name <<
          "] is missing key [limits[angular]]. Skipping entry" << std::endl;
      }
    }
    else
    {
      std::cout << "Robot [" << name <<
        "] is missing key [limits]. Skipping entry" << std::endl;
      continue;
    }

    const auto& profile = robot["profile"];
    if (profile)
    {
      const auto& footprint = profile["footprint"];
      if (footprint)
      {
        const auto& shape = footprint["shape"];
        if (shape)
        {
          if (strcasecmp("circle", shape.as<std::string>().c_str()) == 0)
          {
            const auto& radius = footprint["radius"];
            if (radius)
            {
              const auto& traits = rmf_traffic::agv::VehicleTraits {
                {linear_velocity, linear_acceleration},
                {angular_velocity, angular_acceleration},
                rmf_traffic::Profile{
                  rmf_traffic::geometry::make_final_convex(
                    rmf_traffic::geometry::Circle(radius.as<double>()))
                }};

              rmf_traffic::agv::Graph graph;

              if (robot["graph"])
              {
                try
                {
                  graph =
                    rmf_fleet_adapter::agv::parse_graph(std::string(
                        TEST_MAP_DIR) + robot["graph"].as<std::string>(),
                      traits);
                }
                catch (YAML::BadFile& e)
                {
                  std::cout << "Failed to load map file [" << std::string(
                      TEST_MAP_DIR) + robot["graph"].as<std::string>() << "]" <<
                    std::endl;
                  continue;
                }
              }
              else
              {
                std::cout << "Robot [" << name <<
                  "] is missing key [graph]. Skipping entry" <<
                  std::endl;
                continue;
              }

              scenario.robots.insert({
                  name,
                  {
                    graph,
                    traits
                  }});
            }
            else
            {
              std::cout << "Robot [" << name <<
                "] is missing key [profile[footprint[radius]]]. Skipping entry"
                        << std::endl;
              continue;
            }
          }
          else
          {
            std::cout << "Robot [" << name <<
              "] has unsupported shape value [" << shape.as<std::string>() <<
              "]. Skipping entry" << std::endl;
            continue;
          }
        }
        else
        {
          std::cout << "Robot [" << name <<
            "] is missing key [profile[footprint[shape]]]. Skipping entry" <<
            std::endl;
          continue;
        }
      }
      else
      {
        std::cout << "Robot [" << name <<
          "] is missing key [profile[footprint]]. Skipping entry" << std::endl;
        continue;
      }
    }
    else
    {
      std::cout << "Robot [" << name <<
        "] is missing key [profile]. Skipping entry" << std::endl;
      continue;
    }
  }

  const YAML::Node obstacles = scenario_config["obstacles"];
  for (const auto& obstacle : obstacles)
  {
    if (!obstacle["robot"])
    {
      std::cout << "Missing [robot] key. Skipping entry" << std::endl;
      continue;
    }

    const std::string& name = obstacle["robot"].as<std::string>();

    std::size_t initial_time;
    double initial_orientation;
    std::string initial_waypoint;

    const auto& yaml_trajectory = obstacle["trajectory"];
    if (yaml_trajectory)
    {
      rmf_traffic::Trajectory trajectory;
      for (const auto& wp : yaml_trajectory)
      {
        const double time = wp["time"].as<double>();
        const auto& p = wp["position"];
        const Eigen::Vector3d position(
          p[0].as<double>(), p[1].as<double>(), p[2].as<double>());

        const auto& v = wp["velocity"];
        const Eigen::Vector3d velocity(
          v[0].as<double>(), v[1].as<double>(), v[2].as<double>());

        trajectory.insert(
          rmf_traffic::Time(rmf_traffic::time::from_seconds(time)),
          position, velocity);
      }

      const std::string map = obstacle["map"].as<std::string>();
      scenario.obstacle_routes.push_back({name, {map, trajectory}});
    }
    else
    {
      const auto& start = obstacle["start"];
      if (start)
      {
        const auto& time = start["initial_time"];
        if (time)
        {
          initial_time = time.as<std::size_t>(0);
        }
        else
        {
          std::cout << "Robot [" << name <<
            "] is missing key [start[initial_time]]. Using default value [0]." <<
            std::endl;
        }
        const auto& waypoint = start["initial_waypoint"];
        if (waypoint)
        {
          initial_waypoint = waypoint.as<std::string>();
        }
        else
        {
          std::cout << "Robot [" << name <<
            "] is missing key [start[initial_waypoint]]. Skipping entry." <<
            std::endl;
          continue;
        }
        const auto& orientation = start["initial_orientation"];
        if (orientation)
        {
          initial_orientation = waypoint.as<double>(0);
        }
        else
        {
          std::cout << "Robot [" << name <<
            "] is missing key [start[initial_orientation]]. Assuming initial_orientation 0."
                    << std::endl;
        }
      }
      else
      {
        std::cout << "Robot [" << name <<
          "] is missing key [start]. Skipping entry." << std::endl;
        continue;
      }

      const auto& goal = obstacle["goal"];
      if (goal)
      {
        scenario.obstacle_plans.push_back({name, initial_time, initial_orientation,
            initial_waypoint, goal.as<std::string>()});
      }
      else
      {
        std::cout << "Robot [" << name <<
          "] is missing key [goal]. Skipping entry." << std::endl;
        continue;
      }
    }
  }

  const YAML::Node plan = scenario_config["plan"];
  if (plan)
  {
    const auto& robot = plan["robot"];
    if (robot)
    {
      scenario.plan.robot = robot.as<std::string>();
    }
    else
    {
      throw std::runtime_error("Scenario file is missing the [plan[robot]] key");
    }
    const auto& start = plan["start"];
    if (start)
    {
      const auto& time = start["initial_time"];
      if (time)
      {
        scenario.plan.initial_time = time.as<std::size_t>(0);
      }
      else
      {
        std::cout <<
          "Plan is missing key [start[initial_time]]. Using default value [0]."
                  <<
          std::endl;
      }
      const auto& waypoint = start["initial_waypoint"];
      if (waypoint)
      {
        scenario.plan.initial_waypoint = waypoint.as<std::string>();
      }
      else
      {
        throw std::runtime_error(
                "Plan is missing [start[initial_waypoint]] key.");
      }
      const auto& orientation = start["initial_orientation"];
      if (orientation)
      {
        scenario.plan.initial_orientation = waypoint.as<double>(0);
      }
      else
      {
        std::cout <<
          "Plan is missing key [start[initial_orientation]]. Assuming initial_orientation 0."
                  << std::endl;
      }
    }
    else
    {
      throw std::runtime_error("Plan is missing [start] key.");
    }

    const auto& goal = plan["goal"];
    if (goal)
    {
      scenario.plan.goal = goal.as<std::string>();
    }
    else
    {
      throw std::runtime_error("Plan is missing [goal] key.");
    }
  }
  else
  {
    throw std::runtime_error("Scenario file is missing the [plan] key");
  }
}
