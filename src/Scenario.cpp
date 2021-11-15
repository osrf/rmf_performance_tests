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
#include <rmf_fleet_adapter/agv/parse_graph.hpp>
#include <rmf_traffic/geometry/Circle.hpp>

#include <iostream>

bool rmf_performance_tests::scenario::load(
  std::string file_name,
  YAML::Node& node)
{
  std::cout << "Trying to load scenario file [" + file_name + "]" << std::endl;
  try
  {
    node = YAML::LoadFile(std::string(file_name));
  }
  catch (YAML::BadFile& e)
  {
    std::cout <<"Failed to load scenario file [" + file_name + "]" << std::endl;
    return false;
  }
  return true;
}

bool rmf_performance_tests::scenario::load_graph(
  std::string file_name,
  rmf_traffic::agv::VehicleTraits traits,
  rmf_traffic::agv::Graph& graph)
{
  std::cout << "Trying to load map [" + file_name + "]" << std::endl;
  try
  {
    graph = rmf_fleet_adapter::agv::parse_graph(file_name, traits);
  }
  catch (YAML::BadFile& e)
  {
    std::cout <<"Failed to load map [" + file_name + "]" << std::endl;
    return false;
  }
  return true;
}

void rmf_performance_tests::scenario::parse(
  std::string scenario_file,
  Description& description)
{
  const std::string key_samples = "samples";
  const std::string key_robots = "robots";
  const std::string key_limits = "limits";
  const std::string key_linear = "linear";
  const std::string key_angular = "angular";
  const std::string key_velocity = "velocity";
  const std::string key_acceleration = "acceleration";
  const std::string key_profile = "profile";
  const std::string key_footprint = "footprint";
  const std::string key_shape = "shape";
  const std::string key_radius = "radius";
  const std::string key_graph = "graph";
  const std::string key_obstacles = "obstacles";
  const std::string key_robot = "robot";
  const std::string key_trajectory = "trajectory";
  const std::string key_time = "time";
  const std::string key_position = "position";
  const std::string key_map = "map";
  const std::string key_start = "start";
  const std::string key_goal = "goal";
  const std::string key_initial_time = "initial_time";
  const std::string key_initial_waypoint = "initial_waypoint";
  const std::string key_initial_orientation = "initial_orientation";
  const std::string key_plan = "plan";

  YAML::Node scenario_config;
  if (load(scenario_file, scenario_config))
  {
    std::cout << "Loaded scenario file [" + scenario_file + "]" << std::endl;
  }
  else
  {
    std::string path = std::string(TEST_SCENARIO_DIR);
    if (path.at(path.length() - 1) != '/')
    {
      path.append("/");
    }
    if (scenario_file.rfind(".yaml") == std::string::npos)
    {
      scenario_file.append(".yaml");
    }
    if (load(path + scenario_file, scenario_config))
    {
      std::cout << "Loaded scenario file [" + path + scenario_file + "]" <<
        std::endl;
    }
    else
    {
      throw std::runtime_error("Scenario file does not exist");
    }
  }

  if (scenario_config[key_samples])
  {
    description.samples = scenario_config[key_samples].as<std::size_t>();
  }
  else
  {
    description.samples = 100;
    std::cout <<
      "Scenario file is missing the [samples] key. Using default value [100]" <<
      std::endl;
  }

  std::unordered_set<std::string> graph_required;

  const YAML::Node obstacles = scenario_config[key_obstacles];
  for (const auto& obstacle : obstacles)
  {
    if (!obstacle[key_robot])
    {
      throw YAML::ParserException(
              obstacle.Mark(),
              "Obstacle is missing [robot] key.");
    }

    const std::string& name = obstacle[key_robot].as<std::string>();

    std::size_t initial_time;
    double initial_orientation;
    std::string initial_waypoint;

    const auto& yaml_trajectory = obstacle[key_trajectory];
    if (yaml_trajectory)
    {
      rmf_traffic::Trajectory trajectory;
      for (const auto& wp : yaml_trajectory)
      {
        const double time = wp[key_time].as<double>();
        const auto& p = wp[key_position];
        const Eigen::Vector3d position(
          p[0].as<double>(), p[1].as<double>(), p[2].as<double>());

        const auto& v = wp[key_velocity];
        const Eigen::Vector3d velocity(
          v[0].as<double>(), v[1].as<double>(), v[2].as<double>());

        trajectory.insert(
          rmf_traffic::Time(rmf_traffic::time::from_seconds(time)),
          position, velocity);
      }

      const std::string map = obstacle[key_map].as<std::string>();
      description.obstacle_routes.push_back({name, {map, trajectory}});
    }
    else
    {
      graph_required.insert(name);
      const auto& start = obstacle[key_start];
      if (start)
      {
        const auto& time = start[key_initial_time];
        if (time)
        {
          initial_time = time.as<std::size_t>();
        }
        else
        {
          initial_time = 0;
          std::cout << "Obstacle [" << name <<
            "] is missing key [start[initial_time]]. Using default value [0]."
                    <<
            std::endl;
        }
        const auto& waypoint = start[key_initial_waypoint];
        if (waypoint)
        {
          initial_waypoint = waypoint.as<std::string>();
        }
        else
        {
          throw YAML::ParserException(
                  start.Mark(),
                  "Obstacle [" + name + "] is missing key [" + key_initial_waypoint +
                  "]");
        }
        const auto& orientation = start[key_initial_orientation];
        if (orientation)
        {
          initial_orientation = orientation.as<double>();
        }
        else
        {
          initial_orientation = 0;
          std::cout << "Robot [" << name <<
            "] is missing key [start[initial_orientation]]. Assuming initial_orientation 0."
                    << std::endl;
        }
      }
      else
      {
        throw YAML::ParserException(
                obstacle.Mark(),
                "Obstacle [" + name + "] is missing key [" + key_start + "]");
      }

      const auto& goal = obstacle[key_goal];
      if (goal)
      {
        description.obstacle_plans.push_back({name, initial_time,
            initial_orientation,
            initial_waypoint, goal.as<std::string>()});
      }
      else
      {
        throw YAML::ParserException(
                obstacle.Mark(),
                "Obstacle [" + name + "] is missing key [" + key_goal + "]");
      }
    }
  }

  const YAML::Node plan = scenario_config[key_plan];
  if (plan)
  {
    const auto& robot = plan[key_robot];
    if (robot)
    {
      description.plan.robot = robot.as<std::string>();
      graph_required.insert(description.plan.robot);
    }
    else
    {
      throw YAML::ParserException(
              plan.Mark(),
              "Plan is missing key [" + key_robot + "]");
    }
    const auto& start = plan[key_start];
    if (start)
    {
      const auto& time = start[key_initial_time];
      if (time)
      {
        description.plan.initial_time = time.as<std::size_t>();
      }
      else
      {
        description.plan.initial_time = 0;
        std::cout <<
          "Plan is missing key [start[initial_time]]. Using default value [0]."
                  << std::endl;
      }
      const auto& waypoint = start[key_initial_waypoint];
      if (waypoint)
      {
        description.plan.initial_waypoint = waypoint.as<std::string>();
      }
      else
      {
        throw YAML::ParserException(
                start.Mark(),
                "Plan is missing key [" + key_initial_waypoint + "]");
      }
      const auto& orientation = start[key_initial_orientation];
      if (orientation)
      {
        description.plan.initial_orientation = orientation.as<double>();
      }
      else
      {
        description.plan.initial_orientation = 0;
        std::cout <<
          "Plan is missing key [start[initial_orientation]]. Assuming initial_orientation 0."
                  << std::endl;
      }
    }
    else
    {
      throw YAML::ParserException(
              plan.Mark(),
              "Plan is missing key [" + key_start + "]");
    }

    const auto&  goal = plan[key_goal];
    if (goal)
    {
      description.plan.goal = goal.as<std::string>();
    }
    else
    {
      throw YAML::ParserException(
              plan.Mark(),
              "Plan is missing key [" + key_goal + "]");
    }
  }
  else
  {
    throw YAML::ParserException(
            scenario_config.Mark(),
            "Scenario file is missing key [" + key_plan + "]");
  }

  const YAML::Node robots = scenario_config[key_robots];

  for (auto iter = robots.begin(); iter != robots.end(); ++iter)
  {
    const auto& name = iter->first.as<std::string>();
    const auto& robot = iter->second;

    double linear_velocity, linear_acceleration, angular_velocity,
      angular_acceleration;

    const auto& limits = robot[key_limits];
    if (limits)
    {
      const auto& linear = limits[key_linear];
      const auto& angular = limits[key_angular];

      if (linear)
      {
        const auto& velocity = linear[key_velocity];
        const auto& acceleration = linear[key_acceleration];

        if (velocity)
        {
          linear_velocity = velocity.as<double>();
        }
        else
        {
          throw YAML::ParserException(
                  linear.Mark(),
                  "Robot [" + name + "] is missing key [" + key_velocity + "]");
        }

        if (acceleration)
        {
          linear_acceleration = acceleration.as<double>();
        }
        else
        {
          throw YAML::ParserException(
                  linear.Mark(),
                  "Robot [" + name + "] is missing key [" + key_acceleration +
                  "]");
        }
      }
      else
      {
        throw YAML::ParserException(
                limits.Mark(),
                "Robot [" + name + "] is missing key [" + key_linear + "]");
      }

      if (angular)
      {
        const auto& velocity = angular[key_velocity];
        const auto& acceleration = angular[key_acceleration];

        if (velocity)
        {
          angular_velocity = velocity.as<double>();
        }
        else
        {
          throw YAML::ParserException(
                  angular.Mark(),
                  "Robot [" + name + "] is missing key [" + key_velocity + "]");
        }

        if (acceleration)
        {
          angular_acceleration = acceleration.as<double>();
        }
        else
        {
          throw YAML::ParserException(
                  angular.Mark(),
                  "Robot [" + name + "] is missing key [" + key_acceleration +
                  "]");
        }
      }
      else
      {
        throw YAML::ParserException(
                limits.Mark(),
                "Robot [" + name + "] is missing key [" + key_angular + "]");
      }
    }
    else
    {
      throw YAML::ParserException(
              robot.Mark(),
              "Robot [" + name + "] is missing key [" + key_limits + "]");
    }

    const auto& profile = robot[key_profile];
    if (profile)
    {
      const auto& footprint = profile[key_footprint];
      if (footprint)
      {
        const auto& shape = footprint[key_shape];
        if (shape)
        {
          if (strcasecmp("circle", shape.as<std::string>().c_str()) == 0)
          {
            const auto& radius = footprint[key_radius];
            if (radius)
            {
              const auto& traits = rmf_traffic::agv::VehicleTraits {
                {linear_velocity, linear_acceleration},
                {angular_velocity, angular_acceleration},
                rmf_traffic::Profile{
                  rmf_traffic::geometry::make_final_convex(
                    rmf_traffic::geometry::Circle(radius.as<double>()))
                }};

              if (graph_required.find(name) == graph_required.end())
              {
                description.robots.insert({
                    name,
                    {
                      {},
                      traits
                    }});
                continue;
              }

              rmf_traffic::agv::Graph graph;

              if (robot[key_graph])
              {
                auto map_file = robot[key_graph].as<std::string>();
                if (load_graph(map_file, traits, graph))
                {
                  std::cout << "Loaded map [" + map_file + "]" << std::endl;
                }
                else
                {
                  std::string path = std::string(TEST_SCENARIO_DIR);
                  if (path.at(path.length() - 1) != '/')
                  {
                    path.append("/");
                  }
                  if (scenario_file.rfind(".yaml") == std::string::npos)
                  {
                    scenario_file.append(".yaml");
                  }
                  if (load_graph(path + map_file, traits, graph))
                  {
                    std::cout << "Loaded map [" + path + map_file + "]" <<
                      std::endl;
                  }
                  else
                  {
                    path = std::string(TEST_MAP_DIR);
                    if (path.at(path.length() - 1) != '/')
                    {
                      path.append("/");
                    }
                    if (map_file.rfind(".yaml") == std::string::npos)
                    {
                      map_file.append(".yaml");
                    }
                    if (load_graph(path + map_file, traits, graph))
                    {
                      std::cout << "Loaded map [" + path + map_file + "]" <<
                        std::endl;
                    }
                    else
                    {
                      throw std::runtime_error(
                              "Map [" + map_file + "] does not exist");
                    }
                  }
                }
              }
              else
              {
                throw YAML::ParserException(
                        robot.Mark(),
                        "Robot [" + name + "] is missing key [" + key_graph +
                        "]");
              }

              description.robots.insert({
                  name,
                  {
                    graph,
                    traits
                  }});
            }
            else
            {
              throw YAML::ParserException(
                      footprint.Mark(),
                      "Robot [" + name + "] is missing key [" + key_radius +
                      "]");
            }
          }
          else
          {
            throw YAML::ParserException(
                    shape.Mark(),
                    "Robot [" + name + "] has unsupported shape value [" + shape.as<std::string>() +
                    "].");
          }
        }
        else
        {
          throw YAML::ParserException(
                  footprint.Mark(),
                  "Robot [" + name + "] is missing key [" + key_shape + "]");
        }
      }
      else
      {
        throw YAML::ParserException(
                profile.Mark(),
                "Robot [" + name + "] is missing key [" + key_footprint + "]");
      }
    }
    else
    {
      throw YAML::ParserException(
              robot.Mark(),
              "Robot [" + name + "] is missing key [" + key_profile + "]");
    }
  }
}
