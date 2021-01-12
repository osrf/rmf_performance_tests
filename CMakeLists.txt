cmake_minimum_required(VERSION 3.5.0)

project(rmf_performance_tests)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rmf_traffic REQUIRED)
find_package(rmf_fleet_adapter REQUIRED)

include(GNUInstallDirs)

file(GLOB_RECURSE traffic_editor_paths "maps/*.building.yaml")

set(OUTPUT_MAP_DIR "${CMAKE_CURRENT_BINARY_DIR}/maps")

foreach(path ${traffic_editor_paths})

  # Get the output world name
  string(REGEX REPLACE "\\.[^.]*\.[^.]*$" "" no_extension_path ${path})
  string(REGEX MATCH "[^\/]+$" world_name  ${no_extension_path})

  set(map_path ${path})
  set(output_world_name ${world_name})
  set(output_dir ${OUTPUT_MAP_DIR}/${output_world_name})

  # generate the nav graphs
  set(output_nav_graphs_dir ${output_dir}/nav_graphs/)
  set(output_nav_graphs_phony ${output_nav_graphs_dir}/phony)
  add_custom_command(
    OUTPUT ${output_nav_graphs_phony}
    COMMAND ros2 run building_map_tools building_map_generator nav ${map_path} ${output_nav_graphs_dir}
    DEPENDS ${map_path}
  )

  add_custom_target(generate_${output_world_name}_nav_graphs ALL
    DEPENDS ${output_nav_graphs_phony}
  )

endforeach()

add_executable(test_planner test_planner.cpp)

target_link_libraries(
  test_planner
  PRIVATE
    rmf_traffic::rmf_traffic
    rmf_fleet_adapter::rmf_fleet_adapter
)

# TODO(MXG): If we ever want to distribute this test application in a debian
# package, we will need to be more sophisticated about how we manage these files
target_compile_definitions(
  test_planner
  PRIVATE
    "TEST_MAP_DIR=\"${OUTPUT_MAP_DIR}\""
)

ament_package()