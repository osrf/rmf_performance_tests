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
    COMMAND ros2 run rmf_building_map_tools building_map_generator nav ${map_path} ${output_nav_graphs_dir}
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
    rmf_performance_tests
)

target_include_directories(
    test_planner
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_library(rmf_performance_tests SHARED src/rmf_performance_tests.cpp src/Scenario.cpp)

target_link_libraries(
    rmf_performance_tests
    PRIVATE
    rmf_traffic::rmf_traffic
    rmf_fleet_adapter::rmf_fleet_adapter
)

target_include_directories(
    rmf_performance_tests
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# TODO(MXG): If we ever want to distribute this test application in a debian
# package, we will need to be more sophisticated about how we manage these files
target_compile_definitions(
  test_planner
  PRIVATE
    "TEST_MAP_DIR=\"${OUTPUT_MAP_DIR}\""
    "TEST_SCENARIO_DIR=\"${CMAKE_CURRENT_BINARY_DIR}/scenarios/\""
)

target_compile_definitions(
    rmf_performance_tests
    PRIVATE
    "TEST_MAP_DIR=\"${OUTPUT_MAP_DIR}\""
    "TEST_SCENARIO_DIR=\"${CMAKE_CURRENT_BINARY_DIR}/scenarios/\""
)

file(GLOB_RECURSE scenarios "scenarios/*.yaml")

foreach(scenario ${scenarios})

  get_filename_component(scenario_file ${scenario} NAME)
  configure_file(${scenario} "${CMAKE_CURRENT_BINARY_DIR}/scenarios/${scenario_file}" COPYONLY)

endforeach()


install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

ament_export_targets(rmf_performance_tests HAS_LIBRARY_TARGET)

install(
    TARGETS
      rmf_performance_tests
      test_planner
    EXPORT rmf_performance_tests
    RUNTIME DESTINATION lib/rmf_performance_tests
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

ament_package()
