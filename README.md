# rmf_performance_tests
Performance tests for RMF

## Installation
```bash
git clone https://github.com/osrf/rmf_performance_tests.git
cd {ros2_ws}
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select=rmf_performance_tests
```

## Testing

```bash
cd {ros2_ws}
./install/rmf_performance_tests/lib/rmf_performance_tests/test_planner {SCENARIO_NAME}
```