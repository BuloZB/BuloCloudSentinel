load("@rules_python//python:defs.bzl", "py_binary", "py_library", "py_test")

package(default_visibility = ["//visibility:public"])

exports_files(["requirements.txt"])

py_library(
    name = "sim_lib",
    srcs = glob(["lib/**/*.py"]),
    deps = [
        "@sim_py_deps//:pkg",
    ],
)

py_binary(
    name = "spawn_world",
    srcs = ["scripts/spawn_world.py"],
    deps = [
        ":sim_lib",
        "@sim_py_deps//:pkg",
    ],
)

py_test(
    name = "sim_tests",
    srcs = glob(["tests/**/*.py"]),
    deps = [
        ":sim_lib",
        "@sim_py_deps//:pkg",
    ],
)

# C++ libraries for ROS 2 and Gazebo integration
cc_library(
    name = "sim_bridge_lib",
    srcs = glob(["ros2_ws/src/sim_bridge/src/**/*.cpp"]),
    hdrs = glob(["ros2_ws/src/sim_bridge/include/**/*.hpp"]),
    includes = ["ros2_ws/src/sim_bridge/include"],
    deps = [
        "@gz_transport//:gz_transport",
        "@gz_msgs//:gz_msgs",
        "@ros2//:rclcpp",
        "@ros2//:std_msgs",
        "@ros2//:geometry_msgs",
        "@ros2//:sensor_msgs",
        "@ros2//:nav_msgs",
        "@ros_gz//:ros_gz_bridge",
    ],
)

cc_library(
    name = "sim_control_lib",
    srcs = glob(["ros2_ws/src/sim_control/src/**/*.cpp"]),
    hdrs = glob(["ros2_ws/src/sim_control/include/**/*.hpp"]),
    includes = ["ros2_ws/src/sim_control/include"],
    deps = [
        ":sim_bridge_lib",
        "@gz_sim//:gz_sim",
        "@ros2//:rclcpp",
        "@ros2//:std_msgs",
        "@ros2//:geometry_msgs",
        "@ros2//:sensor_msgs",
        "@ros2//:nav_msgs",
    ],
)

cc_test(
    name = "sim_bridge_tests",
    srcs = glob(["ros2_ws/src/sim_bridge/test/**/*.cpp"]),
    deps = [
        ":sim_bridge_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "sim_control_tests",
    srcs = glob(["ros2_ws/src/sim_control/test/**/*.cpp"]),
    deps = [
        ":sim_control_lib",
        "@com_google_googletest//:gtest_main",
    ],
)
