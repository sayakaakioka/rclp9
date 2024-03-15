#!/usr/bin/env python3

import os
import glob
import subprocess
import sys

def main():
    include_ops = "-I./build/generated/sources/headers/java/main"

    # check the ROS2 distribution
    ros = "/opt/ros"
    if os.path.exists("{}/iron".format(ros)):
        ros_dist = "iron"
    elif os.path.exists("{}/humble".format(ros)):
        ros_dist = "humble"
    else:
        print("This ROS2 version is not supported.");
        sys.exit(0)

    ros = "{}/{}".format(ros, ros_dist)


    # ROS2 headers
    ros_include = "{}/include".format(ros);
    if ros_dist == "iron":
        module_include = ["builtin_interfaces",
                          "geometry_msgs",
                          "rcl",
                          "rcl_yaml_param_parser",
                          "rcutils",
                          "rmw",
                          "rosidl_dynamic_typesupport",
                          "rosidl_runtime_c",
                          "rosidl_typesupport_interface",
                          "service_msgs",
                          "std_msgs",
                          "type_description_interfaces"
                          ]
    else:
        module_include = ["geometry_msgs",
                          "rcl",
                          "rcl_yaml_param_parser",
                          "rcutils",
                          "rmw",
                          "rosidl_runtime_c",
                          "rosidl_typesupport_interface",
                          "std_msgs"
        ]

    for target in module_include:
        include_ops = "{} -I{}/{}".format(include_ops, ros_include, target)

    # JNI
    java_home = os.getenv("JAVA_HOME", "/usr/lib/jvm/java-17-openjdk-amd64")

    ret = subprocess.run("uname", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, encoding="utf-8")
    os_name = ret.stdout.replace("\n", "")
    if os_name == "Darwin":
        include_ops = "{} -I{}/include -I{}/include/darwin".format(include_ops, java_home, java_home)
    elif os_name == "Linux":
        include_ops = "{} -I{}/include -I{}/include/linux".format(include_ops, java_home, java_home)
    else:
        print("This operating system is not supported: {}.".format(os_name))
        sys.exit(0)

    # rclp9 native libraries
    rclp9_lib_dir = "./libs/rclp9"
    if not os.path.exists(rclp9_lib_dir):
        os.makedirs(rclp9_lib_dir)

    cpp_src_dir = "./src/main/cpp"
    files = glob.glob(os.path.join(cpp_src_dir, "*.cpp"))

    # ROS2 shared libraries
    ros_lib_dir = "./libs/ros"
    if not os.path.exists(ros_lib_dir):
        os.makedirs(ros_lib_dir)

    ros_org_dir = "{}/lib".format(ros);
    if ros_dist == "iron":
        module_lib = ["ament_index_cpp",
                      "builtin_interfaces__rosidl_generator_c",
                      "builtin_interfaces__rosidl_typesupport_fastrtps_c",
                      "builtin_interfaces__rosidl_typesupport_introspection_c",
                      "fastcdr",
                      "fastrtps",
                      "geometry_msgs__rosidl_generator_c",
                      "geometry_msgs__rosidl_typesupport_c",
                      "geometry_msgs__rosidl_typesupport_fastrtps_c",
                      "geometry_msgs__rosidl_typesupport_introspection_c",
                      "rcl",
                      "rcl_interfaces__rosidl_generator_c",
                      "rcl_interfaces__rosidl_typesupport_c",
                      "rcl_logging_interface",
                      "rcl_logging_spdlog",
                      "rcl_yaml_param_parser",
                      "rcpputils",
                      "rcutils",
                      "rmw",
                      "rmw_dds_common",
                      "rmw_dds_common__rosidl_generator_c",
                      "rmw_dds_common__rosidl_typesupport_cpp",
                      "rmw_dds_common__rosidl_typesupport_fastrtps_cpp",
                      "rmw_dds_common__rosidl_typesupport_introspection_cpp",
                      "rmw_fastrtps_cpp",
                      "rmw_fastrtps_shared_cpp",
                      "rmw_implementation",
                      "rosidl_dynamic_typesupport",
                      "rosidl_dynamic_typesupport_fastrtps",
                      "rosidl_runtime_c",
                      "rosidl_typesupport_c",
                      "rosidl_typesupport_cpp",
                      "rosidl_typesupport_fastrtps_c",
                      "rosidl_typesupport_fastrtps_cpp",
                      "rosidl_typesupport_introspection_c",
                      "rosidl_typesupport_introspection_cpp",
                      "service_msgs__rosidl_generator_c",
                      "std_msgs__rosidl_generator_c",
                      "std_msgs__rosidl_typesupport_c",
                      "std_msgs__rosidl_typesupport_fastrtps_c",
                      "std_msgs__rosidl_typesupport_introspection_c",
                      "tracetools",
                      "type_description_interfaces__rosidl_generator_c",
                      "type_description_interfaces__rosidl_typesupport_c"
                      ]
    else:
        module_lib = ["ament_index_cpp",
                      "builtin_interfaces__rosidl_generator_c",
                      "builtin_interfaces__rosidl_typesupport_fastrtps_c",
                      "builtin_interfaces__rosidl_typesupport_introspection_c",
                      "fastcdr",
                      "fastrtps",
                      "geometry_msgs__rosidl_generator_c",
                      "geometry_msgs__rosidl_typesupport_c",
                      "geometry_msgs__rosidl_typesupport_fastrtps_c",
                      "geometry_msgs__rosidl_typesupport_introspection_c",
                      "rcl",
                      "rcl_interfaces__rosidl_generator_c",
                      "rcl_interfaces__rosidl_typesupport_c",
                      "rcl_logging_interface",
                      "rcl_logging_spdlog",
                      "rcl_yaml_param_parser",
                      "rcpputils",
                      "rcutils",
                      "rmw",
                      "rmw_dds_common",
                      "rmw_dds_common__rosidl_generator_c",
                      "rmw_dds_common__rosidl_typesupport_cpp",
                      "rmw_dds_common__rosidl_typesupport_fastrtps_cpp",
                      "rmw_dds_common__rosidl_typesupport_introspection_cpp",
                      "rmw_fastrtps_cpp",
                      "rmw_fastrtps_shared_cpp",
                      "rmw_implementation",
                      "rosidl_runtime_c",
                      "rosidl_typesupport_c",
                      "rosidl_typesupport_cpp",
                      "rosidl_typesupport_fastrtps_c",
                      "rosidl_typesupport_fastrtps_cpp",
                      "rosidl_typesupport_introspection_c",
                      "rosidl_typesupport_introspection_cpp",
                      "std_msgs__rosidl_generator_c",
                      "std_msgs__rosidl_typesupport_c",
                      "std_msgs__rosidl_typesupport_fastrtps_c",
                      "std_msgs__rosidl_typesupport_introspection_c",
                      "tracetools"
                      ]

    ros_lib_ops = ""
    for target in module_lib:
        ros_lib_ops = "{} -l{}".format(ros_lib_ops, target)
        libname = "{}/lib{}.so*".format(ros_org_dir, target)
        subprocess.run("cp -f {} {}/".format(libname, ros_lib_dir), shell=True)

    # compile and link
    for file in files:
        filename = os.path.splitext(os.path.basename(file))[0]
        subprocess.run("g++ -shared -fPIC {} -o {}/lib{}.so {} -L{} {}"
                       .format(include_ops, rclp9_lib_dir, filename, file, ros_lib_dir, ros_lib_ops), shell=True)

if __name__ == "__main__":
    main()