# rclp9

[![Build Status](https://github.com/sayakaakioka/rclp9/actions/workflows/test-all.yml/badge.svg?branch=main)](https://github.com/sayakaakioka/rclp9/actions/workflows/test-all.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

## Overview

**rclp9** is a minimal ROS 2 client library for [Processing](https://processing.org/), providing a lightweight interface between Processing sketches and ROS2 nodes.  
It currently supports basic publisher/subscriber communication, timers, and callback functionality, allowing Processing sketches to publish and subscribe to ROS 2 topics seamlessly.

Supported message types include:

- `builtin_interfaces.msg.Time`  
- `std_msgs.msg.Float64`, `std_msgs.msg.Header`, `std_msgs.msg.String`  
- `geometry_msgs.msg.Point`, `geometry_msgs.msg.Pose`, `geometry_msgs.msg.PoseStamped`, `geometry_msgs.msg.Quaternion`, `geometry_msgs.msg.Twist`, `geometry_msgs.msg.Vector3`

Example sketches can be found under  
[`rclp9/src/test/processing`](https://github.com/sayakaakioka/rclp9/tree/main/rclp9/src/test/processing).

This project was inspired by the [ROS2 Java client library](https://github.com/ros2-java/ros2_java).

## Supported Platforms

- **Operating Systems:** Linux (x86_64 and ARM64)
- **ROS2 Distributions:** Humble, Jazzy, and Kilted  
  (Fast DDS `rmw_fastrtps_cpp` is used by default)
- **Java:** JDK 21 (toolchain-managed)
- **Processing:** 4.x series
- **C++:** 17 or newer

## Development and Test Environment

Development and automated testing of **rclp9** are performed using the  
[**ros2-p5-dev**](https://github.com/sayakaakioka/ros2-p5-dev) Docker image family.

These images provide a fully pre-configured environment with:
- ROS2 (Humble / Jazzy / Kilted)
- OpenJDK 21 and Gradle 8.x
- Processing 4.x (portable build)
- Mesa software OpenGL stack with Xvfb for headless execution

Using this image ensures that the native (JNI) components, Processing tests, and ROS2 runtime behave consistently across CI and local environments.  
Direct Docker or DevContainer development is *not* required or recommended for normal use.


## Continuous Integration

This repository is tested automatically on GitHub Actions under multiple configurations:

| ROS 2 Distro | Architecture | Java |
|---------------|---------------|-------|
| Humble | x86_64 | JDK 21 |
| Jazzy | x86_64 | JDK 21 |
| Kilted | x86_64 | JDK 21 |
| Humble | ARM64 | JDK 21 |
| Jazzy | ARM64 | JDK 21 |
| Kilted | ARM64 | JDK 21 |

Each build runs Gradle 8.x–based compilation and JNI testing within a Docker container.  
All tests pass across distros and architectures as of the latest CI run.

## Building from Source

Clone the repository and move into the project directory:

```bash
git clone https://github.com/sayakaakioka/rclp9.git
cd rclp9
```

Build:

```bash
./gradlew clean build
```

Running Tests

All tests are executed via Gradle:

```bash
./gradlew test --info
```

## Installing into Processing

Once built, copy the library to your Processing sketchbook:

```bash
$ cp -r ./rclp9/build/rclp9 ${HOME}/sketchbook/libraries/
```

Then set:

```bash
export LD_LIBRARY_PATH=${HOME}/sketchbook/libraries/rclp9/library
```

Launch the Processing IDE and open one of the example sketches (e.g., `MinimalPublisher` or `TimerExample`).

## License

This project is licensed under the [MIT License](https://img.shields.io/badge/License-MIT-blue.svg)
.