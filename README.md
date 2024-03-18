# rclp9

# What is this?

ROS2 minimal client library for Processing. Currently, a minimal publisher,
a minimal subscriber, timer, and callback are available.
The message types supported by the current version include:
`builtin_interfaces.msg.Time`,
`std_msgs.msg.Float64`,
`std_msgs.msg.Header`,
`std_msgs.msg.String`,
`geometry_msgs.msg.Point`,
`geometry_msgs.msg.Pose`,
`geometry_msgs.msg.PoseStamped`,
and `geometry_msgs.msg.Quaternion`.
Examples can be found [here](https://github.com/sayakaakioka/rclp9/tree/main/rclp9/src/test/processing).

Other functionalities are under development.
Please note that only linux on x86 / arm is supported at this time.

This library was developed with significant inspiration from
[ROS 2 Java client library](https://github.com/ros2-java/ros2_java).

# Prerequirement

ROS2, JDK-17, C++, Python3, and Processing must be installed beforehand.

# Installation

Clone the repository, and move into the project directory (<project_dir>).

```bash
git clone https://github.com/sayakaakioka/rclp9.git
```

```bash
cd rclp9
```

Set `LD_LIBRARY_PATH` and `JAVA_HOME`. `LD_LIBRARY_PATH` here is used for building the library and running tests. Please note that this `LD_LIBRARY_PATH` setting should be different from the setting used after installation.

```bash
export LD_LIBRARY_PATH=<project_dir>/rclp9/libs/rclp9:<project_dir>/rclp9/libs/ros:<project_dir>/rclp9/libs
export JAVA_HOME=<your_java_home>
```

Build the library.

```bash
./gradlew clean
./gradlew build
```

During the building process, you may happen to see the following message, especially on Ubuntu Linux.
You can safely ignore it.

> "Path for java installation '/usr/lib/jvm/openjdk-17' (Common Linux Locations) does not contain a java executable"

To generate the Javadoc for rclp9, execute the following command.
The documentation will be available under the directory
`rclp9/build/docs/javadoc`.

```bash
./gradlew javadoc
```

Now all the libraries for rclp9 should be ready. To use from Processing, simply copy
all the libraries into the appropriate directory, and set `LD_LIBRARY_PATH`.

```bash
$ cp -r ./rclp9/build/rclp9 ${HOME}/sketchbook/libraries/
```

```bash
export LD_LIBRARY_PATH=${HOME}/sketchbook/libraries/rclp9/library
```

Launch the Processing IDE and enjoy! You can find an example of the minimal publisher and timer in
`rclp9/src/test/processing` directory under the project directory.