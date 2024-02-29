# rclp9

# What is this?

ROS2 minimal client library for Processing. Currently, a minimal publisher,
timer, and callback are available. Other functionalities are under development.
Please note that only linux on x86 / arm is supported at this time.

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