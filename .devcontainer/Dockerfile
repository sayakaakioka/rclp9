#ARG BASE_IMAGE=ghcr.io/sayakaakioka/ros2-p5-dev:kilted
#ARG BASE_IMAGE=ghcr.io/sayakaakioka/ros2-p5-dev:jazzy
ARG BASE_IMAGE=ghcr.io/sayakaakioka/ros2-p5-dev:humble
FROM ${BASE_IMAGE}

ARG USERNAME=ubuntu
ARG RCLP9=/home/${USERNAME}/projects/rclp9 
ARG JDK_VERSION=21

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y --no-install-recommends openjdk-${JDK_VERSION}-jdk git g++ && \
    rm -rf /var/lib/apt/lists/* && \
    arch="$(dpkg --print-architecture)" && \
    jvmpath="/usr/lib/jvm/java-${JDK_VERSION}-openjdk-${arch}" && \
    if [ -d "${jvmpath}" ] && [ ! -e "/usr/lib/jvm/java-${JDK_VERSION}-openjdk" ]; then \
      ln -s "${jvmpath}" "/usr/lib/jvm/java-${JDK_VERSION}-openjdk"; \
    fi

ENV JAVA_HOME=/usr/lib/jvm/java-${JDK_VERSION}-openjdk
ENV PATH="$JAVA_HOME/bin:${PATH}"

ENV RCLP9=${RCLP9}
ENV LD_LIBRARY_PATH=${RCLP9}/rclp9/libs/rclp9:${RCLP9}/rclp9/libs/ros:${RCLP9}/rclp9/libs

SHELL ["/bin/bash", "-lc"]
