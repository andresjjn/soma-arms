# SOMA development and validation image (headless).
# Light on purpose (ros-base, no GUI): it builds the workspace, runs the
# tests and validates the xacro models. For RViz in a window use a desktop
# image such as tiryoh/ros2-desktop-vnc:humble.
FROM ros:humble-ros-base-jammy

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-tf2-ros \
    liburdfdom-tools \
    python3-pytest \
    python3-yaml \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
