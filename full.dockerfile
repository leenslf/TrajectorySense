# Use ROS 2 Humble desktop full image as the base
FROM osrf/ros:humble-desktop-full

# Set non-interactive frontend to prevent interactive prompts during package installations
ARG DEBIAN_FRONTEND=noninteractive

# Set SHELL to use bash for the following RUN commands
SHELL ["/bin/bash", "-c"]
ENV SHELL=/bin/bash

# Update system and install necessary packages in a single RUN statement to reduce image layers
RUN apt-get update -y && \
    apt-get install -qq -y --no-install-recommends \
        build-essential \
        sudo \
        git \
        cmake \
        python3-pip \
        libx11-xcb1 \
        libxcb-util1 \
        libxcb-xinerama0 \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-ros \
        ros-humble-gazebo-* \
        ros-humble-turtlebot3-msgs \
        ros-humble-turtlebot3 \
        ros-humble-turtlebot3-simulations \
        ros-humble-turtlebot3-gazebo \
        ros-humble-cartographer \
        ros-humble-cartographer-ros \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup && \
    rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install gdown

# Add user, so the files created in the container are not owned by root
ARG USERNAME=ros2user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a group and user with specified UID/GID and add user to sudoers with no password entry
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to the newly created user
USER $USERNAME

# Set the working directory for the user
WORKDIR /home/${USERNAME}

# Source ROS and Gazebo setup files in the user's bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
RUN echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc