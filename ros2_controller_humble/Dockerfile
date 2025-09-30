# WYMUSZENIE x86_64 platform dla ARM64 Mac
FROM --platform=linux/amd64 osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /ros2_ws/src/robot_controller
WORKDIR /ros2_ws

COPY src/ /ros2_ws/src/robot_controller/
RUN chmod +x /ros2_ws/src/robot_controller/robot_controller/*.py

RUN bash -c "source /opt/ros/humble/setup.bash && colcon build"

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]