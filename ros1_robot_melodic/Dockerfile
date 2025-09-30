# ROS Melodic - jak prawdziwy robot labbot
FROM osrf/ros:melodic-desktop

# Instaluj Python dependencies
RUN apt-get update && apt-get install -y \
    python-pip \
    python-yaml \
    && rm -rf /var/lib/apt/lists/*

# Workspace
RUN mkdir -p /robot_ws
WORKDIR /robot_ws

# Kopiuj kod
COPY src/mock_robot.py /robot_ws/mock_robot.py
RUN chmod +x /robot_ws/mock_robot.py

# Setup ROS
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

EXPOSE 11311

CMD ["bash"]
