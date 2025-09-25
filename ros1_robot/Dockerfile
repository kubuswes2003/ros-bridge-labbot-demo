# ROS Noetic - kompatybilny z ros1_bridge
FROM osrf/ros:noetic-desktop

# Instalujemy wymagane pakiety
RUN apt-get update && apt-get install -y \
    python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# Tworzymy prostą strukturę (bez catkin workspace)
RUN mkdir -p /robot_ws
WORKDIR /robot_ws

# Kopiujemy Python script
COPY src/mock_robot.py /robot_ws/mock_robot.py

# Nadajemy uprawnienia
RUN chmod +x /robot_ws/mock_robot.py

# Ustawiamy środowisko ROS
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Expo port dla ROS master
EXPOSE 11311

# Domyślna komenda
CMD ["bash", "-c", "source /opt/ros/noetic/setup.bash && roscore"]