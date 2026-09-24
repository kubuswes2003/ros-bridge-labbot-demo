# Bazujemy na ROS2 Foxy (kompatybilny z ros1_bridge)
FROM osrf/ros:foxy-desktop

# Instalujemy dodatkowe pakiety
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install flask flask-cors


# Tworzymy workspace ROS2
RUN mkdir -p /ros2_ws/src/robot_controller
WORKDIR /ros2_ws

# Kopiujemy kod controllera
COPY src/ /ros2_ws/src/robot_controller/

# Nadajemy uprawnienia
RUN chmod +x /ros2_ws/src/robot_controller/*.py

# Budujemy workspace (colcon build)
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build"

# Ustawiamy środowisko ROS2  
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Domyślna komenda
CMD ["bash"]