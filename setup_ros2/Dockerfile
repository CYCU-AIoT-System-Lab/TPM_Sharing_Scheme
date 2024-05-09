FROM ros:rolling-ros-core-noble
ARG PACKAGE_NAME
ENV PACKAGE_NAME=${PACKAGE_NAME}
RUN apt update
RUN apt upgrade -y
RUN apt install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-rosdep \
    wget \
    vim
RUN rm -rf /var/lib/apt/lists/*
RUN rosdep init 
RUN rosdep update
RUN echo "source /opt/ros/rolling/setup.bash" >> ~/.bashrc
WORKDIR /ros2_ws
RUN mkdir src && \
    . /opt/ros/rolling/setup.sh && \
    cd src && \
    ros2 pkg create --build-type ament_cmake ${PACKAGE_NAME} && \
    cd ${PACKAGE_NAME} && \
    wget -nv --show-progress -O src/publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/foxy/rclcpp/topics/minimal_publisher/member_function.cpp && \
    wget -nv --show-progress -O src/subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/foxy/rclcpp/topics/minimal_subscriber/member_function.cpp && \
    sed -i '11 i <depend>rclcpp</depend>\n<depend>std_msgs</depend>' package.xml && \
    sed -i '13 i find_package(rclcpp REQUIRED)\nfind_package(std_msgs REQUIRED)' CMakeLists.txt && \
    sed -i '16 i add_executable(talker src/publisher_member_function.cpp)\nadd_executable(listener src/subscriber_member_function.cpp)\nament_target_dependencies(talker rclcpp std_msgs)\nament_target_dependencies(listener rclcpp std_msgs)' CMakeLists.txt && \
    sed -i '19 i install(TARGETS\n\ttalker\n\tlistener\n\tDESTINATION\n\tlib/${PROJECT_NAME})' CMakeLists.txt && \
    rosdep install -i --from-path src --rosdistro rolling -y && \
    colcon build --packages-select ${PACKAGE_NAME} && \
    echo "Build completed."
