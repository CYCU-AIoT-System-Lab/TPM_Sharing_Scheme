[#](#) Setup ROS2

For raspberry pi platform, docker is required; for jetson nano, use normal binary installation.

ROS2 Humble is prefered then Iron for longer support (LTS 2027).

## Common Commands

- Show docker images:
    ```bash
    docker images
    ```
- Show docker containers:
    ```bash
    docker ps -a
    ```
- Download a image:
    ```bash
    docker pull <image_name>:<tag>
    ```
- Create a container from image:
    ```bash
    docker run -it --name <container_name> <image_name>
    ```
    ```bash
    docker run -it --name <container_name> <image_name>:<tag>
    ```
- Launch a existed container:
    ```bash
    docker run -it <image_name>
    ```
- Attach to a running container:
    ```bash
    docker exec -it <container_name> /bin/bash
    ```
- Remove all images:
    ```bash
    docker rmi $(docker images -q)
- Remove all containers:
    ```bash
    docker rm $(docker ps -a -q)
    ```
- Remove everything from docker:
    ```bash
    docker system prune -a
    ```

## References

1. [https://docs.ros.org/en/foxy/How-To-Guides/Installing-on-Raspberry-Pi.html](https://docs.ros.org/en/foxy/How-To-Guides/Installing-on-Raspberry-Pi.html) 
2. [https://robotics.stackexchange.com/questions/105772/which-ros2-distro-iron-or-humble](https://robotics.stackexchange.com/questions/105772/which-ros2-distro-iron-or-humble)
3. [https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Binary.html](https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Binary.html)
4. [https://github.com/ros2/ros2/releases](https://github.com/ros2/ros2/releases)
