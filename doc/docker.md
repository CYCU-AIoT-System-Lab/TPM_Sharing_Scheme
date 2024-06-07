# Docker Commands

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
