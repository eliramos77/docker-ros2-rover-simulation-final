# docker-px4-simulation

## Docker installation
1. Before you install Docker Engine for the first time on a new host machine, you need to set up the Docker repository. Afterward, you can install and update Docker from the repository.
```
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```

2. Install the last version
```bash
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```
4. Verify that Docker has been set up correctly by running a sample container
```bash
sudo docker run hello-world
```

## Only ROS2 Environment

Build the image:

```
docker build -t ros2_foxy:v1 -f <path_to_file>/ros2_foxy.Dockerfile .
docker load
```

Run the container:

```
docker run -it --privileged --env=LOCAL_USER_ID="$(id -u)" -v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY=:0 ros2_foxy:v1 bash
```

##  With PX4 Simulation

Download the MicroXRCEAgent image from the following link: https://www.eprosima.com/index.php/component/ars/repository/eprosima-micro-xrce-dds/eprosima-micro-xrce-dds-2-4-2/ubuntu-xrcedds-suite-v2-4-2-tar-1?format=raw
   and then load it with the following command using the name of the image:
   ```
   docker load -i <image_name>
   ```