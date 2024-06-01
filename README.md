## Procedure to execute the final Workshop

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

## Fork

1. Fork my repository (docker-ros2-rover-simulation-final)

```
git clone <SSH>
```

## URDF

2. Create an URDF of a rover from any cad software (better Onshape)

## ROS2 Environment

Build the image:

```
docker build -t ros2_foxy_rover:v1 -f <path_to_file>/ros2_foxy.Dockerfile .
```

## Open two terminals in Tilix

Run the container in two terminals:

```
docker run -it --privileged --env=LOCAL_USER_ID="$(id -u)" \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-v "$(pwd)":/src/docker-ros2-rover-simulation \
-e DISPLAY=:0 ros2_foxy_rover:v1 bash
```

### Terminal 1

```
cd src/docker-ros2-rover-simulation/
source install/setup.bash
ros2 launch rover_gz_description gazebo.launch.py 
```
Note: if you have errors, firstly run display.launch.py, then gazebo, and lastly launch the gazebo simulation.

### Terminal 2

```
cd src/docker-ros2-rover-simulation/
source install/setup.bash
ros2 run publisher_velocity publisher_cmd.py
```
Once these commands are running, the publisher will be publishing a linear velocity to the rover.

## Publish the angular or linear velocity.

4. Enter in the publisher node the message needed to publish the angular velocity or the linear velocity to the cmd_vel topic.
