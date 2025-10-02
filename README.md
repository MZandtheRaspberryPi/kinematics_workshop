# Kinematics Workshop

To run the code, first download the repo and build the docker file, which will install all the libraries you need to run the activities. This may take a couple minutes to complete.

```
cd $HOME
git clone https://github.com/MZandtheRaspberryPi/kinematics_workshop.git
cd kinematics_workshop
docker build -f docker/Dockerfile -t ros_sim --progress plain .
```

From here, run first the docker container (we pass along some flags to enable graphical interfaces). Then inside of the container (where ROS2 is installed) run the pybullet simulation:
```
docker run --rm -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY -e QT_X11_NO_MITSHM=1 --network host -v $HOME/kinematics_workshop:/home/developer/ros_ws/src/kinematics_workshop ros_sim
cd ros_ws
colcon build --symlink-install
source install/setup.bash
ros2 run ros_sim pybullet_ex
```

In a second terminal, connect to the same running docker container and run the trajectory follower.
```
docker exec -it $(docker ps -lq) /bin/bash
cd ros_ws
source install/setup.bash
ros2 run ros_arm_controller robo_controller --ros-args -p use_sim_time:=true
```

You may open more terminals to allow introspection with:
```
docker exec -it $(docker ps -lq) /bin/bash
cd ros_ws
source install/setup.bash
ros2 topic list
```

If the controller is not running, you may publish goals for the simulator using the below:
```
A command may be published with the below:
```
ros2 topic pub -1 /target_joint_states sensor_msgs/msg/JointState "{position: [1.5, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```