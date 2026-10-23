# kinematics_workshop

A general overview is presented here, details and steps are in sub-headers.

For working in simulation, first pull the docker image. This contains ros2, pybullet, and moveit2 with all the libraries needed for this lab and everything compiled for you (moveit2 takes like 30min to compile), you're welcome. Then we will run the docker image letting it access X11 ports so as to bring up graphical user interfaces (GUIs) and letting it access the folder on our computer where we have our own code. Inside of the docker container, we will build our own code (should be fast, python) and then run it. We will do introspection to understand the system and look at graphs showing trajectories to evaluate it. We will make our changes and test in simulation before testing on the real robot.

To use the real robot we will use a special docker image (built for ARM, doesn't include moveit2 as it's too heavy) that has probably been pre-loaded for you onto the robot. Then we will connect to the robot (pre-configured to broadcast wifi) and run everything on the robot's computer. Students in the past have had networking trouble related to MAC/Windows machines and therefore we avoid this issue by running everything on the robot. Students have also had trouble related to displays/monitors and we avoid this issue by running everything over ssh with no GUI, providing a simulator on student laptops to do introspection and testing.

## Pre-requisites
* Ubuntu 22.04 (via a VM if needed)
* docker engine installed, and post install steps run (so that you can run without `sudo`)
* some free harddrive space and familiarity with ros2

## simulation

### get docker file

If on a system with AMD architecture:
```
sudo docker pull --platform linux/amd64 mzandtheraspberrypi/ros_sim:2025-10-24
```

If on a system with ARM architecture (also update the image in the commands in the README)
```
sudo docker pull --platform linux/arm64 mzandtheraspberrypi/ros_sim:2024-10-24
```

### running examples on your laptop

First we run the simulation with the robot:

MAC USERS:
Run `xhost +` in the terminal before you do the docker run command.

```
cd $HOME
git clone https://github.com/MZandtheRaspberryPi/kinematics_workshop.git
sudo docker run --rm --platform linux/amd64 -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY -e QT_X11_NO_MITSHM=1 --network host -v /home/$USER/kinematics_workshop:/home/developer/ros_ws/src/kinematics_workshop mzandtheraspberrypi/ros_sim:2025-10-24
sudo apt-get remove -y python3-matplotlib
cd ros_ws
colcon build --symlink-install --packages-select ros_sim ros_arm_controller ros_cobot_controller
source install/setup.bash
ros2 run ros_sim pybullet_ex
```

You may open more terminals to allow introspection with:
```
sudo docker exec -it $(sudo docker ps -lq) /bin/bash
cd ros_ws
source install/setup.bash
ros2 topic list
```

A command may be published with the below which should move the robot.
```
ros2 topic pub -1 /target_joint_states sensor_msgs/msg/JointState "{position: [1.5, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

To run the controller and follow the trajectory. Note graphs will be produced visualizing the trajectory in the `kinematics_workshop` base folder upon succesful completion of one trajectory.

```
docker exec -it $(docker ps -lq) /bin/bash
cd ros_ws
source install/setup.bash
ros2 run ros_arm_controller robo_controller --ros-args -p use_sim_time:=true
```

To run Moveit2, note this is NOT included in ARM images (as it would require a lot of setup) so you won't be able to run it from ARM images. Also note, that everything else should be shutdown before running this.
```
colcon build --symlink-install --packages-select mycobot_moveit2
source install/setup.bash
ros2 launch mycobot_moveit2 demo.launch.py
```

## Running on the arm

### connect to the arm

Connect to the robot's WIFI. The WLAN should be named after the robot ID which is on a label on the base of the robot. Double check you are connected to the right robot--if you move a robot from somebody else's group that may surprise them and cause some damage to the robot or people.  SSID “mycobot_23”, password: "autonomy"
Robot ip address: 10.42.0.1, user: er, password: Elephant (or elephant) — to connect use ssh: `ssh er@10.42.0.1`.


### running examples on the robot

When you run the below block, the robot should move to point upwards. At this point you won't be able to move the motors by hand anymore as the servos will be powered.
```
docker run --rm -it --network host --device /dev/ttyAMA0 -v /home/er/kinematics_workshop:/home/developer/ros_ws/src/kinematics_workshop -v /dev/shm:/dev/shm mzandtheraspberrypi/ros_sim:2024-10-24
sudo apt-get remove -y python3-matplotlib
cd ros_ws
colcon build --symlink-install
source install/setup.bash
ros2 run ros_cobot_controller cobot_controller
```

From here, try to publish joint state messages and see if the robot moves:
```
sudo docker exec -it $(sudo docker ps -lq) /bin/bash
cd ros_ws
source install/setup.bash
ros2 topic pub -1 /target_joint_states sensor_msgs/msg/JointState "{position: [1.5, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

Now you can work on the trajectory in simulation, disconnecting from wifi, and once you have a trajectory you like put it on the robot by scp'ing any files you changed over to the robot or editing it on the robot via command line.

To scp from your computer: `scp $HOME/kinematics_workshop/ros_arm_controller/ros_arm_controller/trajectory.py er@10.42.0.1:/home/er/kinematics_workshop/ros_arm_controller/ros_arm_controller/trajectory.py`

To run your trajectory:
```
docker exec -it $(docker ps -lq) /bin/bash
cd ros_ws
source install/setup.bash
ros2 run ros_arm_controller robo_controller --ros-args -p use_sim_time:=false
```

## citations

https://mathweb.ucsd.edu/~sbuss/ResearchWeb/ikmethods/SdlsPaper.pdf for overview including jacobian iterations, and damped least squares

## troubleshooting:

An issue relating to X Error of failed request, simply close the docker image, re-run it, and try again.
```
developer@student-AORUS-5-KB:~/ros_ws$ ros2 run ros_sim pybullet_ex 
pybullet build time: Sep 30 2025 11:26:09
[INFO] [1759315578.864487194] [ros_sim_node]: loading: /home/developer/ros_ws/install/ros_sim/share/ros_sim/resource/robots/mycobot_280_pi/mycobot_280_pi_mod.urdf
argv[0]=
startThreads creating 1 threads.
starting thread 0
started thread 0 
argc=3
argv[0] = --unused
argv[1] = 
argv[2] = --start_demo_name=Physics Server
ExampleBrowserThreadFunc started
X11 functions dynamically loaded using dlopen/dlsym OK!
X11 functions dynamically loaded using dlopen/dlsym OK!
MESA: error: Failed to query drm device.
libGL error: glx: failed to create dri3 screen
libGL error: failed to load driver: iris
libGL error: failed to open /dev/dri/card1: No such file or directory
libGL error: failed to load driver: iris
Creating context
Created GL 3.3 context
Direct GLX rendering context obtained
Making context current
GL_VENDOR=Mesa
GL_RENDERER=llvmpipe (LLVM 15.0.7, 256 bits)
GL_VERSION=4.5 (Core Profile) Mesa 23.2.1-1ubuntu3.1~22.04.3
GL_SHADING_LANGUAGE_VERSION=4.50
pthread_getconcurrency()=0
Version = 4.5 (Core Profile) Mesa 23.2.1-1ubuntu3.1~22.04.3
Vendor = Mesa
Renderer = llvmpipe (LLVM 15.0.7, 256 bits)
b3Printf: Selected demo: Physics Server
startThreads creating 1 threads.
starting thread 0
started thread 0 
MotionThreadFunc thread started
ven = Mesa
X Error of failed request:  BadValue (integer parameter out of range for operation)
  Major opcode of failed request:  130 (MIT-SHM)
  Minor opcode of failed request:  3 (X_ShmPutImage)
  Value in failed request:  0x400
  Serial number of failed request:  61
  Current serial number in output stream:  62
[ros2run]: Process exited with failure 1
```

An issue relating to failed udp writes appearing in logs frequently. Turn off wifi on your computer, restart it, and try again. ROS2 is a network protocol and it may be trying to talk to other computers on the network.

```
tev: ddsi_udp_conn_write to udp/10.23.12.21:7410 failed with retcode -1
```