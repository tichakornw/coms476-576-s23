## Lab 07 - ROS Navigation with Turtlebot 3



### 1. Environment Setup

Install dependent ROS 1 packages.

```
sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy \
  ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc \
  ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan \
  ros-melodic-rosserial-arduino ros-melodic-rosserial-python \
  ros-melodic-rosserial-server ros-melodic-rosserial-client \
  ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server \
  ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro \
  ros-melodic-compressed-image-transport ros-melodic-rqt* \
  ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers
```

Install Turtlebot3 Package

```
sudo apt-get install ros-melodic-turtlebot3
```

Set the default `TURTLEBOT3_MODEL` name to your model. Enter the below command to a terminal.

```
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
```

Source the bashrc with below command

```
source ~/.bashrc
```



### 2. SLAM

The **SLAM (Simultaneous Localization and Mapping)** is a technique to draw a map by estimating current location in an arbitrary space.

#### Run SLAM Node

1. Simulate the Turtlebot3 with Gazebo

   ```
   roslaunch turtlebot3_gazebo turtlebot3_world.launch
   ```

2. Open a new terminal with `Ctrl` + `Alt` + `T` and launch the SLAM node. The Gmapping is used as a default SLAM method

   ```
   roslaunch turtlebot3_slam turtlebot3_slam.launch
   ```

3. Open a new terminal and run the teleoperation node

   ```
   roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
   ```

   after that you will see the teleoperation instruction

   ```
    Control Your TurtleBot3!
    ---------------------------
    Moving around:
           w
      a    s    d
           x

    w/x : increase/decrease linear velocity
    a/d : increase/decrease angular velocity
    space key, s : force stop

    CTRL-C to quit
   ```

4. Start exploring and drawing the map

#### Tuning Guide

Gmapping has many parameters to change performances for different environments. You can get an information about whole parameters in [ROS WiKi](http://wiki.ros.org/gmapping) or refer to the Chapter 11 of [ROS Robot Programming](https://community.robotsource.org/t/download-the-ros-robot-programming-book-for-free/51). This tuning guide provides tips when configuring gmapping parameters. If you want to optimize SLAM performances for your environments, this section might be helpful.

Below parameters are defined in `turtlebot3_slam/config/gmapping_params.yaml` file.

* **maxUrange**

  This parameter is set the maximum usable range of the lidar sensor.

* **map_update_interval**

  This parameter defines time between updating the map. The smaller the value, the more frequent the map is updated.
  However, setting this too small will be require more processing power for the map calculation. Set this parameter depending on the map environment.

* **minimumScore**

  This parameter sets the minimum score value that determines the success or failure of the sensor’s scan data matching test. This can reduce errors in the expected position of the robot in a large area. If the parameter is set properly, you will see information similar to one shown below.

  ```
  Average Scan Matching Score=278.965
  neff= 100
  Registering Scans:Done
  update frame 6
  update ld=2.95935e-05 ad=0.000302522
  Laser Pose= -0.0320253 -5.36882e-06 -3.14142
  ```

  If set too high, you might see below warnings.

  ```
  Scan Matching Failed, using odometry. Likelihood=0
  lp:-0.0306155 5.75314e-06 -3.14151
  op:-0.0306156 5.90277e-06 -3.14151
  ```

* **linearUpdate**

  When the robot translates longer distance than this value, it will run the scan process.

* **angularUpdate**

  When the robot rotates more than this value, it will run the scan process. It is recommended to set this value less than linearUpdate.



#### Save map

The map is drawn based on the robot’s [odometry](https://en.wikipedia.org/wiki/Odometry), [tf](http://wiki.ros.org/tf) and scan information. These map data is drawn in the RViz window as the TurtleBot3 was traveling. After creating a complete map of desired area, save the map data to the local drive for the later use.

1. Launch the **map_saver** node in the map_server package to create map files. The map file is saved in the directory where the map_saver node is launched at. Unless a specific file name is provided, `map` will be used as a default file name and create `map.pgm` and `map.yaml`.

   ```
   rosrun map_server map_saver -f ~/map
   ```

   The `-f` option specifies a folder location and a file name where files to be saved.
   With the above command, `map.pgm` and `map.yaml` will be saved in the home folder `~/`(/home/${username}).



#### Map

The map uses two-dimensional **Occupancy Grid Map (OGM)**, which is commonly used in ROS. The saved map will look like the figure below, where **white** area is collision free area while **black** area is occupied and inaccessible area, and **gray** area represents the unknown area. This map is used for the [Navigation](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#navigation).

![img](https://emanual.robotis.com/assets/images/platform/turtlebot3/slam/map.png)



### 3. Navigation

#### Run Navigation nodes

```
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```

#### Estimate Initial Pose

**Initial Pose Estimation** must be performed before running the Navigation as this process initializes the AMCL parameters that are critical in Navigation. TurtleBot3 has to be correctly located on the map with the LDS sensor data that neatly overlaps the displayed map.

1. Click the `2D Pose Estimate` button in the RViz menu.

   ![img](https://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/2d_pose_button.png)

2. Click on the map where the actual robot is located and drag the large green arrow toward the direction where the robot is facing.

3. Repeat step 1 and 2 until the LDS sensor data is overlayed on the saved map.

4. Launch keyboard teleoperation node to precisely locate the robot on the map.

   ```
   roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
   ```

5. Move the robot back and forth a bit to collect the surrounding environment information and narrow down the estimated location of the TurtleBot3 on the map which is displayed with tiny green arrows.

   ![img](https://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/tb3_amcl_particle_01.png)

6. Terminate the keyboard teleoperation node by entering `Ctrl` + `C` to the teleop node terminal in order to prevent different **cmd_vel** values are published from multiple nodes during Navigation.

#### Set Navigation Goal

1. Click the `2D Nav Goal` button in the RViz menu.

   ![img](https://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/2d_nav_goal_button.png)

2. Click on the map to set the destination of the robot and drag the green arrow toward the direction where the robot will be facing.

   - This green arrow is a marker that can specify the destination of the robot.
   - The root of the arrow is `x`, `y` coordinate of the destination, and the angle `θ` is determined by the orientation of the arrow.
   - As soon as x, y, θ are set, TurtleBot3 will start moving to the destination immediately.

   ![img](https://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/2d_nav_goal.png)

#### Tuning Guide

Navigation stack has many parameters to change performances for different robots.

You can get more information about Navigation tuning from [Basic Navigation Tuning Guide](http://wiki.ros.org/navigation/Tutorials/Navigation Tuning Guide), [ROS Navigation Tuning Guide by Kaiyu Zheng](http://kaiyuzheng.me/documents/navguide.pdf), and the chapter 11 of [ROS Robot Programming](https://community.robotsource.org/t/download-the-ros-robot-programming-book-for-free/51) book.

##### inflation_radius

- Defined in `turtlebot3_navigation/param/costmap_common_param_${TB3_MODEL}.yaml`
- This parameter makes inflation area from the obstacle. Path would be planned in order that it don’t across this area. It is safe that to set this to be bigger than robot radius. For more information, please refer to the [costmap_2d wiki](http://wiki.ros.org/costmap_2d#Inflation).

![img](https://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/tuning_inflation_radius.png)

##### cost_scaling_factor

- Defined in `turtlebot3_navigation/param/costmap_common_param_${TB3_MODEL}.yaml`

- This factor is multiplied by cost value. Because it is an reciprocal propotion, this parameter is increased, the cost is decreased.

  ![img](https://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/tuning_cost_scaling_factor.png)

##### max_vel_x

- Defined in `turtlebot3_navigation/param/dwa_local_planner_params_${TB3_MODEL}.yaml`
- This factor is set the maximum value of translational velocity.

##### min_vel_x

- Defined in `turtlebot3_navigation/param/dwa_local_planner_params_${TB3_MODEL}.yaml`
- This factor is set the minimum value of translational velocity. If set this negative, the robot can move backwards.

##### max_trans_vel

- Defined in `turtlebot3_navigation/param/dwa_local_planner_params_${TB3_MODEL}.yaml`
- Actual value of the maximum translational velocity. The robot can not be faster than this.

##### min_trans_vel

- Defined in `turtlebot3_navigation/param/dwa_local_planner_params_${TB3_MODEL}.yaml`
- Actual value of the minimum translational velocity. The robot can not be slower than this.

##### max_rot_vel

- Defined in `turtlebot3_navigation/param/dwa_local_planner_params_${TB3_MODEL}.yaml`
- Actual value of the maximum rotational velocity. The robot can not be faster than this.

##### min_rot_vel

- Defined in `turtlebot3_navigation/param/dwa_local_planner_params_${TB3_MODEL}.yaml`
- Actual value of the minimum rotational velocity. The robot can not be slower than this.

##### acc_lim_x

- Defined in `turtlebot3_navigation/param/dwa_local_planner_params_${TB3_MODEL}.yaml`
- Actual value of the translational acceleration limit.

##### acc_lim_theta

- Defined in `turtlebot3_navigation/param/dwa_local_planner_params_${TB3_MODEL}.yaml`
- Actual value of the rotational acceleration limit.

##### xy_goal_tolerance

- Defined in `turtlebot3_navigation/param/dwa_local_planner_params_${TB3_MODEL}.yaml`
- The x,y distance allowed when the robot reaches its goal pose.

##### yaw_goal_tolerance

- Defined in `turtlebot3_navigation/param/dwa_local_planner_params_${TB3_MODEL}.yaml`
- The yaw angle allowed when the robot reaches its goal pose.

##### sim_time

- Defined in `turtlebot3_navigation/param/dwa_local_planner_params_${TB3_MODEL}.yaml`

- This factor is set forward simulation in seconds. Too low value is in sufficient time to pass narrow area and too high value is not allowed rapidly rotates. You can watch defferences of length of the yellow line in below image.

  ![img](https://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/tuning_sim_time.png)