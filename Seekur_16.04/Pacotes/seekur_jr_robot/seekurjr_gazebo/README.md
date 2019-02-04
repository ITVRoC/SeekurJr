# Seekur Jr. URDF/CAD robot model simulation

To satisfy requirements for Gazebo>=1.9 in ROS Groovy:

~~~{.bash}
sudo apt-get install ros-groovy-pcl-conversions
sudo apt-get install ros-groovy-control-msgs
~~~

Every session should be started as:

~~~{.bash}
source ~/rosws/setup.bash
source devel/setup.bash
localmaster

~~~

or

~~~{.bash}
source ~/rosws/setup.bash
localmaster

~~~

Simulator start:

~~~{.bash}
roscore

rosrun gazebo_ros gazebo

roslaunch seekurjr_gazebo spawn_model.xml

roslaunch seekurjr_control steering.launch

rosrun seekurjr_control test_trajectory

roslaunch seekurjr_gazebo rqt_imu.xml

rosrun rqt_graph rqt_graph

rosrun rviz rviz -d src/seekurjr_gazebo/urdf.rviz 

roslaunch seekurjr_gazebo xacrodisplay.xml model:=src/seekurjr_gazebo/urdf/seekurjr.xacro.xml

~~~

Spawn robot model:

~~~{.bash}
rosrun gazebo_ros spawn_model -file seekurjr.urdf -urdf -z 1 -model seekurjr

rosrun gazebo_ros spawn_model -file src/seekurjr_gazebo/urdf/seekurjr.urdf -urdf -z 1 -model seekurjr

roslaunch seekurjr_gazebo display.xml model:=src/seekurjr_gazebo/urdf/seekurjr.urdf
~~~

Execute ROS stereoprocessing core and rqt gui control application:

~~~{.bash}
ROS_NAMESPACE=mobileranger/camera rosrun stereo_image_proc stereo_image_proc
rosrun image_view stereo_view stereo:=/mobileranger/camera image:=image_rect_color
rosrun rqt_gui rqt_gui 
~~~

Sample session can be seen by following video link below.

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/Nogzb74IlV0/0.jpg)](https://www.youtube.com/watch?v=Nogzb74IlV0)

# Experiment

~~~{.bash}
roscore

rosrun gazebo_ros gazebo

roslaunch seekurjr_gazebo spawn_model.xml

roslaunch seekurjr_control steering.launch

roslaunch seekurjr_gazebo imu.xml

roslaunch seekurjr_gazebo rqt_imu.xml


rosrun rviz rviz -d src/seekurjr_gazebo/urdf.rviz 

roslaunch seekurjr_gazebo robot_pose_ekf.xml

ROS_NAMESPACE=mobileranger/camera rosrun stereo_image_proc stereo_image_proc
rosrun image_view stereo_view stereo:=/mobileranger/camera image:=image_rect_color

~~~

With launch files:

~~~{.bash}
roscore

	# simulator
rosrun gazebo_ros gazebo

	# load model
roslaunch seekurjr_gazebo spawn_model.xml

	# manual model control
roslaunch seekurjr_control steering.launch

	# imu
roslaunch seekurjr_gazebo imu.xml

	# stereo core 
roslaunch seekurjr_gazebo stereocore.xml

	# laser nodes
roslaunch seekurjr_gazebo laser.xml
roslaunch seekurjr_gazebo pcl_conv.xml

	# map server
roslaunch seekurjr_gazebo octomap_server.xml

	# visualisation
rosrun rviz rviz -d src/seekurjr_gazebo/urdf.rviz 
roslaunch seekurjr_gazebo rqt_imu.xml
rosrun image_view stereo_view stereo:=/mobileranger/camera image:=image_rect_color

~~~


# Inertia

Table 1. Main SeekurJr mechanical specification

Specification | Value
--- | ---:
DIMENSIONS (LxWxH) | 1050mm x 840mm x 500mm
WEIGHT      | 77kg (1 battery)
GROUND CLEARANCE | 105mm
TIRES | 400mm
WHEELBASE | 425mm


Hector team uses simplified inertia tensors:

~~~
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

  <xacro:macro name="inertial_cuboid" params="mass x_length y_length z_length">
    <inertial>
      <mass value="${mass}" />
      <origin xyz="0 0 0" />
      <inertia ixx="${(1/12) * mass* (y_length*y_length+ z_length*z_length)}" ixy="0.0" ixz="0.0"
               iyy="${(1/12) * mass* (x_length*x_length+ z_length*z_length)}" iyz="0.0"
               izz="${(1/12) * mass* (x_length*x_length+ y_length*y_length)}" />
    </inertial>
  </xacro:macro>

  <xacro:macro name="inertial_sphere" params="mass diameter">
    <inertial>
      <mass value="${mass}" />
      <origin xyz="0 0 0" />
      <inertia ixx="${(2/5) * mass* ( (diameter*0.5)) * (diameter*0.5))}" ixy="0.0" ixz="0.0"
               iyy="${(2/5) * mass* ( (diameter*0.5)) * (diameter*0.5))}" iyz="0.0"
               izz="${(2/5) * mass* ( (diameter*0.5)) * (diameter*0.5))}" />
    </inertial>
  </xacro:macro>

</robot>
~~~


According to 
[Wikipedia](http://en.wikipedia.org/wiki/List_of_moment_of_inertia_tensors),
list of moment of inertia tensors:

One can calculate approximate inertai tensor for SeekurJr in Ocatve:

~~~{.octave}
m=77; h=0.5; w=0.84; d=1.05; 
ixx = 1/12*m*(h^2+d^2)
iyy = 1/12*m*(w^2+d^2) 
izz = 1/12*m*(h^2+d^2) 
~~~

Results are used in `<inertia />` URDF-tag parameters:

~~~
<inertia  ixx="8.6785" ixy="0"  ixz="0"  iyy="11.602"  iyz="0"  izz="8.6785" />
~~~

Another possible way to calculate inertia tensor for complicated objects is to use
dedicated software. For example one can load solid mechanical model in **Meshlab**
and calculate inertia in two steps:

- View > Show Layer Dialog
- Filter > Quality Measure and computations > Compute Geometric measures

~~~
Opened mesh /home/kp/rosws/catkin/src/seekurjr_gazebo/meshes/seekurjr_body_new_low.stl in 15 msec
All files opened in 1902 msec
Mesh Bounding Box Size 0.627341 0.394017 1.197709
Mesh Bounding Box Diag 1.408301 
Mesh Volume is 0.155966
Mesh Surface is 4.018419
Thin shell barycenter 0.008623 0.248038 -0.001686
Center of Mass is 0.006557 0.130922 -0.038411
Inertia Tensor is :
| 0.011083 -0.000132 -0.000303 |
| -0.000132 0.011182 -0.000597 |
| -0.000303 -0.000597 0.005307 |
Principal axes are :
| 0.885792 -0.460959 0.053744 |
| 0.454450 0.885044 0.100860 |
| -0.094058 -0.064917 0.993448 |
axis momenta are :
| 0.011047 0.011294 0.005230 |
Applied filter Compute Geometric Measures in 12 msec
~~~

Resulting `<inertia />` tag wolud be:

~~~
<inertia  ixx="0.011083" ixy="-0.000132"  ixz="-0.000303"  iyy="0.011182"  iyz="-0.000597"  izz="0.005307" />
~~~

Useful links:

~~~
http://answers.ros.org/question/30539/choosing-the-right-coefficients-for-gazebo-simulation/
http://wiki.ros.org/urdf/Tutorials/Adding%20Physical%20and%20Collision%20Properties%20to%20a%20URDF%20Model
http://answers.gazebosim.org/question/4372/the-inertia-matrix-explained/
~~~

Example for SeekurJr robot can be found in `seekurjr_gazebo/urdf/base.urdf.xml`.

# Wheels and Steering

As for mu, mu2, slip1, slip2 they are explained here, notice that the values are between [0..1]. For more information check out this ODE page and just search for mu, mu2, slip1, slip2 it will be more deeply explained.

~~~
http://answers.gazebosim.org/question/1505/how-do-i-set-up-mu-and-slip-for-a-skid-steer-robot/
http://www.ode.org/ode-latest-userguide.html
~~~

For skid-steering model simulation `<plugin name="skid_steer_drive_controller" filename="libgazebo_ros_skid_steer_drive.so">` is used.

Example for SeekurJr robot can be found in `seekurjr_gazebo/urdf/base.urdf.xml`
and `seekurjr_gazebo/urdf/base.gazebo.xml`.



## Recommended Mesh Resolution

For collision checking using the ROS motion planning packages, as few faces per link as possible are recommended for the collision meshes that you put into the URDF (ideally less than 1000). If possible, approximating the meshes with other primitives is encouraged. 
    
[ROS Wiki](http://wiki.ros.org/urdf/XML/link)

# IMU

Similary to other sensors, for IMU simulation we need to define corresponding 
**link**, **joint**, and **gazebo** parameters.

[ROS Answers](http://answers.ros.org/question/12430/modelling-sensorsimu-in-gazebo/)


Example for SeekurJr robot can be found in `seekurjr_gazebo/urdf/imu.urdf.xml`
and `seekurjr_gazebo/urdf/imu.gazebo.xml`.

# Bumpers

Similary to other sensors, bumper requires to define corresponding 
**link**, **joint**, and **gazebo** parameters.

~~~
<!-- BAMPER -->
<!--
<gazebo>
  <plugin name="${name}_gazebo_ros_bumper_controller" filename="libgazebo_ros_bumper.so">
    <alwaysOn>true</alwaysOn>
    <updateRate>${update_rate}</updateRate>
    <bumperTopicName>${name}_bumper</bumperTopicName>
    <frameName>world</frameName>
  </plugin>
</gazebo>
-->
~~~

~~~
http://answers.gazebosim.org/question/3877/contact-sensor-no-data-output/
http://answers.gazebosim.org/question/5355/adding-ros-integrated-contact-sensors/
http://answers.ros.org/question/29158/how-do-i-use-force-sensor-bumper-sensor-in-gazebo/
~~~

Example for SeekurJr robot can be found in `seekurjr_gazebo/urdf/bumpers.urdf.xml`
and `seekurjr_gazebo/urdf/bumpers.gazebo.xml`.

# Laser

Similary to other sensors, laser rangefinder requires to define corresponding 
**link**, **joint**, and **gazebo** parameters.

General examples can be found on Gazebo 
[wiki](http://gazebosim.org/wiki/Tutorials/1.9/ROS_Motor_and_Sensor_Plugins).

Example for SeekurJr robot can be found in `seekurjr_gazebo/urdf/lms1xx.urdf.xml`
and `seekurjr_gazebo/urdf/lms1xx.gazebo.xml`.


# Camera

Similary to other sensors, laser rangefinder requires to define corresponding 
**link**, **joint**, and **gazebo** parameters.

Example for SeekurJr robot can be found in `seekurjr_gazebo/urdf/stereocamera.urdf.xml`
and `seekurjr_gazebo/urdf/stereocamera.gazebo.xml`.

[Link](http://answers.ros.org/question/61712/how-to-use-libgazebo_ros_cameraso-in-gazebo-urdf/)

