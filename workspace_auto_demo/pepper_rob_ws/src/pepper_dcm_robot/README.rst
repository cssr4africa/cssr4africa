pepper_dcm_robot
=============

DCM stack integrating tools to control Pepper robot

Installation
------------
- install dependencies

.. code-block:: bash

        sudo apt-get install ros-indigo-pepper-robot ros-indigo-pepper-meshes ros-indigo-pepper-control ros-indigo-naoqi-dcm-driver

- then, install `pepper_dcm_bringup <https://github.com/ros-naoqi/pepper_dcm_robot>`_ or compile it from source

.. code-block:: bash

        sudo apt-get install ros-indigo-pepper-dcm-bringup

- optionally, install `pepper_moveit_config <https://github.com/ros-naoqi/pepper_moveit_config>`_

.. code-block:: bash

        sudo apt-get install ros-indigo-pepper-moveit-config

How to use it
-------------

**Trajectory control**

To command your robot remotely with ROS control:
    
- start the DCM bringup proving your robot's IP (be aware that the package will stop Autonomous Life on your robot)

.. code-block:: bash

        roslaunch pepper_dcm_bringup pepper_bringup.launch robot_ip:=<ROBOT_IP>

- start Naoqi Driver (to get the odom frame) proving your robot's IP

.. code-block:: bash

        roslaunch naoqi_driver naoqi_driver.launch nao_ip:=<ROBOT_IP>

*Control via MoveIt*

- start Moveit! to control the robot via GUI

.. code-block:: bash

        roslaunch pepper_moveit_config moveit_planner.launch

- check "Allow approximate IK solutions"
- control a planning_group via an interactive marker 

*Control via Actionlib*

- you can send a trajectory to the desired controller (actionlib)

.. code-block:: bash

        rosrun actionlib axclient.py <name of the goal topic of the action server>

example:

.. code-block:: bash

        rosrun actionlib axclient.py /pepper_dcm/LeftArm_controller/follow_joint_trajectory/goal

To choose the controllers you want to load, modify pepper_control/launch/pepper_control_trajectory.launch.
The list of implemented controllers, you can find in pepper_control/config/pepper_trajectory_control.yaml. 
You can start and stop the ros-controllers using the rqt plugin ControllerManager.

**Position control**

To command joints positions via ROS:

- start the DCM bringup proving your robot's IP (be aware that the package will stop Autonomous Life on your robot):

.. code-block:: bash

        roslaunch pepper_dcm_bringup pepper_dcm_bringup_position.launch robot_ip:=<ROBOT_IP>

- send a position to the desired controller, for example

.. code-block:: bash

        rostopic pub /pepper_dcm/HeadYaw_position_controller/command std_msgs/Float64 "data: 1"
