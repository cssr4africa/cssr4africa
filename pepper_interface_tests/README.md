# Pepper Interface Tests
<a href="https://cssr4africa.github.io/"> <img src="https://github.com/cssr4africa/cssr4africa.github.io/blob/main/docs/images/CSSRforAfrica_logo_red.png?raw=true" alt="CSSR4Africa" height=20> </a>

Pepper interface tests is a ROS package to test the sensors and actuators of the Pepper robot on a physical and simulator platform. After setting up the development environment using the instruction on the document 

## Documentation
The main documentation for this deliverable is found in [D3.3 Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf) that
provides a detailed installation guide.

## Running Tests
To run the test on the physical platform, change the first line of actuatorTestConfiguration.ini file in the config folder
to “platform robot”. On the other hand, to run the test on the simulator platform, change the first line of simulatorTestConfiguration.ini file to “platform simulator”.

## Physical Robot
This command launches pepper interface test for the physical robot.
```sh
roslaunch pepper_interface_tests interface.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface_name>
```

This command launches naoqi driver for the physical robot.
```sh
roslaunch naoqi_driver naoqi_driver.launch nao_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface_name>
```

## Simulator Robot
This command launches pepper interface test for the simulator robot.
```sh
roslaunch pepper_gazebo_plugin pepper_gazebo_plugin_in_office_CPU.launch
```


## Sensor and actuator Test
actuatorTestInput.dat and sensorTestInput.dat are provided that container key-value pair to test the different actuator and sensor found in Pepper robot.

This command test the sensor for the physical robot
```sh
rosrun pepper_interface_tests sensorTest
```

This command test the actuator for the physical robot
```sh
rosrun pepper_interface_tests actuatorTest
```
