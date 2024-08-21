# Pepper Interface Tests

<img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:100%; height:auto;">

The Pepper interface tests package is a ROS package designed to test the sensors and actuators of the Pepper robot on both physical and simulated platforms. After setting up the development environment using the software installation document as outlined in the [D3.3 Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf), the package can be installed and run on the Pepper robot.

The package is divided into two parts: sensor tests and actuator tests. The sensor tests are designed to evaluate the performance of the following sensors: sonar, laser, microphone, and camera. The actuator tests assess the functionality of the following actuators: head, arms, hands, legs, and wheels.

## Documentation
Accompnaying this code, there are deliverable reports that provides a detailed explanation of the code and how to run the tests. The deliverable reports are can be found in [D4.1 Sensor test](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.1.pdf) and [5.1 Actuator test](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.1.pdf)

## Running Tests
To run the test on the physical platform, change the first line of `actuatorTestConfiguration.ini` file in the config folder
to “platform robot”. On the other hand, to run the test on the simulator platform, change the first line of simulatorTestConfiguration.ini file to “platform simulator”.

## Physical Robot
This command launches actuator test
```sh
roslaunch pepper_interface_tests actuatorTestLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface>
```

This command launches sensor test 
```sh
roslaunch roslaunch pepper_interface_tests sensorTestLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface_name>
```

## Simulator Robot
This command launches pepper interface test for the simulator robot.
```sh
roslaunch pepper_interface_tests interfaceTestLaunchSimulator.launch
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
