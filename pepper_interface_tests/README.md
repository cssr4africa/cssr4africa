<div align="center">
<h1> Pepper Interface Tests </h1>
</div>

<div align="center">
  <img src="../CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

The **Pepper Interface Tests** package is a ROS package designed to test the sensors and actuators of the Pepper robot on both physical and simulated platforms. The package is divided into two parts: sensor tests and actuator tests. The sensor tests evaluate the performance of sonar, laser, microphone, and camera sensors. The actuator tests assess the functionality of head, arms, hands, legs, and wheels actuators.

# 📄 Documentation
The main documentation for this deliverable is found in [D4.1 Sensor Test](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D4.1.pdf) and [D5.1 Actuator Test](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.1.pdf) that provide more details.

# 🛠️ Installation 

Install the required software components to instantiate and set up the development environment for controlling the Pepper robot. Use the [CSSR4Africa Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf).

## Installation on Ubuntu (x86-based Systems)

1. Prerequisites  
Make sure you are running Ubuntu 20.04. The ROS environment should be properly set up with the CSSR4Africa workspace.

2. Build the Package
```sh
# Source the workspace
cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash

# Build the package
catkin_make
```

# 🔧 Configuration Parameters
The following configuration files are used to specify test parameters:

| File | Description | Platform |
|------|-------------|----------|
| `actuatorTestConfiguration.ini` | Configuration for actuator tests | Set first line to "platform robot" for physical robot or "platform simulator" for simulator |
| `sensorTestConfiguration.ini` | Configuration for sensor tests | Set first line to "platform robot" for physical robot or "platform simulator" for simulator |

Test input files:
- `actuatorTestInput.dat` - Key-value pairs to test different actuators
- `sensorTestInput.dat` - Key-value pairs to test different sensors

# 🚀 Running the Tests

**Run the Pepper Interface Tests from the `pepper_interface_tests` package:**

Source the workspace in first terminal:
```bash
cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash
```

Follow below steps, run in different terminals.

## Physical Robot Tests

1️. Launch actuator tests on physical robot:
```bash
roslaunch pepper_interface_tests actuatorTestLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface>
```

2️. Launch sensor tests on physical robot:
```bash
roslaunch pepper_interface_tests sensorTestLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface_name>
```

3️. Run actuator test executable:
```bash
rosrun pepper_interface_tests actuatorTest
```

4️. Run sensor test executable:
```bash
rosrun pepper_interface_tests sensorTest
```

## Simulator Robot Tests

1️. Launch interface tests for simulator robot:
```bash
roslaunch pepper_interface_tests interfaceTestLaunchSimulator.launch
```

# 💡 Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:yohanneh@andrew.cmu.edu">yohanneh@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>

# 📜License
Copyright (C) 2023 CSSR4Africa Consortium  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme

2025-03-15
