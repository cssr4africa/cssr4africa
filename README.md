### The CSSR4Africa Project

<img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:90%; height:auto;">

Based on an ethnographic research to acquire cultural knowledge about acceptable modes of communication, the CSSR4Africa project aims to equip robots with the ability to interact sensitively and politely with people in Africa using spatial, non-verbal, and verbal modes of communication. 

The project has the following objectives. 

1. Identify the verbal and non-verbal social and cultural norms of human interaction that are prevalent in countries in Africa.
2. Encapsulate them in the behavioural patterns of social robots so that they can engage with African people in a manner that is consistent with their expectations of acceptable social interaction.
3. Demonstrate these culturally-sensitive social robot behaviours in two use cases: one for giving a tour of a university laboratory, and one for assisting and giving directions to visitors at the reception of a university.

The CSSR4Africa software repository contains the code and resources for the CSSR4Africa project. The repository is organized as follows:

- `cssr4africa` directory contains the source code for the CSSR4Africa project.
- `cssr4africa.github.io` contains the the project's website.
- `gazebo_model_velocity_plugin` publisheds
- `naoqi_dcm_driver` hardware interface allowing control Nao, Romeo, and pepper robots using the Naoqi API.
- `naoqi_driver` module that establishes a ROS brige to the Naoqi API. It publishes sensor data as well as the robot position. 
- `pepper_dcm_robot` DCM (Device Communication Manager) stack integrating tools to control the Pepper robot.
- `software installation scripts` contains scripts for installing the software required for the CSSR4Africa project.

A detailed report on how to install the software and run the CSSR4Africa project can be found in the [CSSR4Africa Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf).

The system architecture details the ROS packages that will eventually be part of the CSSR4Africa sofware repository. The architecture is available in the [CSSR4Africa System Architecture](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.1.pdf). As of now, the available nodes are `animateBehaviour`, `gestureExecution`, and `overtAttention` in the [cssr_system](/cssr_system/) directory. These software have been passed through the software integration process as reported in the [System Integration and Quality Assurance](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.5.pdf). This integration process is based on the standards set out in the [Software Engineering Standards Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.2.pdf) and the [System Integration and Quality Assurance Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.4.pdf). 

Unit tests for the `cssr_system` nodes are provides in the [unit_test](/unit_tests/) directory. Instructions for running the unit tests in provided in the [README.md](/unit_tests/README.md) and in the `README.md` file in each unit test directory. A user manual will be made available on the [cssr4africa](https://cssr4africa.github.io/) website in due course.



## Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:david@vernon.eu">david@vernon.eu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>

## License  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme

Date:   2025-02-07
