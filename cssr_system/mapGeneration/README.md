<div align="center">
  <h1>Environment Map Generation</h1>
</div>

<div align="center">
  <img src="../../CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

The `mapGeneration` node in the CSSR4Africa project provides functionality for generating environment maps and configuration space maps for robot navigation. This node processes obstacle data from input files, generates workspace maps representing the physical environment, and creates configuration space maps that account for the robot's physical dimensions.

# Documentation

This node is part of the CSSR4Africa project and is documented in the deliverable report [D5.5.3 Environment Map Generation](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.5.3.pdf). The report provides a detailed explanation of the map generation process, algorithms used, and how to configure and run the node.

# Running the Map Generation Node

## Setup and Execution

### Prerequisites
Before running the mapGeneration node, ensure you have the following:
- ROS environment setup and roscore running
- CSSR4Africa software components installed

For setting up the development environment, refer to the [CSSR4Africa Software Installation Manual](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf).

### Steps

1. **Clone and build the project (if not already cloned)**:
   - Move to the source directory of the workspace
      ```bash 
      cd $HOME/workspace/pepper_rob_ws/src
      ```
   - Clone the `CSSR4Africa` software from the GitHub repository
      ```bash 
      git clone https://github.com/cssr4africa/cssr4africa.git
      ```
   - Build the source files
      ```bash
      cd ..
      ```
      ```bash
      catkin_make
      ```
      ```bash
      source devel/setup.bash 
      ```

    <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
    <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
    <span style="color: #cccccc;"> Make sure you have the <code>cssr4africa/cssr_system/mapGeneration</code> package built successfully before proceeding. </span>
</div>

2. **Configure input data**:
   - The node uses data files located in `mapGeneration/data/`:
     - `mapGenerationInput.dat`: Defines map dimension and filenames
     - `obstacles.dat`: Contains obstacle definitions

   - Sample `mapGenerationInput.dat` contains:
    <div style="background-color:rgb(0, 0, 0); padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;"> <span style="color: #33ff33; font-weight: bold;">NOTE: </span> <span style="color: #cccccc;"> Map dimensions (mapWidth and mapHeight) should be specified in meters. The resolution parameter determines the conversion to pixels (e.g., resolution = 0.01 means 1 centimeter per pixel, so a 6.80m x 9.93m map becomes 680x993 pixels). </span> </div>

     ```
      mapWidth 6.80
      mapHeight 9.93
      obstacleFile obstacles.dat
      environmentMapFile environmentMap.png
      configurationSpaceMapFile configurationSpaceMap.png
     ```

   - Sample `obstacles.dat` contains:
     ```
      OBSTACLE   2.5775 3.12  2.315 1.44
      OBSTACLE   1.3  0.45  2.60 0.90
      OBSTACLE   2.20  0.30  4.40 0.60
      OBSTACLE   3.31  1.925  0.22 0.25
      OBSTACLE   3.31  5.525  0.22 0.25
      OBSTACLE   3.31  7.325  0.22 0.25
      OBSTACLE   3.31  9.125  0.22 0.25
      OBSTACLE   0.40  4.965  0.80 9.93
      OBSTACLE   3.40  9.465  6.80 0.93
      OBSTACLE   6.20  4.965  1.20 9.93


     ```

1. **Configure node parameters**:
   - The node's behavior can be customized through the `mapGenerationConfiguration.ini` file:
    
    | Parameter | Description | Values | Default Value |
    |-----------|-------------|--------|---------------|
    | mode | Operation mode | `CAD` or `SLAM` | `CAD` |
    | verboseMode | Enable detailed logging | `true` or `false` | `false` |
    | inputFile | Input file name | Any valid file name | `mapGenerationInput.dat` |
    | resolution | Map resolution in meters/pixel | Any positive float | `0.01` |
    | robotRadius | Robot radius in meters | Any positive float | `0.3` |

   <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
    <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
    <span style="color: #cccccc;"> If you need to update the configuration values, please refer to the <a href="https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D5.5.3.pdf" style="color: #66b3ff;">D5.5.3 Environment Map Generation</a>. Otherwise, the default values are already set in the configuration file.</span>
   </div>

   **Effect of Configuration Options:**
   - **mode**: Selecting `CAD` uses geometric data from an input file for map generation, while `SLAM` would use sensor data (currently CAD is the only supported mode).
   - **verboseMode**: Setting to `true` enables detailed logging messages for debugging purposes.
   - **inputFile**: Changes the source file for map generation parameters and output filenames.
   - **resolution**: Adjusting this value changes the precision of the generated maps. Lower values create more detailed maps but require more processing time.
   - **robotRadius**: Altering this value affects the configuration space map by changing how much space around obstacles is marked as unreachable.

2. **Run the mapGeneration node**:
   
   It's important to note that the mapGeneration node operates independently of physical robot hardware. You can generate environment maps and configuration space maps without connecting to a Pepper robot. Since the CAD-based map generation relies solely on geometric data provided in input files, no sensor data or robot connectivity is required.
   
   **Make sure you start a roscore in a new terminal**

   You can run the node in the following way in another terminal:

   - Using rosrun:
     ```bash
     cd $HOME/workspace/pepper_rob_ws
     source devel/setup.bash
     
     ```
     ```bash
     rosrun cssr_system mapGeneration
     ```

3. **Generated Output**
   
   The node generates the following output files in the data directory:

   - **environmentMap.png**:  
     The workspace map showing the physical environment with obstacles.

   - **configurationSpaceMap.png**:  
     The configuration space map accounting for the robot's physical dimensions, showing areas where the robot's center point can safely navigate.

## Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:david@vernon.eu">david@vernon.eu</a>, <a href="mailto:africa-robotics@andrew.cmu.edu">africa-robotics@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>, <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:ebirhan@andrew.cmu.edu">bgirmash@andrew.cmu.edu</a><br>


## License  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme

Date: 2025-06-05
