# Integrating-Occupancy-grip-map-and-LIDAR-on-a-Differential-Drive-Robot-in-ROS2
The focus of this project was on integrating an occupancy grid map and a LiDAR sensor to enhance the simulation's fidelity and utility of a differnetial drive robot in ROS2.


### Description of the Overall Approach:
**System Overview**
In expanding the differential drive robot simulator from Project 4a, the focus was on integrating an occupancy grid map and a LiDAR sensor to enhance the simulation's fidelity and utility. The
occupancy grid provides a discrete representation of the environment, marking free, occupied, and unknown spaces, which allows the robot to make informed navigation decisions. The
addition of a simulated LiDAR sensor equips the robot with the ability to perceive its surroundings, detect obstacles, and adjust its trajectory in real-time. Together, these
enhancements enable the simulation of more complex scenarios and behaviors, such as obstacle avoidance and path planning.

**Node 1: Velocity Translator Node**
This remains the same from the previous project (4a)

**Node 2: Simulator Node**
The DifferentialDriveSimulator node simulates a differential drive robot in a two-dimensional environment, as defined by the provided robot_name and world_file_name parameters. The robot is represented by a URDF (Unified Robot Description Format) file, which outlines its physical characteristics, such as body radius, wheel distance, and laser scanner specifications. Additionally, the robot's behavior is influenced by parameters such as error variance for wheel velocities, update rates for error calculation, and laser scanner properties including range, angle, and failure probabilities. Upon initialization, the simulator reads in a world file that specifies the layout of the environment, including obstacles and the initial pose of the robot. This file is used to generate an OccupancyGrid, which is a grid-based representation of the environment where each cell can be free, occupied, or unknown. The grid is then published to the /map topic, allowing visualization in RViz or other ROS tools.

The node subscribes to /vl and /vr topics to receive velocity commands for the left and right wheels, respectively. These velocities are subject to simulated random errors based on the predefined variances. The simulator uses these 
velocities to compute the robot's new pose at regular intervals, considering the robot's current orientation and the specified wheel velocities.Project 4b Report Group 38 : Robotics and Spatial Intelligence Collision detection is an integral part of the simulation. Before updating the robot's pose, a collision check is performed by creating a set of boundary points around the robot's circumference and verifying if any of these points would lie within an obstacle in the next timestep. If a collision is detected, the robot's movement is halted. The laser scanner simulation is achieved through a ray-casting method. For each laser beam, an angle is calculated relative to the robot's orientation, and a ray is cast into the environment. If the ray encounters an obstacle within the laser's range, the distance is recorded; otherwise, the maximum range is returned. To add realism, each measurement can fail with a certain probability, resulting in a NaN value, and Gaussian noise is added to the distance measurements. Lastly, the laser scan data is published to the /scan topic at a frequency determined by the laser's specified rate, enabling the visualization of the environment from the robot's perspective in RViz. 

### Launch File and System Integration

To ensure synchronized operation, a launch file is used to initialize the system. It sets up the necessary arguments for input and output bag files, starts the playback of recorded sensor data, initiates all the required nodes, and captures the system's output to the specified output bag file.

The choices made in designing the program: When designing the enhanced differential drive robot simulator, several deliberate choices were made to ensure a robust and realistic simulation environment:

1. Occupancy Grid Map Integration: The decision to incorporate an occupancy grid map was driven by the need to simulate a more realistic navigation scenario. The grid represents the spatial layout of the environment and is fundamental for path planning and collision detection. It allows the robot to make decisions based on the structure of the simulated world, enhancing the realism of the simulation.
2. LiDAR Sensor Simulation: Simulating a LiDAR sensor was a pivotal choice to provide the robot with environmental awareness. Ray-casting algorithms were employed to mimic the sensor's behavior, allowing the robot to detect obstacles and measure distances to them. This simulation includes the addition of Gaussian noise and a failure rate to the sensor readings to closely emulate real-world uncertainties and sensor imperfections.
3. Error Simulation: Incorporating error simulation in wheel velocities and sensor measurements was a deliberate choice to prepare for the imperfect nature of real-world robotics. By considering these factors, the simulator provides a more challenging and instructive platform for developing and testing control algorithms.
4. Real-Time Updates and Feedback: The system was designed to operate in real-time, continuously responding to control inputs, updating the robot's state, and publishing sensor data. This dynamic nature is crucial for testing the responsiveness of navigation algorithms under simulated real-world conditions.
5. Modularity and Parameterization: The program was structured with modularity in mind, allowing individual components, such as the robot's physical parameters or the environmental map, to be easily modified or replaced. This design choice facilitates experimentation with different robot configurations or maps without extensive code changes.

### Did the results meet our expectations?
In evaluating the performance of the differential drive simulator node, the results met our expectations. The integration of the occupancy grid map and the LiDAR sensor simulation functioned as intended, providing a comprehensive representation of the robot's environment. The system successfully interpreted the YAML world file to construct a simulated map, accurately reflecting the placement of obstacles and free spaces within the grid. The simulated LiDAR sensor, powered by a ray-casting algorithm, effectively identified obstacles by publishing detailed scan data to the /scan topic. This allowed for a realistic depiction of how a physical robot's sensor would operate, taking into account various environmental factors and sensor noise.

Collision detection was another aspect that performed as expected. The simulator preemptively checked the robot's proposed movements against the map data, preventing it from moving into occupied grid cells. Overall, the simulator node achieved its goal of creating a dynamic and interactive environment for the differential drive robot. It allowed for the testing of control strategies, sensor integration, and autonomous behavior in a setting that closely mimics the challenges faced in actual robotic applications. The successful implementation of these features underscores the effectiveness of the simulation in providing a reliable platform for robotics research and development.

### Screenshots showing the final configuration in rviz:

![image](https://github.com/khullarsanket/Integrating-Occupancy-grip-map-and-LIDAR-on-a-Differential-Drive-Robot-in-ROS2/assets/119709438/ced0a6ea-6b2b-4057-8ab7-b8a7e1be32d1)

![image](https://github.com/khullarsanket/Integrating-Occupancy-grip-map-and-LIDAR-on-a-Differential-Drive-Robot-in-ROS2/assets/119709438/3beffe14-21da-4c2f-8419-9c76f307454a)

![image](https://github.com/khullarsanket/Integrating-Occupancy-grip-map-and-LIDAR-on-a-Differential-Drive-Robot-in-ROS2/assets/119709438/b0b7605c-50db-431f-a893-4d845ae3cd14)

![image](https://github.com/khullarsanket/Integrating-Occupancy-grip-map-and-LIDAR-on-a-Differential-Drive-Robot-in-ROS2/assets/119709438/a0278c35-1423-4547-815a-19fa8bcf4744)

![image](https://github.com/khullarsanket/Integrating-Occupancy-grip-map-and-LIDAR-on-a-Differential-Drive-Robot-in-ROS2/assets/119709438/2a954a7e-25d4-4fd3-900a-477526de9f1a)

![image](https://github.com/khullarsanket/Integrating-Occupancy-grip-map-and-LIDAR-on-a-Differential-Drive-Robot-in-ROS2/assets/119709438/6e6f773a-ef4f-419e-97d1-f56f2f8cefd9)



