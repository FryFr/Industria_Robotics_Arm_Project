# Industria_Robotics_Arm_Project
This repository contains all the components of the final project for the Robotics course, which involves the design, simulation, and construction of a 6-DoF (Degrees-of-Freedom) robotic arm equipped with a gripper. The goal is to develop a robotic arm capable of catching an object at a known location using the Robot Operating System 2 (ROS2).

# 6-DoF Robotic Arm with Gripper

![urdf](https://github.com/FryFr/JavaPOOExercise/assets/79547422/4be8ba65-809e-4ff0-ac63-4192f2127995)

## Final Project for Robotics Course

**Institution**: Universidad EAN  
**Professor**: Nikolay Prieto, Ph.D.  
**Due Date**: June 10th, 2024  

## 3D Design

The project began with a crucial phase of 3D design, using Solidworks as the cornerstone for success. Solidworks, a powerful and versatile CAD (Computer-Aided Design) software, allowed the students to translate their ideas into precise and detailed digital models.

This initial stage was vital as it laid the foundation for the subsequent physical development of the robotic arm. Through Solidworks, the students were able to create virtual representations of each component of the arm, from joints and links to fastening and control elements. This allowed project participants to clearly visualize the shape, dimensions, and relationships between the various parts of the robot, facilitating sound technical decision-making.

The precision and detail of the 3D models generated in Solidworks were not only useful for design conceptualization but also served as a fundamental input for the manufacturing stage. The digital models could be exported in formats compatible with 3D printers, allowing for the precise and efficient fabrication of each component of the robotic arm. Thus, the students' ideas were materialized into tangible physical pieces.

![WhatsApp Image 2024-06-12 at 8 35 18 AM](https://github.com/FryFr/JavaPOOExercise/assets/79547422/39657554-21d6-486e-97b8-94641b8c337a)

## Component Acquisition

After the meticulous 3D design phase, it was time to materialize the robotic arm by acquiring the necessary components for its construction. Based on the technical specifications defined during the design phase, the following elements were purchased:

- **Servomotors**: Two types of servomotors were selected for the project: MG996R and SG90. The MG996R, with higher torque and precision, were used for the main joints of the arm, while the more economical and lighter SG90 were used for the secondary joints.
- **Arduino Uno Board**: This microcontroller board, known for its ease of use and versatility, became the brain of the robotic arm. Its processing and communication capabilities made it the ideal choice for controlling the arm's movements and sensor interactions.
- **PCA9685 Board**: To extend the servo control capacity of the Arduino Uno, the PCA9685 board was incorporated. This expansion board allowed connecting and controlling up to 16 additional servomotors, which was essential for the complex articulated structure of the robotic arm.
- **Jumpers**: These flexible connection cables were used to establish electrical connections between the Arduino Uno board, the PCA9685 board, the servomotors, and the sensors. Their practicality and ease of use sped up the assembly and configuration process of the system.
- **Power Supply**: To provide stable power to all the electronic components of the robotic arm, a DC power supply with adequate capacity and voltage was acquired. This ensured the proper functioning of the servomotors, the Arduino Uno board, and the PCA9685 board.
- **PLA Filament**: For the 3D printing of the parts designed in Solidworks, PLA (polylactic acid) filament was used. This high-quality, biodegradable material was the ideal choice for manufacturing the lightweight and durable structures of the robotic arm.
- **Fasteners**: To ensure precise and robust fastening of the 3D printed parts and electronic components, a selection of screws, nuts, and washers of different sizes and materials was acquired. The careful selection of fasteners ensured the stability and durability of the robotic arm.



## Simulation Environment

The Denavit-Hartenberg (DH) algorithm was applied. This algorithm was chosen because it is one of the most efficient for finding the forward kinematics in robotic manipulators. A reference system 0 was established and assigned to axis 1. Following the algorithm, the other reference systems from 1 to 6 were positioned until reaching the end-effector, always avoiding pure rotations and translations around the y-axis.

It is true that there are other methodologies for finding forward kinematics, such as trigonometric analysis and the formulation of position equations in x, y, z coordinates. These methodologies are suitable for representing robots with two or three degrees of freedom but present difficulties when analyzing a higher number of degrees of freedom, as in the case of the present robot.

Once all the reference systems of the 6-degree-of-freedom robot were obtained and the dimensions defined in terms of variables, it was time to proceed with the DH table. To complete this table, it is necessary to identify: the pure rotation in z, pure translation in z, pure translation in x, and pure rotation in x (in that order) of each reference system established in relation to the next system. 

![WhatsApp Image 2024-06-11 at 7 05 25 PM](https://github.com/FryFr/JavaPOOExercise/assets/79547422/60f90e4a-f01a-4976-9a9b-4cf5eb635b6d)

After developing the physical robotic arm and its control system through automatic programming, the next crucial step in the project is creating a virtual simulation environment. This environment will allow testing, validating, and optimizing the robotic arm's performance in a controlled setting before its real-world implementation.

For creating the simulation environment, the following powerful tools will be used:

- **ROS2 (Robot Operating System 2)**: An open-source software framework widely used in robotics for communication between different nodes and distributed task execution.
- **MoveIt**: A set of software tools for motion planning and robotic manipulation, integrated into the ROS2 ecosystem. MoveIt provides advanced algorithms to generate safe and efficient trajectories for the robotic arm, considering physical constraints and environmental restrictions.

### Steps to Create the Simulation Environment:

1. **Installation of ROS2 and MoveIt**: ROS2 and MoveIt will be installed on the Ubuntu system following the official installation instructions.
2. **Modeling the Robotic Arm**: A precise 3D model of the robotic arm will be created using 3D modeling tools like Solidworks. The model should include all joints, dimensions, and physical properties of the real arm.
3. **Integration of the Model into ROS2**: The 3D model of the robotic arm will be integrated into the ROS2 simulation environment using tools like RViz. This will allow visualizing the robotic arm in the virtual environment and controlling its movements through ROS2 commands.
4. **MoveIt Configuration**: MoveIt will be configured to work with the robotic arm model and the simulation environment. This involves defining the joints, physical properties, and movement limits of the arm, as well as the motion planning parameters.
5. **Testing and Validation**: Extensive testing will be conducted to verify the proper functioning of the simulation environment, the accuracy of the robotic arm model, and the effectiveness of MoveIt's motion planning algorithms.

### Benefits of the Simulation Environment:

- **Movement Validation**: The simulation environment allows testing and validating different movements of the robotic arm without risking the actual hardware.
- **Programming Optimization**: The robotic arm's programming can be optimized in the simulation environment before implementing it in the physical system.
- **Error Detection**: The simulation environment facilitates error detection in the programming or the robotic arm model before they cause issues in the real system.
- **Algorithm Training**: The simulation environment can be used to train and optimize control or motion planning algorithms for the robotic arm.
- **Testing in Different Environments**: Tests can be conducted in various virtual environments to evaluate the robotic arm's behavior in different situations and conditions.

![Simulation](images/simulation.png)

---

For more details on each stage of the project and to access the necessary files, please review the following sections of the repository.
