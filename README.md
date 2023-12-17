####UR5 Robot Simulation (Deliverable 1)####

This code simulates the UR5 robot, a six-axis industrial robot arm. The code uses the Denavit-Hartenberg (DH) parameters to represent the robot's kinematics.
DH Parameters

The DH parameters of the UR5 robot are as follows:
Link	a	d	θ	φ
1	0	0.08946	π/2	0
2	-0.425	0	0	0
3	-0.3922	0	0	0
4	0	0.1091	π/2	-π/2
5	0	0.09465	-π/2	0
6	0	0	0	0

where:

    a is the length of the link
    d is the offset from the previous link's z-axis to the current link's x-axis
    θ is the twist angle between the previous link's z-axis and the current link's z-axis
    φ is the rotation angle around the current link's x-axis

Simulation

The code simulates the UR5 robot by creating a SerialLink object and calling its plot() method. The plot() method takes a 6-tuple of joint angles as input. The first three angles represent the joints in the first three links, the next three angles represent the joints in the next three links, and the last angle represents the last joint.

The code also displays the DH table of parameters for the UR5 robot using the display() method.

###Deliverable 2####
Three-Link Planar Manipulator Control

This code demonstrates the control of a three-link planar manipulator using the Jacobian and manipulability. The manipulator has three revolute joints, and the goal is to move the end-effector to a desired position and orientation.

The code first defines the manipulator's parameters, including the link lengths and the initial configuration. Then, it calculates the manipulator's Jacobian matrix and its manipulability.

Next, the code moves the end-effector to a desired position and orientation using two different control strategies:

    With maximization of the manipulability: The Jacobian is augmented with the manipulability metric, and the inverse Jacobian is used to calculate the joint velocities.

    Without maximization of the manipulability: The Jacobian is simply inverted to calculate the joint velocities.

The results of both control strategies are then compared.
Results

The code shows that the strategy that maximizes the manipulability results in smoother and more accurate motion. This is because the manipulability metric measures the ability of the manipulator to move the end-effector in different directions without changing its orientation. By maximizing the manipulability, the code is able to find a control strategy that allows the manipulator to move efficiently and accurately.
Conclusion

This code demonstrates that the Jacobian and manipulability can be used to control a robotic manipulator effectively. By maximizing the manipulability, the code is able to find control strategies that result in smoother and more accurate motion.
Additional notes

    The code was written in MATLAB.
    The manipulability metric is calculated using the following formula:

w = sqrt(det(J * J'))

where J is the Jacobian matrix.

    The code uses the fkine and invkin functions from the Robotics Toolbox to calculate the forward and inverse kinematics of the manipulator.

    The code uses the pinv function to find the pseudoinverse of the Jacobian matrix.
