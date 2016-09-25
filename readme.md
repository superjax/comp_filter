# Mahoney

My implementations of the attitude complementary filter described in "Nonlinear Complementary Filters on the Special Orthoganol Group" By Robert Mahoney, published in 2007.  I have augmented this approach with additions from "Attitude Representation and Kinematic Propagation for Low-Cost UAVs" By Robert Casey, published in 2013.

This is a ROS package, written in python, and contains two nodes.

## generate.py
This is a "truth generator", and is what I used to generate sensor data for testing my node.  It publishes a sensor_msgs/Imu message of simulated IMU data, and a geometry_msgs/Vector3 message of the true euler angles.  It uses rk4 under the hood to propagate attitude changes, and a guassian approximation of noise on each of the 6 axes of the IMU.  It simulates the "rocking" of an IMU on each of the three axes at specified rates and amplitudes.  The noise characteristics, as well as the rate and amplitude of the motion on each axis can be modified in lines 29-36.

## mahoney.py
This is the implementation of the algorithm described above.  For the details of the
algorithm, just read my code and ask questions if I'm not clear.  It subscribes to
IMU and publishes a few messages, including /estimate, which is a Vector3 of the
estimated euler angles, /bias is the estimated biases of the gyroscope measurements
/quaternion is the estimated quaternion /error is the quaternion error between the estimated accelerometer vector and the measured accelerometer vector.  This was useful in debugging my accelerometer adjustment step.
