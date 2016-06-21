#!/usr/bin/env python
# license removed for brevity
import rospy
import time
import tf
from math import atan2, acos, asin, cos, sin, sqrt
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
import numpy as np

def quat_multiply(q1, q2):
    s1 = q1[0,0]
    s2 = q2[0,0]

    v1 = q1[1:4]
    v2 = q2[1:4]

    w = s1*s2 - v1.T.dot(v2)
    xyz = s1*v2 + s2*v1 - np.cross(v1,v2, axis=0)

    out = np.append(w, xyz, axis=0)
    return out



class Controller():

    def __init__(self):

        self.R = np.eye(3)
        self.kp= 0.5
        self.ki = 0.000

        self.b = np.array([[0, 0, 0]]).T
        self.prev_time = time.time()

        self.quat = np.array([[1, 0, 0, 0]]).T

        self.w1 = np.zeros([3,1])
        self.w2 = np.zeros([3,1])

        self.imu_sub_ = rospy.Subscriber('imu/data', Imu, self.imuCallback, queue_size=5)
        self.att_pub_ = rospy.Publisher('attitude', Vector3, queue_size=5)
        self.error_pub_ = rospy.Publisher('error', Vector3, queue_size=5)
        self.bias_pub_ = rospy.Publisher('bias', Vector3, queue_size=5)
        self.q_pub_ = rospy.Publisher('quaternion', Quaternion, queue_size=5)
        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()


    def imuCallback(self, msg):
        now = time.time()
        dt = now - self.prev_time
        self.prev_time = now

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        a = np.array([[ax, -ay, -az]]).T
        I = np.array([[0, 0, -1]]).T

        # Pull in Accelerometer, and get attitude measurement
        norm =  sqrt(ax**2 + ay**2 + az **2)
        w_meas = np.array([[0, 0, 0]]).T
        q_tilde = np.array([[1, 0, 0, 0]]).T
        if norm < 1.15*9.80665 and norm > 0.85*9.80665:
            # normalize acceleration vector
            recipNorm = 1.0/np.linalg.norm(a)
            a *= recipNorm

            # Build w_meas from Mahoney Eq. 47a
            half = a + I
            half *= 1.0/np.linalg.norm(half)
            q_meas = a.T.dot(half)
            q_meas = np.append(q_meas, np.array([np.cross(a[:,0],half[:,0])]).T, axis=0)
            q_meas_inverse = q_meas.copy()
            q_meas_inverse[0,0] *= -1.0
            q_tilde = quat_multiply(q_meas_inverse,self.quat)
            s_tilde = q_tilde[0,0]
            v_tilde = q_tilde[1:4]
            w_meas = -2*s_tilde*v_tilde


            # Perform Bias Estimator (Eq. 48c)
            # self.b = -self.ki*w_meas

        # calculate quadratic estimate of omega (see eq. 14, 15 and 16 of reference)
        w = np.array([[msg.angular_velocity.x,
                       msg.angular_velocity.y,
                       msg.angular_velocity.z]]).T
        wbar = 1/12.0*(-1*self.w2 + 8*self.w1 + 5*w)
        self.w2 = self.w1
        self.w1 = wbar

        # This is the vector that is inside the p function in equation 47a of the Mahoney
        # Paper
        wfinal = wbar - self.b + self.kp*w_meas

        p = wfinal[0,0]
        q = wfinal[1,0]
        r = wfinal[2,0]

        norm_w = np.linalg.norm(wfinal)

        Omega = np.array([[0, -p, -q, -r],
                          [p, 0, r, -q],
                          [q, -r, 0, p],
                          [r, q, -p, 0]])

        if norm_w > 0:
            # Matrix Exponential Approximation (From Attitude Representation and Kinematic
            # Propagation for Low-Cost UAVs by Robert T. Casey et al.
            self.quat = (cos(norm_w*dt/2.0)*np.eye(4)
                         + 1.0/norm_w*sin(norm_w*dt/2.0)*Omega).dot(self.quat)

        # Normalize Quaternion
        recipNorm = 1.0/np.linalg.norm(self.quat)
        self.quat *= recipNorm

        q0 = self.quat[0]
        q1 = self.quat[1]
        q2 = self.quat[2]
        q3 = self.quat[3]
        roll  = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2))
        pitch = asin(2*(q0*q2 - q3*q1))
        yaw = atan2(2*(q0*q3 + q1*q2),1 - 2*(q2**2 + q3**2))

        # Pack up and send command
        attitude = Vector3()
        attitude.x = roll
        attitude.y = pitch
        attitude.z = yaw
        self.att_pub_.publish(attitude)

        # Calculate Error from Accel Measurement
        q0 = q_tilde[0]
        q1 = q_tilde[1]
        q2 = q_tilde[2]
        q3 = q_tilde[3]
        roll  = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2))
        pitch = asin(2*(q0*q2 - q3*q1))
        yaw = atan2(2*(q0*q3 + q1*q2),1 - 2*(q2**2 + q3**2))
        attitude.x = roll
        attitude.y = pitch
        attitude.z = yaw
        self.error_pub_.publish(attitude)

        # Publish Biases
        bias = Vector3()
        bias.x = self.b[0,0]
        bias.y = self.b[1,0]
        bias.z = self.b[2,0]
        self.bias_pub_.publish(bias)



if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    try:
        controller = Controller()
    except:
        rospy.ROSInterruptException
    pass