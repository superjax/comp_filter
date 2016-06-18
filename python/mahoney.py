#!/usr/bin/env python
# license removed for brevity
import rospy
import time
import tf
from math import atan2, acos, asin, cos, sin
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
import numpy as np

class Controller():

    def __init__(self):

        self.R = np.eye(3)
        self.kp= 10

        self.bx = 0
        self.by = 0
        self.bz = 0

        self.prev_time = time.time()

        self.quat = np.array([[1, 0, 0, 0]]).T

        self.Omega1 = np.zeros([4,4])
        self.Omega2 = np.zeros([4,4])

        self.imu_sub_ = rospy.Subscriber('imu/data', Imu, self.imuCallback, queue_size=5)
        self.att_pub_ = rospy.Publisher('attitude', Vector3, queue_size=5)
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

        p = msg.angular_velocity.x
        q = msg.angular_velocity.y
        r = msg.angular_velocity.z

        w = np.array([[p, q, r]]).T
        norm_w = np.linalg.norm(w)

        # calculate quadratic estimate of Omega (see eq. 14, 15 and 16 of reference)
        Omega = np.array([[0, -p, -q, -r],
                              [p, 0, r, -q],
                              [q, -r, 0, p],
                              [r, q, -p, 0]])
        Omegabar = 1/12.0*(-self.Omega2 + 8*self.Omega1 + 5*Omega)

        # Shift measurements for quaternion estimation
        self.Omega2 = self.Omega1
        self.Omega1 = Omega

        if norm_w > 0:
            # Matrix Exponential Approximation (From Attitude Representation and Kinematic
            # Propagation for Low-Cost UAVs by Robert T. Casey et al.
            self.quat = (cos(norm_w*dt/2.0)*np.eye(4)
                         + 1.0/norm_w*sin(norm_w*dt/2.0)*Omegabar).dot(self.quat)

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




if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    try:
        controller = Controller()
    except:
        rospy.ROSInterruptException
    pass