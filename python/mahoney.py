#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from math import atan2, acos, asin
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

        self.qw = 1.0
        self.qx = 0
        self.qy = 0
        self.qz = 0

        self.roll = 0
        self.pitch = 0
        self.yaw = 0


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

        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        #
        # ex = 0
        # ey = 0
        # ez = 0
        #
        # spin_rate = pow(gx**2 + gy**2 + gz**2, 0.5)
        #
        # recipNorm = 0 # ax**2 + ay**2 + az**2
        # if recipNorm > 0.0:
        #     recipNorm = 1/pow(recipNorm,0.5)
        #     ax *= recipNorm
        #     ay *= recipNorm
        #     az *= recipNorm
        #
        #     # calculate error between estimated gravity vector (R) and measured (a*)
        #     # This can also be said as the cross product of the gravity vector and the
        #     # bottom row of R
        #     ex = ay * self.R[2][2] - az * self.R[2][1]
        #     ey = az * self.R[2][0] - ax * self.R[2][2]
        #     ez = ax * self.R[2][1] - ay * self.R[2][0]
        #
        # # Bias estimation (none for now)
        # self.bx = 0
        # self.by = 0
        # self.bz = 0
        #
        # # # apply proportional feedback
        # # gx += self.kp*ex + self.bx
        # # gy += self.kp*ey + self.by
        # # gz += self.kp*ez + self.bz
        #
        # # integrate quaternion
        # # gx *= (0.5 * dt)
        # # gx *= (0.5 * dt)
        # # gx *= (0.5 * dt)
        #
        # qa = self.qw
        # qb = self.qx
        # qc = self.qy
        # qd = self.qz
        #
        # self.qw += -qb*gx - qc*gy - qd*gz
        # self.qx += qa*gx + qc*gz - qd*gy
        # self.qy += qa*gy - qb*gz + qd*gx
        # self.qz += qa*gz + qb*gy -qc*gx
        #
        # # normalize quaternion
        # recipNorm = 1/pow(self.qx**2 + self.qy**2 + self.qz**2 + self.qw**2,0.5)
        # self.qw *= recipNorm
        # self.qx *= recipNorm
        # self.qy *= recipNorm
        # self.qz *= recipNorm
        #
        # # Compute Rotation Matrix From Quaternion
        # q1q1 = self.qx**2
        # q2q2 = self.qy**2
        # q3q3 = self.qz**2
        #
        # q0q1 = self.qw*self.qx
        # q0q2 = self.qw*self.qy
        # q0q3 = self.qw*self.qz
        # q1q2 = self.qx*self.qy
        # q1q3 = self.qx*self.qz
        # q2q3 = self.qy*self.qz
        #
        # self.R[0][0] = 1 - 2*q2q2 - 2*q3q3
        # self.R[0][1] = 2 * (q1q2 - q0q3)
        # self.R[0][2] = 2 * (q1q3 + q0q2)
        #
        # self.R[1][0] = 2 * (q1q2 + q0q3)
        # self.R[1][1] = 1 - 2* q1q1- 2*q3q3
        # self.R[1][2] = 2 * (q2q3 - q0q1)
        #
        # self.R[2][0] = 2 * (q1q3 - q0q2)
        # self.R[2][1] = 2 * (q2q3 + q0q1)
        # self.R[2][2] = 1 - 2*q1q1 - 2*q2q2
        #
        # # extract euler angles
        # roll = atan2(self.R[2][1], self.R[2][2]) * 180/3.14159
        # pitch = atan2(-self.R[2][0], pow(self.R[2][1]**2 + self.R[2][2]**2,0.5)) * 180/3.14159
        # yaw = atan2(self.R[1][0], self.R[0][0])* 180/3.14159

        self.roll += gx*dt
        self.pitch += gy*dt
        self.yaw += gz*dt

        # Pack up and send command
        attitude = Vector3()
        attitude.x = self.roll
        attitude.y = self.pitch
        attitude.z = self.yaw
        self.att_pub_.publish(attitude)

        # quat= Quaternion()
        # quat.x = self.qx
        # quat.y = self.qy
        # quat.z = self.qz
        # quat.w = self.qw
        # self.q_pub_.publish(quat)


if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    try:
        controller = Controller()
    except:
        rospy.ROSInterruptException
    pass