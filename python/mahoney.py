#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from math import atan2, acos, asin, sin, cos, tan
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
import numpy as np

def skewSymmetric(v):
    v = v[0,:]
    A =  np.array([ [ 0, -v[2], v[1] ],
                    [ v[2], 0, -v[0] ],
                    [-v[1], v[0], 0  ] ])
    return A

def antiSymmetric(H):
    A = 1/2.0*(H - H.T)
    return A

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

        recipNorm = 1/pow(msg.linear_acceleration.x**2 + msg.linear_acceleration.y **2 + msg.linear_acceleration.z **2, 0.5)

        ax = msg.linear_acceleration.x * recipNorm
        ay = msg.linear_acceleration.y * recipNorm
        az = msg.linear_acceleration.z * recipNorm

        gx = msg.angular_velocity.x - 0.08512
        gy = msg.angular_velocity.y + 0.02128
        gz = msg.angular_velocity.z + 0.029792

        # Calculate Rotation Matrix from body to inertial given acc measurement
        v_acc_b = np.array([[ax, ay, az]]).T
        v_acc_i = np.array([[0, 0, 1]]).T
        v = skewSymmetric(v_acc_i.T).dot(v_acc_b)
        s = v.T.dot(v)
        c = v_acc_i.T.dot(v_acc_i)

        R_meas = np.eye(3) + skewSymmetric(v.T) + skewSymmetric(v.T)**2*(1-c)/(s**2)
        print(R_meas)


        Omega = np.array([[gx, gy, gz]])

        omegax = skewSymmetric(Omega)
        self.R += (omegax.dot(self.R))*dt


        # Re-Normalize Rotation
        # Per Direction Cosine Matrix IMU: Theory
        # By William Premerlani and Paul Bizard
        # https://www.google.com/webhp?sourceid=chrome-instant&ion=1&espv=2&ie=UTF-8#q=DCMDraft.pdf
        x = np.array([self.R[0]])
        y = np.array([self.R[1]])
        error = x.dot(y.T)

        x = x - error/2*y
        y = y - error/2*x
        z = skewSymmetric(x).dot(y.T).T
        x = 1/2.0*(3-x.dot(x.T))*x
        y = 1/2.0*(3-y.dot(y.T))*y
        z = 1/2.0*(3-z.dot(z.T))*z
        self.R = np.array([x[0,:],
                           y[0,:],
                           z[0,:]])


        # extract euler angles
        # Per "Decomposing a Rotation Matrix - Nghia Ho
        # http://nghiaho.com/?page_id=846
        roll = atan2(self.R[2][1], self.R[2][2]) * 180/3.14159
        pitch = atan2(-self.R[2][0], pow(self.R[2][1]**2 + self.R[2][2]**2,0.5)) * 180/3.14159
        yaw = atan2(self.R[1][0], self.R[0][0])* 180/3.14159


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