#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import time
from geometry_msgs.msg import Vector3
from math import sin, cos, atan2, sqrt
import numpy as np
from scipy import linalg

def R(phi, theta, psi):
    ct = cos(theta)
    st = sin(theta)
    cs = cos(psi)
    ss = sin(psi)
    cp = cos(phi)
    sp = sin(phi)
    out = np.array([[ct*cs, ct*ss, -st],
                    [sp*st*cs-cp*ss, sp*st*ss+cp*cs, sp*ct],
                    [cp*st*cs+sp*ss, cp*st*ss-sp*cs, sp*st]])
    return out

class Generate:

    def __init__(self):

        self.prev_time = time.time()
        self.prev_publish_time = time.time()
        self.start_time = time.time()
        self.frequency = np.array([0.05, 1, 0])
        self.amplitude = np.array([3.14159/2.0*0.005, 3.14159/4.0, 0])

        self.accel_noise = 0.1
        self.gyro_noise = 0.1

        self.gyro_bias = np.array([-.1, .3, 0.0])
        self.accel_bias = np.array([0, 0, 0])

        self.R = np.eye(3)

        self.imu_pub_ = rospy.Publisher("imu/data", Imu, queue_size=5)
        self.truth_pub_ = rospy.Publisher("truth", Vector3, queue_size=5)

        while not rospy.is_shutdown():
            self.publish()
        debug = 1

    def skew(self, x):
        cross = np.array([[0, -x[2][0], x[1][0]],
                          [x[2][0], 0, -x[0][0]],
                          [-x[1][0], x[0][0], 0]])
        return cross


    def publish(self):
        now = time.time()
        t = now - self.start_time
        dt = now - self.prev_time
        # dt = 0.01
        # t = self.prev_time + dt
        # now = t
        self.prev_time = now
        p = self.amplitude[0] * cos(t * self.frequency[0] * (2 * 3.14159))
        q = self.amplitude[1] * cos(t * self.frequency[1] * (2 * 3.14159))
        r = self.amplitude[2] * cos(t * self.frequency[2] * (2 * 3.14159))
        w = np.array([[p, q, r]]).T

        # Integrate
        self.R = self.R.dot(linalg.expm(self.skew(w * dt)))
        if now - self.prev_publish_time > 0.01:
            self.prev_publish_time = now
            imu = Imu()

            # Extract Roll, Pitch, and Yaw
            phi = atan2(self.R[2,1], self.R[2,2])
            theta = atan2(-self.R[2,0], sqrt(self.R[2,1]**2 + self.R[2,2]**2))
            psi = atan2(self.R[1,0], self.R[0,0])

            # Get Simulated Acceleration and Angular Velocity Vectors
            I = np.array([[0, 0, 9.80665]]).T
            a = self.R.dot(I)
            imu.linear_acceleration.x = a[0,0]
            imu.linear_acceleration.y = a[1,0]
            imu.linear_acceleration.z = a[2,0]
            imu.angular_velocity.x = p
            imu.angular_velocity.y = q
            imu.angular_velocity.z = r

            # Add Noise
            imu.linear_acceleration.x += np.random.normal(0,self.accel_noise,1)[0] + self.accel_bias[0]
            imu.linear_acceleration.y += np.random.normal(0,self.accel_noise,1)[0] + self.accel_bias[1]
            imu.linear_acceleration.z += np.random.normal(0,self.accel_noise,1)[0] + self.accel_bias[2]
            imu.angular_velocity.x += np.random.normal(0,self.gyro_noise,1)[0] + self.gyro_bias[0]
            imu.angular_velocity.y += np.random.normal(0,self.gyro_noise,1)[0] + self.gyro_bias[1]
            imu.angular_velocity.z += np.random.normal(0,self.gyro_noise,1)[0] + self.gyro_bias[2]

            #Timestamp
            imu.header.stamp = rospy.Time.now()

            # Publish IMU
            self.imu_pub_.publish(imu)

            # Publish Truth
            truth = Vector3()
            truth.x = phi
            truth.y = theta
            truth.z = psi
            self.truth_pub_.publish(truth)

            # Maybe Publish Quaternion?

if __name__ == '__main__':
    rospy.init_node('imu_sim', anonymous=True)

    gen = Generate()
    pass
