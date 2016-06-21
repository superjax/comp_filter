import rospy
from sensor_msgs.msg import Imu
import time
from geometry_msgs.msg import Vector3
from math import sin, cos
import numpy as np


class Test:

    def __init__(self):

        self.prev_time = 0
        self.start_time = time.time()
        self.frequency = 1
        self.amplitude = 3.14159 / 2.0

        self.accel_noise = 1
        self.gyro_noise = 1

        self.gyro_bias = np.array([0, 0, 0])
        self.accel_bias = np.array([0, 0, 0])

        self.axis = 0

        self.imu_pub_ = rospy.Publisher("imu/data", Imu, queue_size=5)
        self.truth_pub_ = rospy.Publisher("truth", Vector3, queue_size=5)

        while not rospy.is_shutdown():
            self.publish()

    def publish(self):
        now = time.time()
        if now - self.prev_time > 0.001:
            self.prev_time = now
            t = now - self.start_time

            truth = Vector3()
            truth.x = 0
            truth.y = 0
            truth.z = 0

            imu = Imu()

            angle = self.amplitude*sin(t * self.frequency*(2*3.14159))
            rate = self.amplitude*self.frequency*(2*3.14159)*cos(t * self.frequency*(2*3.14159))

            if self.axis == 0:
                imu.linear_acceleration.x = 0
                imu.linear_acceleration.y = 9.8066*sin(angle)
                imu.linear_acceleration.z = 9.8066*cos(angle)
                imu.angular_velocity.x = rate
                imu.angular_velocity.y = 0
                imu.angular_velocity.z = 0
                truth.x = angle

            elif self.axis == 1:
                imu.linear_acceleration.x = 9.8066*sin(angle)
                imu.linear_acceleration.y = 0
                imu.linear_acceleration.z = 9.8066*cos(angle)
                imu.angular_velocity.x = 0
                imu.angular_velocity.y = rate
                imu.angular_velocity.z = 0
                truth.y = angle

            elif self.axis == 2:
                imu.linear_acceleration.x = 0
                imu.linear_acceleration.y = 0
                imu.linear_acceleration.z = 9.8066
                imu.angular_velocity.x = 0
                imu.angular_velocity.y = 0
                imu.angular_velocity.z = rate
                truth.z = angle

            # Add Noise
            imu.linear_acceleration.x += np.random.normal(0,self.accel_noise,1)[0] + self.accel_bias[0]
            imu.linear_acceleration.y += np.random.normal(0,self.accel_noise,1)[0] + self.accel_bias[1]
            imu.linear_acceleration.z += np.random.normal(0,self.accel_noise,1)[0] + self.accel_bias[2]
            imu.angular_velocity.x += np.random.normal(0,self.gyro_noise,1)[0] + self.gyro_bias[0]
            imu.angular_velocity.y += np.random.normal(0,self.gyro_noise,1)[0] + self.gyro_bias[1]
            imu.angular_velocity.z += np.random.normal(0,self.gyro_noise,1)[0] + self.gyro_bias[2]

            #Timestamp
            imu.header.stamp = rospy.Time.now()

            # Publish
            self.imu_pub_.publish(imu)

            # Truth
            self.truth_pub_.publish(truth)
        else:
            time.sleep(0.005)



if __name__ == '__main__':
    rospy.init_node('imu_sim', anonymous=True)
    try:
        test = Test()
    except:
        rospy.ROSInterruptException
    pass

