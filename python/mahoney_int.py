import rospy
import time
from turbovec import *
from turbotrig import turboasin, turboatan2
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

class Controller():

    def __init__(self):

        self.kp= 1000 # This should be turned up on initialization, then turned down
        self.ki = 100  # This should be turned up on initialization, then turned down

        self.b = [0, 0, 0]
        self.prev_time = int(time.time()*1000000)

        self.quat = [1000, 0, 0, 0]

        self.w1 = [0, 0, 0]
        self.w2 = [0, 0, 0]

        self.imu_sub_ = rospy.Subscriber('imu/data', Imu, self.imuCallback, queue_size=5)
        self.att_pub_ = rospy.Publisher('attitude', Vector3, queue_size=5)
        self.error_pub_ = rospy.Publisher('error', Vector3, queue_size=5)
        self.bias_pub_ = rospy.Publisher('bias', Vector3, queue_size=5)
        self.q_pub_ = rospy.Publisher('quaternion', Quaternion, queue_size=5)
        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()


    def imuCallback(self, msg):
        now = int(time.time()*1000000)
        dt = now - self.prev_time
        self.prev_time = now

        ax = int(msg.linear_acceleration.x*1000)
        ay = int(msg.linear_acceleration.y*1000)
        az = int(msg.linear_acceleration.z*1000)

        gx = int(msg.angular_velocity.x * 1000)
        gy = int(msg.angular_velocity.y * 1000)
        gz = int(msg.angular_velocity.z * 1000)

        a = [ax, ay, az]
        g = [0, 0, 1000]

        w_meas = [0, 0, 0]
        q_tilde = [1000, 0, 0, 0]

        # sqrnorm = ax*ax +ay*ay + az*az
        # if sqrnorm < 127190000 and sqrnorm < 69483000:
        #     vector_normalize(a)
        #     q_acc = quat_between_vectors(a, g)
        #     q_acc_inv = quat_inverse(q_acc)
        #     q_tilde = quat_multiply(q_acc_inv, self.quat)
        #     w_meas = scalar_multiply(-2*q_tilde[0], [q_tilde[1], q_tilde[2], q_tilde[3]])
        #     # bhat = bhat - ki*dt*w_meas
        #     self.b = vector_sub(self.b, scalar_multiply(self.ki*dt/100000, w_meas))

        # calculate quadratic estimate of omega (see eq. 14, 15 and 16 of reference)
        w = [gx, gy, gz]
        # we might want to convert this to floats, there's quite a bit of loss here
        wbar = vector_add(vector_add(scalar_multiply(-83,self.w2),
                                     scalar_multiply(667,self.w1)),
                          scalar_multiply(417,w))
        self.w2 = self.w1
        self.w1 = wbar

        wcomp = vector_add(vector_sub(wbar, self.b), scalar_multiply(self.kp, w_meas))

        sqrnorm_w = sqred_norm(wcomp)










if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    try:
        controller = Controller()
    except:
        rospy.ROSInterruptException
    pass