#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from IMU import IMU

FRAME_ID = "imu_link"
# imu.orientation_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
# imu.angular_velocity_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
# imu.linear_acceleration_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};

class IMUMessage:
    def __init__(self):
        self.odom_pub = rospy.Publisher(FRAME_ID, Odometry, queue_size=50)
        # self.odom_broadcaster = tf.TransformBroadcaster()

        self.imu = IMU()

        self.last_distance = self.we.get_distance()
        self.last_time = rospy.Time.now()
        self.seq = 0

    def tick(self):
        self.imu.tick()
        reading = self.imu.get_reading()
        imu_msg = Imu()

        imu_msg.linear_acceleration.x = reading['ax']
        imu_msg.linear_acceleration.y = reading['ay']
        imu_msg.linear_acceleration.z = reading['az']

        imu_msg.angular_velocity.x = reading['gx']
        imu_msg.angular_velocity.y = reading['gy']
        imu_msg.angular_velocity.z = reading['gz']

        imu_msg.orientation = quaternion_from_euler(reading['mx'], reading['my'], reading['mz'])

        imu_msg.header.stamp = rospy.Time.now()
        self.imu_msg.header.frame_id = FRAME_ID
        self.imu_msg.header.seq = self.seq

        self.pub_imu.publish(self.imu_msg)
        self.seq += 1


def begin():
    rospy.init_node('picar_imu', anonymous=True)
    e = IMU()

    while not rospy.is_shutdown():
        e.tick()


if __name__ == '__main__':
    begin()
