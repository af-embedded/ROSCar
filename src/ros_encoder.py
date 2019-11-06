#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry
from wheel_encoder import WheelEncoder

FRAME_ID = "odom"
PUB_TOPIC = "/wheel_encoder_odom"

class Encoder:
    def __init__(self):
        self.odom_pub = rospy.Publisher(PUB_TOPIC, Odometry, queue_size=50)
        # self.odom_broadcaster = tf.TransformBroadcaster()

        self.we = WheelEncoder()

        self.last_distance = self.we.get_distance()
        self.last_time = rospy.Time.now()

    def tick(self):
        current_time = rospy.Time.now()
        current_distance = self.we.get_distance()

        distance = current_distance - self.last_distance
        time = (current_time - self.last_time).to_sec()
        speed = distance / time

        odom = Odometry()
        odom.child_frame_id = "base_link"
        odom.header.stamp = current_time
        odom.header.frame_id = FRAME_ID
        odom.pose.pose.position.x = distance
        odom.twist.twist.linear.x = speed
        self.odom_pub.publish(odom)

        self.last_distance = current_distance
        self.last_time = current_time


def begin():
    rospy.init_node('picar_encoder', anonymous=True)
    e = Encoder()

    while not rospy.is_shutdown():
        e.tick()


if __name__ == '__main__':
    begin()
