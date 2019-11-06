#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class Robot:
    FORWARD_X_SPEED = 0.5 # linear x speed when moving forwards
    TURN_X_SPEED = 0.15 # linear x speed when turning
    TURN_Z_SPEED = 0.35 # angular z speed when turning
    TURN_YAW_DIFF = 0.45 # amount to turn the robot by
    COLLISION_DISTANCE = 0.7 # distance that is considered a collision
    NUMBER_COLLISIONS = 5 # number of measurements before a collision is declared

    def __init__(self):
        self.msg = Twist()
        self.pose = Pose()
        self.collided_once = False
        self.is_colliding = False
        self.collision_side = None

        self.publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.laser_cb)
        rospy.Subscriber("/odometry/filtered", Odometry, self.pose_cb)

    def move(self):
        self.turn()
        self.forward()

    def forward(self):
        current_yaw = self.pose.orientation.z
        print("starting moving, current yaw:", current_yaw)
        self.msg.linear.x = Robot.FORWARD_X_SPEED
        self.msg.angular.z = 0
        while not self.is_colliding and not rospy.is_shutdown():
            self.publisher.publish(self.msg)

        self.stop()

    def turn(self):
        current_yaw = self.pose.orientation.z
        print("starting turning, current yaw:", current_yaw)

        self.msg.linear.x = -Robot.TURN_X_SPEED

        if self.collision_side == "left":
            self.msg.angular.z = Robot.TURN_Z_SPEED
            new_yaw = round(current_yaw + Robot.TURN_YAW_DIFF, 2)
        else:
            self.msg.angular.z = -Robot.TURN_Z_SPEED
            new_yaw = round(current_yaw - Robot.TURN_YAW_DIFF, 2)

        if new_yaw < -1:
            new_yaw += 2
        elif new_yaw > 1:
            new_yaw -= 2
        print("turning, target yaw:", new_yaw)

        curr_yaw = round(self.pose.orientation.z, 2)
        while curr_yaw != new_yaw and not rospy.is_shutdown():
            temp_yaw = round(self.pose.orientation.z, 2)
            curr_yaw = temp_yaw
            self.publisher.publish(self.msg)

        self.stop()
        print("finished turning, new yaw:", self.pose.orientation.z)

    def stop(self):
        print("stopping")
        self.is_colliding = False
        self.msg.linear.x = 0
        self.msg.angular.z = 0
        self.publisher.publish(self.msg)

    def laser_cb(self, data):
        num_close = len(filter(lambda x: x <= Robot.COLLISION_DISTANCE, data.ranges))
        if num_close >= Robot.NUMBER_COLLISIONS:
            if self.collided_once:
                if data.ranges.index(min(data.ranges)) > 360:
                    self.collision_side = "left"
                else:
                    self.collision_side = "right"

                if not self.is_colliding:
                    print("collision confirmed on the", self.collision_side)
                self.is_colliding = True
            else:
                print("possible collision detected")
                self.collided_once = True
        else:
            self.collided_once = False
            self.is_colliding = False
            self.collision_side = None

    def pose_cb(self, data):
        new_pose = data.pose.pose
        self.pose = new_pose

    def pprint_laser(self, data):
        print(data.ranges.index(min(data.ranges)))


def begin():
    rospy.init_node('husky_bounce', anonymous=True)
    robot = Robot()

    while not rospy.is_shutdown():
        robot.forward()
        robot.turn()


if __name__ == '__main__':
    begin()
