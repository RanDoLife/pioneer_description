#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import math

class MovementController:
    def __init__(self):
        rospy.init_node("movement_controller")

        self.left_pub = rospy.Publisher('/left_wheel_effort_controller/command', Float64, queue_size=1)
        self.right_pub = rospy.Publisher('/right_wheel_effort_controller/command', Float64, queue_size=1)
        self.steer_pub = rospy.Publisher('/front_yaw_position_controller/command', Float64, queue_size=1)

        rospy.Subscriber('/move_distance', Float64, self.move_callback)
        rospy.Subscriber('/steer_angle', Float64, self.steer_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.start_pos = None      # стартовая позиция (x, y)
        self.target_distance = 0.0
        self.moving = False
        self.current_distance = 0.0

        self.rate = rospy.Rate(10)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        current_pos = (pos.x, pos.y)

        if self.moving:
            if self.start_pos is None:
                self.start_pos = current_pos
                self.current_distance = 0.0
            else:
                dx = current_pos[0] - self.start_pos[0]
                dy = current_pos[1] - self.start_pos[1]
                self.current_distance = math.sqrt(dx*dx + dy*dy)
                rospy.loginfo(f"Distance traveled: {self.current_distance:.3f} m")

                if self.current_distance >= abs(self.target_distance):
                    # Остановить движение
                    self.left_pub.publish(0.0)
                    self.right_pub.publish(0.0)
                    rospy.loginfo("Target distance reached. Stopping.")
                    self.moving = False

    def move_callback(self, msg):
        dist = msg.data
        rospy.loginfo(f"Received move command: {dist} meters")
        self.start_pos = None
        self.target_distance = dist
        self.moving = True
        effort = 1.0 if dist > 0 else -1.0
        self.left_pub.publish(effort)
        self.right_pub.publish(effort)

    def steer_callback(self, msg):
        angle = msg.data
        rospy.loginfo(f"Received steer command: {angle} radians")
        self.steer_pub.publish(angle)

    def spin(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":
    MovementController().spin()
