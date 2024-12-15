#!/usr/bin/env python
import rospy
import random
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WanderBot:
    def __init__(self):
        rospy.init_node('wander_bot', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.vel_msg = Twist()
        self.forward_speed = 0.3  # Reduced forward speed
        self.turn_speed = 0.7  # Reduced turn speed
        self.state = "explore"  # Possible states: "explore", "turn"
        self.explore_timer = rospy.Timer(rospy.Duration(7), self.change_exploration_direction)  # Faster exploration changes

    def scan_callback(self, scan):
        try:
            # Filter invalid values (0.0, inf, NaN) from ranges
            front_ranges = [r for r in scan.ranges[0:15] + scan.ranges[345:359] if 0.1 < r < 10 and not math.isinf(r)]
            left_ranges = [r for r in scan.ranges[60:120] if 0.1 < r < 10 and not math.isinf(r)]
            right_ranges = [r for r in scan.ranges[240:300] if 0.1 < r < 10 and not math.isinf(r)]

            # Assign default values if no valid ranges exist
            front = min(front_ranges) if front_ranges else 10
            left = min(left_ranges) if left_ranges else 10
            right = min(right_ranges) if right_ranges else 10

            if self.state == "explore":
                if front < 0.5:
                    # Obstacle detected, decide to turn left or right
                    self.state = "turn"
                    self.decide_turn_direction(left, right)
                else:
                    # Explore freely with dynamic angular movement
                    self.vel_msg.linear.x = self.forward_speed
                    self.vel_msg.angular.z = random.uniform(-0.4, 0.4)
            elif self.state == "turn":
                if front >= 0.7:  # Enough clearance to explore again
                    self.state = "explore"
                    self.vel_msg.linear.x = self.forward_speed
                    self.vel_msg.angular.z = random.uniform(-0.4, 0.4)
                else:
                    # Keep turning to avoid obstacle
                    self.vel_msg.linear.x = 0.0

            self.pub.publish(self.vel_msg)
        except Exception as e:
            rospy.logwarn(f"Error processing LaserScan data: {e}")

    def change_exploration_direction(self, event):
        if self.state == "explore":
            # Introduce a random turn during exploration to simulate curiosity
            self.vel_msg.angular.z = random.uniform(-1.0, 1.0)  # Wider range of angular velocities
            self.pub.publish(self.vel_msg)

    def decide_turn_direction(self, left, right):
        if left > right:
            rospy.loginfo("Turning left to avoid obstacle")
            self.vel_msg.angular.z = self.turn_speed
        elif right > left:
            rospy.loginfo("Turning right to avoid obstacle")
            self.vel_msg.angular.z = -self.turn_speed
        else:
            rospy.loginfo("Randomly choosing turn direction")
            self.vel_msg.angular.z = random.choice([-self.turn_speed, self.turn_speed])

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        wander_bot = WanderBot()
        wander_bot.run()
    except rospy.ROSInterruptException:
        pass

