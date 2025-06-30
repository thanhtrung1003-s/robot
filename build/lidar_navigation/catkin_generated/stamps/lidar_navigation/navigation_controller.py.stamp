#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist, PointStamped, PolygonStamped
from visualization_msgs.msg import MarkerArray

class NavigationController:
    def __init__(self):
        rospy.init_node('navigation_controller')
        
        # Parameters
        self.target_distance = rospy.get_param('~target_distance', 0.5)
        self.max_speed = rospy.get_param('~max_speed', 0.2)
        self.max_steering = rospy.get_param('~max_steering', 0.5)
        
        # Subscribers
        rospy.Subscriber('/nearest_obstacle', PointStamped, self.obstacle_callback)
        rospy.Subscriber('/crop_rows', PolygonStamped, self.crops_callback)
        
        # Publisher
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Variables
        self.obstacle_position = None
        self.crop_rows = []
        
        # Start control loop
        self.control_loop()
    
    def obstacle_callback(self, msg):
        self.obstacle_position = (msg.point.x, msg.point.y)
    
    def crops_callback(self, msg):
        self.crop_rows = []
        for marker in msg.markers:
            if marker.ns == "crop_rows":
                self.crop_rows.append({
                    'position': (marker.pose.position.x, marker.pose.position.y),
                    'direction': marker.pose.orientation
                })
    
    def control_loop(self):
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            cmd = Twist()
            
            # Simple obstacle avoidance
            if self.obstacle_position:
                dist = np.hypot(self.obstacle_position[0], self.obstacle_position[1])
                if dist < self.target_distance:
                    # Turn away from obstacle
                    cmd.angular.z = -np.sign(self.obstacle_position[1]) * self.max_steering
                    cmd.linear.x = self.max_speed * 0.5
            
            # Follow crop rows if available
            elif len(self.crop_rows) >= 2:
                # Sort by x position
                sorted_rows = sorted(self.crop_rows, key=lambda r: r['position'][0])
                left_row = sorted_rows[0]
                right_row = sorted_rows[1]
                
                # Calculate center path
                center_x = (left_row['position'][0] + right_row['position'][0]) / 2
                center_y = (left_row['position'][1] + right_row['position'][1]) / 2
                
                # Simple P-controller to steer towards center
                cmd.angular.z = -0.5 * center_x
                cmd.linear.x = self.max_speed
            else:
                # Default behavior: move forward slowly
                cmd.linear.x = self.max_speed * 0.3
            
            self.cmd_pub.publish(cmd)
            rate.sleep()

if __name__ == '__main__':
    controller = NavigationController()