#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, PolygonStamped, Point32  # Thêm dòng này
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import DBSCAN

class LidarProcessor:
    def __init__(self):
        rospy.init_node('lidar_processor')
        
        # Load parameters
        self.min_range = rospy.get_param('~min_range', 0.1)
        self.max_range = rospy.get_param('~max_range', 4.0)
        self.eps = rospy.get_param('~eps', 0.2)
        self.min_samples = rospy.get_param('~min_samples', 5)
        self.crop_width = rospy.get_param('~crop_width', 0.5)
        
        # Publishers
        self.nearest_pub = rospy.Publisher('/nearest_obstacle', PointStamped, queue_size=10)
        self.crop_pub = rospy.Publisher('/crop_rows', MarkerArray, queue_size=10)
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.marker_array_pub = rospy.Publisher('/crop_markers', MarkerArray, queue_size=10)
        
        # Subscriber
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        rospy.loginfo("LIDAR Processor initialized")

    def scan_callback(self, scan):
        # Convert to numpy array
        ranges = np.array(scan.ranges)
        
        # Filter invalid data
        filtered_ranges = np.where(
            (ranges >= self.min_range) & (ranges <= self.max_range),
            ranges,
            np.nan
        )
        
        # Convert to cartesian coordinates
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        x = filtered_ranges * np.cos(angles)
        y = filtered_ranges * np.sin(angles)
        
        # Remove NaN points
        valid_idx = ~np.isnan(x) & ~np.isnan(y)
        points = np.vstack((x[valid_idx], y[valid_idx])).T
        
        # Process data
        self.detect_nearest_obstacle(points)
        crop_rows = self.detect_crop_rows(points)
        self.visualize_results(points, crop_rows)
    
    def detect_nearest_obstacle(self, points):
        if len(points) == 0:
            return
            
        # Calculate distances to origin (robot position)
        distances = np.linalg.norm(points, axis=1)
        min_idx = np.argmin(distances)
        nearest_point = points[min_idx]
        
        # Publish nearest obstacle
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "laser"
        msg.point.x = nearest_point[0]
        msg.point.y = nearest_point[1]
        self.nearest_pub.publish(msg)
    
    def detect_crop_rows(self, points):
        if len(points) < 10:
            return []
            
        # Cluster points using DBSCAN
        clustering = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(points)
        
        # Group points by cluster
        unique_labels = set(clustering.labels_)
        clusters = []
        for label in unique_labels:
            if label == -1:  # Skip noise
                continue
            cluster_points = points[clustering.labels_ == label]
            if len(cluster_points) > 10:  # Minimum points threshold
                clusters.append(cluster_points)
        
        # Sort clusters by x position
        clusters.sort(key=lambda c: np.mean(c[:,0]))
        
        return clusters
    
    def visualize_results(self, points, crop_rows):
        # Create marker array for all points
        marker_array = MarkerArray()
        
        # Visualize raw points
        marker = Marker()
        marker.header.frame_id = "laser"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "lidar_points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        
        for point in points:
            p = Point32()
            p.x = point[0]
            p.y = point[1]
            marker.points.append(p)
        
        marker_array.markers.append(marker)
        
        # Visualize crop rows
        for i, row in enumerate(crop_rows):
            # Calculate centroid and direction
            centroid = np.mean(row, axis=0)
            
            marker = Marker()
            marker.header.frame_id = "laser"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "crop_rows"
            marker.id = i + 1
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = centroid[0]
            marker.pose.position.y = centroid[1]
            marker.pose.position.z = 0
            marker.scale.x = 0.2
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.g = 1.0
            
            # Add direction based on row orientation
            cov = np.cov(row.T)
            eigvals, eigvecs = np.linalg.eig(cov)
            direction = eigvecs[:, np.argmax(eigvals)]
            angle = np.arctan2(direction[1], direction[0])
            
            marker.pose.orientation.z = np.sin(angle/2)
            marker.pose.orientation.w = np.cos(angle/2)
            
            marker_array.markers.append(marker)
        
        self.marker_array_pub.publish(marker_array)

if __name__ == '__main__':
    processor = LidarProcessor()
    rospy.spin()