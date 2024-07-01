#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt

class PointCloudROISelector:
    def __init__(self):
        self.point_cloud_data = None
        self.selected_points = []

    def pointcloud_callback(self, msg):
        # Convert PointCloud2 message to numpy array
        gen = pc2.read_points(msg, skip_nans=True)
        self.point_cloud_data = np.array(list(gen))

    def select_roi(self):
        if self.point_cloud_data is None:
            rospy.logwarn("Point cloud data not received yet.")
            return

        fig, ax = plt.subplots()
        
        # Plotting point cloud in 2D YZ plane
        ax.scatter(self.point_cloud_data[:, 1], self.point_cloud_data[:, 2])

        # Interactive selection of points using mouse
        def on_click(event):
            if event.button == 1:  # Left mouse button
                self.selected_points.append((event.ydata, event.xdata))  # Swap y and x for YZ plane
                rospy.loginfo(f"Selected point: (Y, Z) = ({event.ydata}, {event.xdata})")
        
        fig.canvas.mpl_connect('button_press_event', on_click)

        ax.set_xlabel('Y')
        ax.set_ylabel('Z')

        plt.show()

    def run(self):
        rospy.init_node('pointcloud_roi_selector', anonymous=True)
        rospy.Subscriber('/toydata/pointcloud2', PointCloud2, self.pointcloud_callback)

        # Wait until we get the first message
        while self.point_cloud_data is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        rospy.loginfo("Point cloud data received.")

        self.select_roi()

        # Print selected points at the end
        rospy.loginfo("Selected points:")
        for point in self.selected_points:
            rospy.loginfo(f"Y: {point[0]}, Z: {point[1]}")

        rospy.spin()

if __name__ == '__main__':
    try:
        selector = PointCloudROISelector()
        selector.run()
    except rospy.ROSInterruptException:
        pass
