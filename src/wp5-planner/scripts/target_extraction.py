import rospy
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
import os
import rospkg
import time
import open3d as o3d
from std_msgs.msg import Header

class PointCloudTransformer:
    def __init__(self):
        self.depth_image = None
        self.record_pressed = False
        self.record_num = 0
        self.points = []
        self.roi_mask = None
        self.depth_images = []
        self.mean_points_array = []
        self.i_for_mean = 0
        self.rospack = rospkg.RosPack()
        self.package_path = self.rospack.get_path('wp5_perception')

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.points.append((x, y))
            if len(self.points) == 4:
                self.roi_mask = np.zeros_like(self.color_image[:, :, 0])
                roi_corners = np.array([self.points], dtype=np.int32)
                cv2.fillPoly(self.roi_mask, roi_corners, (255,))
                self.save_points_to_file(self.points)
            elif len(self.points) > 4:
                self.points = self.points[:-1]
                self.roi_mask = np.zeros_like(self.color_image[:, :, 0])
                roi_corners = np.array([self.points], dtype=np.int32)
                cv2.fillPoly(self.roi_mask, roi_corners, (255,))

    def save_points_to_file(self, points):
        with open(self.package_path + '/data/target_boundary/points.txt', 'w') as file:
            for point in points:
                file.write(f"{point[0]},{point[1]}\n")

    def color_image_callback(self, msg):
        bridge = CvBridge()
        self.color_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def depth_image_callback(self, msg):
        bridge = CvBridge()
        self.depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def point_cloud_callback(self, color_msg, depth_msg, publisher):
        if self.roi_mask is None:
            return
        depth_values = self.depth_image[self.roi_mask != 0] / 1000.0
        v, u = np.where(self.roi_mask != 0)
        x = (u - 320) * depth_values / 570
        y = (v - 240) * depth_values / 570
        z = depth_values

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "base_link"

        points_array = np.vstack((x, y, z)).T.astype(np.float32)
        point_cloud_msg = pc2.create_cloud_xyz32(header, points_array)
        publisher.publish(point_cloud_msg)

    def save_color_image(self, image):
        directory = self.package_path + "/data/pictures_target"
        if not os.path.exists(directory):
            os.makedirs(directory)
        file_path = os.path.join(directory, "target_image.png")
        cv2.imwrite(file_path, image)

    def point_cloud_callback_transformed(self, msg):
        points_list = []
        for point in pc2.read_points(msg, skip_nans=True):
            points_list.append([point[0], point[1], point[2]])
        if len(points_list) == 0:
            rospy.logwarn("Received empty point cloud")
            return

        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(np.array(points_list))
        transformation_matrix = np.load(self.package_path + "/data/matrix/Y1_matrices.npy")
        point_cloud_transformed = point_cloud.transform(transformation_matrix)

        points_transformed = np.asarray(point_cloud_transformed.points)
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "base_link"

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]

        point_cloud2_msg = pc2.create_cloud(header, fields, points_transformed)
        self.transformed_pc_pub.publish(point_cloud2_msg)

        if self.record_pressed:
            self.record_num += 1
            self.record_pressed = False
            output_file_path = self.package_path + "/data/pointclouds/transformed_pointcloud" + str(self.record_num) + ".ply"
            success = o3d.io.write_point_cloud(output_file_path, point_cloud_transformed, write_ascii=True)
            if not success:
                rospy.logerr("Failed to save transformed point cloud.")

    def main(self):
        time.sleep(5)
        rospy.init_node('point_cloud_transformer', anonymous=True)

        rospy.Subscriber('/camera/depth/points_crop', PointCloud2, self.point_cloud_callback_transformed)
        self.transformed_pc_pub = rospy.Publisher('/camera/depth/points_crop_transformed', PointCloud2, queue_size=100)
        rate = rospy.Rate(3)

        rospy.Subscriber("/camera/color/image_raw", Image, self.color_image_callback)
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback)
        publisher = rospy.Publisher("/camera/depth/points_crop", PointCloud2, queue_size=10)

        color_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        depth_sub = message_filters.Subscriber("/camera/depth/image_rect_raw", Image)
        ts = message_filters.TimeSynchronizer([color_sub, depth_sub], 10)
        ts.registerCallback(self.point_cloud_callback, publisher)

        cv2.namedWindow("Color Image")
        cv2.setMouseCallback("Color Image", self.mouse_callback)

        while not rospy.is_shutdown():
            if self.roi_mask is not None:
                color_image_with_roi = cv2.addWeighted(self.color_image, 0.7, cv2.cvtColor(self.roi_mask, cv2.COLOR_GRAY2BGR), 0.3, 0)
                cv2.imshow("Color Image", color_image_with_roi)
                self.save_color_image(color_image_with_roi)

            else:
                cv2.imshow("Color Image", self.color_image)

            key = cv2.waitKey(10)
            if key == 27:
                break
            if key == ord('r'):
                self.record_pressed = True
            rate.sleep()

        cv2.destroyAllWindows()

if __name__ == '__main__':
    pc_transformer = PointCloudTransformer()
    pc_transformer.main()
