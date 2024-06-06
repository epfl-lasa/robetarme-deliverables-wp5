import rospy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import rospkg

class PointCloudTransformer:
    def __init__(self):
        self.transformed_pc_pub = None
        # Initialize the ROS package manager
        rospack = rospkg.RosPack()

        # Get the path of a specific ROS package
        package_path = rospack.get_path('wp5_planner')


    def point_cloud_callback(self, msg):
        # Convert ROS PointCloud2 message to Open3D PointCloud
        points_list = []

        for point in pc2.read_points(msg, skip_nans=True):
            points_list.append([point[0], point[1], point[2]])

        if len(points_list) == 0:
            rospy.logwarn("Received empty point cloud")
            return

        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(np.array(points_list))

        # Load transformation matrix from .npy file
        transformation_matrix = np.load(self.package_path + "/data/matrix/Y1_matrices.npy")

        # Apply transformation matrix to point cloud
        point_cloud_transformed = point_cloud.transform(transformation_matrix)

        # Convert Open3D PointCloud to ROS PointCloud2 message
        points_transformed = np.asarray(point_cloud_transformed.points)
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = msg.header.frame_id

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]

        point_cloud2_msg = pc2.create_cloud(header, fields, points_transformed)

        # Publish the transformed point cloud
        self.transformed_pc_pub.publish(point_cloud2_msg)

        # Optionally save the transformed point cloud data
        output_file_path = self.package_path + "/data/pointclouds/transformed_pointcloud.ply"
        success = o3d.io.write_point_cloud(output_file_path, point_cloud_transformed, write_ascii=True)
        if success:
            rospy.loginfo("Transformed point cloud saved successfully.")
        else:
            rospy.logerr("Failed to save transformed point cloud.")

    def main(self):

        rospy.init_node('point_cloud_transformer', anonymous=True)

        # Subscribe to the point cloud topic
        rospy.Subscriber('/camera/depth/points_crop_transformed', PointCloud2, self.point_cloud_callback)

        # Publisher for the transformed point cloud
        self.transformed_pc_pub = rospy.Publisher('/transformed_point_cloud', PointCloud2, queue_size=10)

        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    transformer = PointCloudTransformer()
    transformer.main()
