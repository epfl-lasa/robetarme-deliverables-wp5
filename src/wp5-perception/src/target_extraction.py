import rospy
from sensor_msgs.msg import Image, PointCloud2
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
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

# Global variables to store the color image, depth image, ROI mask, and a flag indicating if the record key is pressed
depth_image = None
record_pressed = False
recordNum = 0
points = []
roi_mask = None
depth_images = []
mean_points_array = []
iForMean= 0
rospack = rospkg.RosPack()
package_path = rospack.get_path('wp5_perception')

# Mouse event handler function to select points for ROI
def mouse_callback(event, x, y, flags, param):
    global points, roi_mask
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))
        if len(points) == 4:
            roi_mask = np.zeros_like(color_image[:, :, 0])
            roi_corners = np.array([points], dtype=np.int32)
            cv2.fillPoly(roi_mask, roi_corners, (255,))
            save_points_to_file(points)  # Save points to file
        elif len(points) > 4:
            points = points[:-1]  # Limit to 4 points
            roi_mask = np.zeros_like(color_image[:, :, 0])
            roi_corners = np.array([points], dtype=np.int32)
            cv2.fillPoly(roi_mask, roi_corners, (255,))

def save_points_to_file(points):

    with open(package_path + '/data/target_boundary/points.txt', 'w') as file:
        for point in points:
            file.write(f"{point[0]},{point[1]}\n")

# Define a callback function to handle incoming color image messages
def color_image_callback(msg):
    # Convert ROS Image message to OpenCV image
    global color_image
    bridge = CvBridge()
    color_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

# Define a callback function to handle incoming depth image messages
def depth_image_callback(msg):
    # Convert ROS Image message to OpenCV image
    global depth_image
    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

# Define a callback function to handle incoming point cloud messages
def point_cloud_callback(color_msg, depth_msg, publisher):
    global roi_mask
    global depth_image
    global mean_points_array
    # Check if either color_msg or depth_msg is empty

    if roi_mask is None:
        return  # Skip processing if ROI mask is not available

    # Extract ROI depth values
    depth_values = depth_image[roi_mask != 0] / 1000.0  # Convert mm to meters

    # Extract ROI pixel coordinates
    v, u = np.where(roi_mask != 0)

    # Convert pixel coordinates to world coordinates
    x = (u - 320) * depth_values / 570
    y = (v - 240) * depth_values / 570
    z = depth_values

    # Create PointCloud2 message
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "base_link"

    points_array = np.vstack((x, y, z)).T.astype(np.float32)
    point_cloud_msg = pc2.create_cloud_xyz32(header, points_array)

    # Publish the PointCloud2 message
    publisher.publish(point_cloud_msg)
    depth_images = []
    # Save point cloud as .ply file if record key is pressed
    global record_pressed
    global iForMean
    

# Function to save point cloud as .ply file
def save_point_cloud(points_array):
    # Ensure the directory exists
    directory = package_path + "/data/pointclouds"
    if not os.path.exists(directory):
        os.makedirs(directory)

    # Generate file path
    file_path = os.path.join(directory, "pointcloud_target.ply")

    # Write point cloud to .ply file
    with open(file_path, 'w') as file:
        file.write("ply\n")
        file.write("format ascii 1.0\n")
        file.write("element vertex {}\n".format(len(points_array)))
        file.write("property float x\n")
        file.write("property float y\n")
        file.write("property float z\n")
        file.write("end_header\n")
        for point in points_array:
            file.write("{} {} {}\n".format(point[0], point[1], point[2]))

# Function to save color image
def save_color_image(image):
    # Ensure the directory exists
    directory = package_path + "/data/pictures_target"
    if not os.path.exists(directory):
        os.makedirs(directory)

    # Generate file path
    file_path = os.path.join(directory, "target_image.png")

    # Save color image
    cv2.imwrite(file_path, image)

def point_cloud_callback_transformed(msg):
    global record_pressed
    global recordNum

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
    transformation_matrix = np.load(package_path + "/data/matrix/Y1_matrices.npy")

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
    transformed_pc_pub.publish(point_cloud2_msg)

    if record_pressed:
        recordNum =recordNum+1
        record_pressed = False
        # # Optionally save the transformed point cloud data
        output_file_path = package_path + "/data/pointclouds/transformed_pointcloud" +str(recordNum)+".ply"
        success = o3d.io.write_point_cloud(output_file_path, point_cloud_transformed, write_ascii=True)
        if not success:
            rospy.logerr("Failed to save transformed point cloud.")

# Main function
def main():
    global transformed_pc_pub
    # Wait for 5 seconds
    time.sleep(5)
    
    check_display = True
    global record_pressed

    # Initialize ROS node
    rospy.init_node('point_cloud_transformer', anonymous=True)
    
    
    # # Subscribe to the point cloud topic
    rospy.Subscriber('/camera/depth/points_crop', PointCloud2, point_cloud_callback_transformed)


    # Publisher for the transformed point cloud
    transformed_pc_pub = rospy.Publisher('/camera/depth/color/points_crop_transformed', PointCloud2, queue_size=100)
    rate = rospy.Rate(3)

    # Create subscribers for color and depth images
    rospy.Subscriber("/camera/color/image_raw", Image, color_image_callback)
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_image_callback)

    # Create a publisher for the cropped point cloud
    publisher = rospy.Publisher("/camera/depth/points_crop", PointCloud2, queue_size=10)
    
    # Create a point cloud subscriber using message_filters for synchronized processing
    color_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
    depth_sub = message_filters.Subscriber("/camera/depth/image_rect_raw", Image)
    ts = message_filters.TimeSynchronizer([color_sub, depth_sub], 10)
    ts.registerCallback(point_cloud_callback, publisher)

    # Create OpenCV window and set mouse and keyboard callback functions
    cv2.namedWindow("Color Image")
    cv2.setMouseCallback("Color Image", mouse_callback)

    # Loop to continuously display the color image and depth map
    while not rospy.is_shutdown():
        # Display the color image
        if check_display:
            if roi_mask is not None:
                # Overlay ROI mask on color image
                color_image_with_roi = cv2.addWeighted(color_image, 0.7, cv2.cvtColor(roi_mask, cv2.COLOR_GRAY2BGR), 0.3, 0)
                cv2.imshow("Color Image", color_image_with_roi)
                save_color_image(color_image_with_roi)
                check_display = False


            else:
                cv2.imshow("Color Image", color_image)

        # Check for keyboard events
        key = cv2.waitKey(10)

        if key == 27:  # Check for escape key (ASCII code 27)
            break  # Exit loop if escape key is pressed
        if key == ord('r'):
            record_pressed = True
        # elif not record_pressed and iForMean == 20:  # Exit loop if recording finished
        #     break
        elif key == ord('q'):
            break
        rate.sleep()
    # Close all OpenCV windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
