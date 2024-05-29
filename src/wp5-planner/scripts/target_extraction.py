import rospy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
import os

# Global variables to store the color image, depth image, ROI mask, and a flag indicating if the record key is pressed
depth_image = None
record_pressed = False
points = []
roi_mask = None
depth_images = []
mean_points_array = []
iForMean= 0
# Mouse event handler function to select points for ROI
def mouse_callback(event, x, y, flags, param):
    global points, roi_mask
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))
        if len(points) == 4:
            roi_mask = np.zeros_like(color_image[:, :, 0])
            roi_corners = np.array([points], dtype=np.int32)
            cv2.fillPoly(roi_mask, roi_corners, (255,))
        elif len(points) > 4:
            points = points[:-1]  # Limit to 4 points
            roi_mask = np.zeros_like(color_image[:, :, 0])
            roi_corners = np.array([points], dtype=np.int32)
            cv2.fillPoly(roi_mask, roi_corners, (255,))

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
    header.frame_id = "camera_depth_frame"

    points_array = np.vstack((x, y, z)).T.astype(np.float32)
    point_cloud_msg = pc2.create_cloud_xyz32(header, points_array)

    # Publish the PointCloud2 message
    # publisher.publish(point_cloud_msg)
    depth_images = []
    # Save point cloud as .ply file if record key is pressed
    global record_pressed
    global iForMean
    if record_pressed:
        print("Recording...")
        print(iForMean)
        if (iForMean == 0):
            # Calculate mean point cloud from the next 100 frames
            mean_points_array = np.zeros_like(points_array)

        if len(depth_values) != 0:

            depth_values = np.array(depth_values) / 1000.0  # Convert mm to meters

            # Convert pixel coordinates to world coordinates
            x = (u - 320) * depth_values / 570
            y = (v - 240) * depth_values / 570
            z = depth_values

            # Accumulate points
            points_array = np.vstack((x, y, z)).T.astype(np.float32)
            mean_points_array += points_array
            iForMean  = iForMean +1

        if (iForMean == 20):
            
            # Calculate mean
            mean_points_array /= 20

            # Save mean point cloud
            save_point_cloud(mean_points_array)
            record_pressed = False  # Reset the flag after saving
            iForMean == 0
            print("... done ")


# Function to save point cloud as .ply file
def save_point_cloud(points_array):
    # Ensure the directory exists
    directory = "../data/pointclouds"
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
    directory = "../data/pictures_target"
    if not os.path.exists(directory):
        os.makedirs(directory)

    # Generate file path
    file_path = os.path.join(directory, "target_image.png")

    # Save color image
    cv2.imwrite(file_path, image)

# Main function
def main():
    check_display = True
    global record_pressed

    # Initialize ROS node
    rospy.init_node('color_depth_image_subscriber', anonymous=True)

    # Create subscribers for color and depth images
    rospy.Subscriber("/camera/color/image_raw", Image, color_image_callback)
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_image_callback)

    # Create a publisher for the cropped point cloud
    publisher = rospy.Publisher("/camera/depth/color/points_crop", PointCloud2, queue_size=10)

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
        if key == ord('r'):
            record_pressed = True
        elif key == 27:  # Check for escape key (ASCII code 27)
            break  # Exit loop if escape key is pressed
        elif not record_pressed and iForMean == 20:  # Exit loop if recording finished
            break

    # Close all OpenCV windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
