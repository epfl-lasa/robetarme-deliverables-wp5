#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped

def publish_data():
    # Initialize the ROS node
    rospy.init_node('data_publisher', anonymous=True)

    # Create publishers for the PoseStamped and WrenchStamped messages
    pose_publisher = rospy.Publisher('/vrpn_client_node/TargetRobetarme/pose_transform', PoseStamped, queue_size=10)
    wrench_publisher = rospy.Publisher('/robotiq_ft_sensor', WrenchStamped, queue_size=10)

    # Set the publishing rate (adjust as needed)
    rate = rospy.Rate(10)  # 10 Hz

    # Create a PoseStamped message
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = "base"
    pose_msg.pose.position.x = -0.061671819088715774  
    pose_msg.pose.position.y = -1.2231335817673443
    pose_msg.pose.position.z = 0.6230377347931229    
    pose_msg.pose.orientation.x = 0.7107592148598265
    pose_msg.pose.orientation.y = -0.010362802471094404
    pose_msg.pose.orientation.z = 0.01012737717438275
    pose_msg.pose.orientation.w = 0.7032861345486314

    # Create a WrenchStamped message
    wrench_msg = WrenchStamped()
    wrench_msg.header.frame_id = "robotiq_ft_frame_id"  # Set the appropriate frame ID
    wrench_msg.wrench.force.x = -40  # Modify force values as needed
    wrench_msg.wrench.force.y = 57
    wrench_msg.wrench.force.z = -37
    wrench_msg.wrench.torque.x = -0.6  # Modify torque values as needed
    wrench_msg.wrench.torque.y = 0.14
    wrench_msg.wrench.torque.z = -0.8

    # Start publishing
    while not rospy.is_shutdown():
        # Update timestamp and sequence number
        pose_msg.header.stamp = rospy.Time.now()
        wrench_msg.header.stamp = rospy.Time.now()

        # Publish the messages
        pose_publisher.publish(pose_msg)
        wrench_publisher.publish(wrench_msg)

        # Sleep to control the publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_data()
    except rospy.ROSInterruptException:
        pass
