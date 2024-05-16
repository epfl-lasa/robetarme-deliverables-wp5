import rospy
import matplotlib
# matplotlib.use('Agg')  # Use Agg backend for non-interactive plotting
import matplotlib.pyplot as plt
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import sys
import math
import select
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
class DataSubscriber:
    def __init__(self):
        rospy.init_node('data_subscriber', anonymous=True)
        self.force_data = []
        self.desired_twist_data = []
        self.actual_twist_data = []
        self.actual_pose_data = []  # Added for actual pose data
        self.record_time = 0
        self.start_time = None
        self.force_sub = rospy.Subscriber('/ft_sensor/netft_data', WrenchStamped, self.force_callback)
        self.desired_twist_sub = rospy.Subscriber('/desiredDsTwist', Float64MultiArray, self.desired_twist_callback)
        self.actual_twist_sub = rospy.Subscriber('/actualCartesianTwistEEF', Float64MultiArray, self.actual_twist_callback)
        self.actual_pose_sub = rospy.Subscriber('/actualCartesianPoseEEF', PoseStamped, self.actual_pose_callback)  # Subscriber for actual pose data
        self.first_eef_received = False
    def force_callback(self, msg):
        if self.start_time is not None:
            self.force_data.append(msg)
    def desired_twist_callback(self, msg):
        if self.start_time is not None:
            self.desired_twist_data.append(msg)
    def actual_twist_callback(self, msg):
        if msg:
            self.first_eef_received = True
        if self.start_time is not None:
            self.actual_twist_data.append(msg)
    def actual_pose_callback(self, msg):  # Callback function for actual pose data
        if self.start_time is not None:
            self.actual_pose_data.append(msg)
    def record(self, seconds):
        self.record_time = seconds
        self.start_time = rospy.get_time()
    def run(self):
        key = 0
        rate = rospy.Rate(10)  # 10 Hz
        print("Press an integer to start recording for that many seconds:")
        while not rospy.is_shutdown():
            if self.start_time is not None and rospy.get_time() - self.start_time >= self.record_time:
                print("Recording stopped.")
                break
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                key = int(sys.stdin.readline().strip())
                print("it will record for {} seconds".format(key))
                print("Waiting to receive topics eef..")
            if key > 0 and self.first_eef_received == True and self.record_time == 0 :
                print("start recording..")
                self.record(key)
            rate.sleep()
    def plot_data(self, force_data, desired_twist_data, actual_twist_data, actual_pose_data):
        start_time = self.start_time
        force_time = [msg.header.stamp.to_sec() for msg in force_data]
        force_x = [msg.wrench.force.x for msg in force_data]
        force_y = [msg.wrench.force.y for msg in force_data]
        force_z = [msg.wrench.force.z for msg in force_data]
        twist_linear_x_desired = [msg.data[0] for msg in desired_twist_data]
        twist_linear_y_desired = [msg.data[1] for msg in desired_twist_data]
        twist_linear_z_desired = [msg.data[2] for msg in desired_twist_data]
        twist_linear_x_actual = [msg.data[0] for msg in actual_twist_data]
        twist_linear_y_actual = [msg.data[1] for msg in actual_twist_data]
        twist_linear_z_actual = [msg.data[2] for msg in actual_twist_data]


        # Calculate errors at each step
        errors = []
        for dx, dy, dz, ax, ay, az in zip(twist_linear_x_desired, twist_linear_y_desired, twist_linear_z_desired,
                                        twist_linear_x_actual, twist_linear_y_actual, twist_linear_z_actual):
            error = math.sqrt((dx - ax) ** 2 + (dy - ay) ** 2 + (dz - az) ** 2)
            errors.append(error)

        # Calculate average error
        average_error = sum(errors) / len(errors)

        # Calculate RMSE
        rmse = math.sqrt(sum(e ** 2 for e in errors) / len(errors))


        # Extract position data from actual_pose_data
        eef_time = [msg.header.stamp.to_sec() for msg in actual_pose_data]
        eef_time = np.array(eef_time) - eef_time[0]
        eef_size = len(eef_time) -10
        positions = np.array([np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]) for msg in actual_pose_data])

        # Plot Force
        plt.figure(figsize=(15, 9))
        plt.plot(force_time, force_x, label='Force X')
        plt.plot(force_time, force_y, label='Force Y')
        plt.plot(force_time, force_z, label='Force Z')
        plt.xlabel('Time (s)')
        plt.ylabel('Force (N)')
        plt.title('Force Data')
        plt.legend()
        plt.tight_layout()
        # plt.savefig("../plots/" + str(start_time) + "_force.png")
        # plt.close()
        plt.show()
        
        # Split the Desired vs. Actual Linear Speed plot into three subplots
        plt.figure(figsize=(15, 9))

        # Subplot 1: X Linear Speed
        plt.subplot(3, 1, 1)
        plt.plot(eef_time[100:eef_size], twist_linear_x_desired[100:eef_size], label=r'$\dot{x}_{d1}$', linestyle='--',color='r')
        plt.plot(eef_time[100:eef_size], twist_linear_x_actual[100:eef_size], label=r'$\dot{x}_{a1}$',color='r')
        plt.xlabel('Time (s)', fontsize=14)
        plt.ylabel('Linear Speed X (m/s)', fontsize=14)
        plt.title('Desired vs. Actual Linear Speed X', fontsize=16)
        plt.legend(loc='upper right')

        # Subplot 2: Y Linear Speed
        plt.subplot(3, 1, 2)
        plt.plot(eef_time[100:eef_size], twist_linear_y_desired[100:eef_size], label=r'$\dot{x}_{d2}$', linestyle='--',color='g')
        plt.plot(eef_time[100:eef_size], twist_linear_y_actual[100:eef_size], label=r'$\dot{x}_{a2}$',color='g')
        plt.xlabel('Time (s)', fontsize=14)
        plt.ylabel('Linear Speed Y (m/s)', fontsize=14)
        plt.title('Desired vs. Actual Linear Speed Y', fontsize=16)
        plt.legend(loc='upper right')

        # Subplot 3: Z Linear Speed
        plt.subplot(3, 1, 3)
        plt.plot(eef_time[100:eef_size], twist_linear_z_desired[100:eef_size], label=r'$\dot{x}_{d3}$', linestyle='--',color='b')
        plt.plot(eef_time[100:eef_size], twist_linear_z_actual[100:eef_size], label=r'$\dot{x}_{a3}$',color='b')
        plt.xlabel('Time (s)', fontsize=14)
        plt.ylabel('Linear Speed Z (m/s)', fontsize=14)
        plt.title('Desired vs. Actual Linear Speed Z', fontsize=16)
        plt.legend(loc='upper right')


        plt.tight_layout()
        # plt.savefig("../plots/" + str(start_time) + "_desired_vs_actual_linear_speed.png")
        # plt.close()
        plt.show()

        # Plot Trajectory
        fig = plt.figure(figsize=(15, 9))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot3D(positions[100:, 0], positions[100:, 1], positions[100:, 2])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Actual Path')
        # Set equal scaling for all dimensions
        max_range = np.array([positions[100:,0].max()-positions[100:,0].min(), positions[100:,1].max()-positions[100:,1].min(), positions[100:,2].max()-positions[100:,2].min()]).max()
        mid_x = (positions[100:,0].max()+positions[100:,0].min()) * 0.5
        mid_y = (positions[100:,1].max()+positions[100:,1].min()) * 0.5
        mid_z = (positions[100:,2].max()+positions[100:,2].min()) * 0.5
        ax.set_xlim(mid_x - max_range * 0.5, mid_x + max_range * 0.5)
        ax.set_ylim(mid_y - max_range * 0.5, mid_y + max_range * 0.5)
        ax.set_zlim(mid_z - max_range * 0.5, mid_z + max_range * 0.5)
        plt.tight_layout()
        # plt.savefig("../plots/" + str(start_time) + "_actualPath.png")
        # plt.close()
        plt.show()


        # Plot RMSE
        plt.figure(figsize=(15, 9))
        plt.plot(eef_time[100:eef_size], errors[100:eef_size], label='errors')
        plt.axhline(y=rmse, color='r', linestyle='--', label='RMSE: {:.4f}'.format(rmse))
        plt.axhline(y=average_error, color='b', linestyle='--', label='average_error: {:.4f}'.format(average_error))
        plt.xlabel('Time (s)')
        plt.ylabel('RMSE')
        plt.legend()
        plt.tight_layout()
        # plt.savefig("../plots/" + str(start_time) + "_rmse.png")
        # plt.close()
        plt.show()

if __name__ == '__main__':
    data_subscriber = DataSubscriber()
    try:
        data_subscriber.run()
        data_subscriber.plot_data(data_subscriber.force_data, data_subscriber.desired_twist_data, data_subscriber.actual_twist_data, data_subscriber.actual_pose_data)
    except rospy.ROSInterruptException:
        pass
