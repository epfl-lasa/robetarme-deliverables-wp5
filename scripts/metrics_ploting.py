import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64MultiArray
import sys
import select


class DataSubscriber:
    def __init__(self):
        rospy.init_node('data_subscriber', anonymous=True)
        self.force_data = []
        self.desired_twist_data = []
        self.actual_twist_data = []
        self.record_time = 0
        self.start_time = None
        self.force_sub = rospy.Subscriber('/ft_sensor/netft_data', WrenchStamped, self.force_callback)
        self.desired_twist_sub = rospy.Subscriber('/desiredDsTwist', Float64MultiArray, self.desired_twist_callback)
        self.actual_twist_sub = rospy.Subscriber('/actualCartesianTwistEEF', Float64MultiArray, self.actual_twist_callback)

    def force_callback(self, msg):
        if self.start_time is not None:
            self.force_data.append(msg)

    def desired_twist_callback(self, msg):
        if self.start_time is not None:
            self.desired_twist_data.append(msg)

    def actual_twist_callback(self, msg):
        if self.start_time is not None:
            self.actual_twist_data.append(msg)

    def record(self, seconds):
        self.record_time = seconds
        self.start_time = rospy.get_time()

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        print("Press an integer to start recording for that many seconds:")
        while not rospy.is_shutdown():
            if self.start_time is not None and rospy.get_time() - self.start_time >= self.record_time:
                print("Recording stopped.")
                break

            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                key = int(sys.stdin.readline().strip())
                print("Recording for {} seconds...".format(key))
                self.record(key)

            rate.sleep()

def plot_data(force_data, desired_twist_data, actual_twist_data):
    force_time = [msg.header.stamp.to_sec() for msg in force_data]
    force_x = [msg.wrench.force.x for msg in force_data]
    force_y = [msg.wrench.force.y for msg in force_data]
    force_z = [msg.wrench.force.z for msg in force_data]

    twist_linear_x_desired = [msg.data[0] for msg in desired_twist_data]
    twist_linear_y_desired = [msg.data[1] for msg in desired_twist_data]
    twist_linear_z_desired = [msg.data[2] for msg in desired_twist_data]
    twist_angular_x_desired = [msg.data[3] for msg in desired_twist_data]
    twist_angular_y_desired = [msg.data[4] for msg in desired_twist_data]
    twist_angular_z_desired = [msg.data[5] for msg in desired_twist_data]

    twist_linear_x_actual = [msg.data[0] for msg in actual_twist_data]
    twist_linear_y_actual = [msg.data[1] for msg in actual_twist_data]
    twist_linear_z_actual = [msg.data[2] for msg in actual_twist_data]
    twist_angular_x_actual = [msg.data[3]for msg in actual_twist_data]
    twist_angular_y_actual = [msg.data[4] for msg in actual_twist_data]
    twist_angular_z_actual = [msg.data[5]for msg in actual_twist_data]

    # Plot Force
    plt.figure(figsize=(10, 9))
    plt.subplot(2, 1, 1)
    plt.plot(force_time, force_x, label='Force X')
    plt.plot(force_time, force_y, label='Force Y')
    plt.plot(force_time, force_z, label='Force Z')
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.title('Force Data')
    plt.legend()

    # Plot Desired Linear Speed
    plt.subplot(2, 1, 2)
    plt.plot(twist_linear_x_desired, label='Linear X Desired')
    plt.plot(twist_linear_y_desired, label='Linear Y Desired')
    plt.plot(twist_linear_z_desired, label='Linear Z Desired')
    plt.plot(twist_linear_x_actual, label='Linear X Actual')
    plt.plot(twist_linear_y_actual, label='Linear Y Actual')
    plt.plot(twist_linear_z_actual, label='Linear Z Actual')
    plt.xlabel('Index')
    plt.ylabel('Linear Speed (m/s)')
    plt.title('Desired Linear Speed Data')
    plt.legend()

    # # Plot Actual Linear Speed
    # plt.subplot(3, 1, 3)
    # plt.plot(twist_angular_x_desired, label='angular X Desired')
    # plt.plot(twist_angular_y_desired, label='angular Y Desired')
    # plt.plot(twist_angular_z_desired, label='angular Z Desired')    
    # plt.plot(twist_angular_x_actual, label='angular X Actual')
    # plt.plot(twist_angular_y_actual, label='angular Y Actual')
    # plt.plot(twist_angular_z_actual, label='angular Z Actual')
    # plt.xlabel('Index')
    # plt.ylabel('Linear Speed (m/s)')
    # plt.title('Actual Linear Speed Data')
    # plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    data_subscriber = DataSubscriber()
    try:
        data_subscriber.run()
        plot_data(data_subscriber.force_data, data_subscriber.desired_twist_data, data_subscriber.actual_twist_data)
    except rospy.ROSInterruptException:
        pass
