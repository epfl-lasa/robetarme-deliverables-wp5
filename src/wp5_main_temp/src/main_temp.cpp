#include "IRoboticArmBase.h"
#include "RoboticArmUr5.h"
#include "ros/ros.h"
// #include <Eigen/Dense>
// #include <eigen3/Eigen/Dense>
#include "RosInterfaceNoetic.h"
#include <tuple>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_temporary");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);

    // Create an instance of RosInterfaceNoetic
    RosInterfaceNoetic rosInterface(nh);
    IRoboticArmBase iRoboticArmBase;
    RoboticArmUr5 ur5Arm;


    tuple<vector<double>, vector<double>, vector<double>> stateJoints;
    while (ros::ok()){
        stateJoints = rosInterface.receive_state();
        vector<double>& retrievedPosition = get<0>(stateJoints);
        vector<double>& retrievedSpeed    = get<1>(stateJoints);
        vector<double>& retrievedTorque   = get<2>(stateJoints);
        std::cout << "retrievedPosition:"<<retrievedPosition[3]<< std::endl;
        ros::spinOnce(); // Allow the message to be subscribed
        loop_rate.sleep(); 
    }

    return 0;
}
