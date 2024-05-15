## Config
Here you have all the file that you need to verify before launching the code.

arm_robot_config:
This one has all the caracteristic of the robot that you want to use. The filed for IIWA 7 and Ur5 is already set. But we add Cobod for futur works.

control_config:
Here you all the the info for the dynamical system. For example, you can choose to change the radius or speed of the limit cycle.

robot_task_config:
This is is here to specif which robot you will use depending on the task.

ros_interface_config:
You can set the desired topic to communicate with ros Noetic. You have the topics for the joint_state, the comande, or the force torque sensor for exemple.

target_config:
This one is here to extract the target from optitrack. This package will be replace by information given by other partners from vision system.


## Maintainers
- Tristan Bonato - <tristan_bonato@hotmail.com>
- Rui Wu - <wurui19930213@gmail.com>
