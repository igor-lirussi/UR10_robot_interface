# UR10_robot_interface
**UR10 Robot interface for controlling it with ROS**
## Description 
This repo allows you to control easily the UR10 robot from your computer with ROS, sending commands to retrieve joint/cartesian position or to move the robot to desired positions.
There are two versions of the interface, one with [MoveIt](https://moveit.ros.org) motion planning framework (suggested) and one sending directly the commands to the ROS topics, if you have problems with MoveIt installation.

### Topics:
- Robotics
- UR10

## Result
![Result](./img/result.jpg)

![Result](./img/image.gif)

## Requirements & Dependencies
- ROS 
- ros-noetic-moveit
- rospy and numpy
- https://github.com/ros-industrial/universal_robot build in your system with catkin
- https://github.com/UniversalRobots/Universal_Robots_ROS_Driver build in your system with catkin

(Please follow [this tutorial](https://jaspereb.github.io/UR5_With_ROS_Moveit_Tutorial/) for installation)


## Install 
*   clone in your project
*   remember to source the setup.bash of catkin
*   run the node to communicate with the robot
*   (if you use moveit) run the node to plan
*   use the interface as in the "Run" section
*   (optional)(if you use moveit) run the node to have the RViz GUI visualization

## Run
```python
from ur10_interface_moveit import UR10
import rospy

rospy.init_node("ur10moving")
rospy.sleep(1.0)
robot = UR10()

pos=robot.get_joint_position()
print(pos)

robot.table_pose()
robot.set_joint_position([-0.015, -1.7569, -1.3694, -1.4978, 1.7183, -2.4595])
```

## Useful Resources & Extra
- https://jaspereb.github.io/UR5_With_ROS_Moveit_Tutorial/
- https://youtu.be/18SQssJ-l_Y
- https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
- for ROS2: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver

## Authors
* **Igor Lirussi** @ BOUN Boğaziçi University - CoLoRs (Cognitive Learning and Robotics) Lab

## Acknowledgments
*   All the people that contributed with suggestions and tips.

## License
This project is licensed - see the [LICENSE](LICENSE) file for details.
