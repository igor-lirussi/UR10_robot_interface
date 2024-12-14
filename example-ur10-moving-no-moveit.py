"""
Description: Example of usage of UR10 interface without moveit
Author: Igor Lirussi (https://igor-lirussi.github.io)
"""
from ur10_interface import UR10 # this example uses ur10_interface without moveit and doesn't have "get_cartesian_position". # if possible use the interface for Moveit installed in the system
import rospy

rospy.init_node("ur10moving")
rospy.sleep(2.0)
robot = UR10()
rospy.sleep(2.0)

cart=robot.get_cartesian_position()
print(cart)

pos=robot.get_joint_position()
print('real position\n',pos)

print('press enter button to move to position, remember to keep the robot emergency stop red button next to you')
input()
robot.set_joint_position([-0.015, -1.7569, -1.3694, -1.4978, 1.7183, -2.4595])

print('press enter button to go table pose and then zero pose')
input()
robot.table_pose()

rospy.sleep(2.0)

robot.zero_pose()