#this example uses the interface with Moveit installed
from ur10_interface_moveit import UR10
import rospy

rospy.init_node("ur10moving")
rospy.sleep(2.0)
robot = UR10()
rospy.sleep(2.0)

pos=robot.get_joint_position()
print(pos)

print('press enter button to go table pose')
input()
robot.table_pose()
rospy.sleep(2.0)

print("going to zero pose")
robot.zero_pose()