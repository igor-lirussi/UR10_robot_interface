from ur10_interface import UR10
import rospy
# this example uses package ur_kinematics to calculate inverse kinematics when using the ur10_interface without moveit.
# if possible use the interface for Moveit installed in the system
from ur_ikfast import ur_kinematics

rospy.init_node("ur10moving")
rospy.sleep(2.0)
robot = UR10()
rospy.sleep(2.0)

ur_kin = ur_kinematics.URKinematics('ur10')

pos=robot.get_joint_position()
print('real position\n',pos)

pose_matrix= ur_kin.forward(pos, 'matrix')
print('calculated cartesian position\n',pose_matrix)

new_joint_pos = ur_kin.inverse(pose_matrix, False, q_guess=pos)
print('calculated joint position\n',new_joint_pos)


print('press enter button to go table pose')
input()
robot.table_pose()
rospy.sleep(2.0)

print("going to zero pose")
robot.zero_pose()