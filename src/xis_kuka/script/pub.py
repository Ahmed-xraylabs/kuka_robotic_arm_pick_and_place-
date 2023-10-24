#!/usr/bin/env python3


# import rospy
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from std_msgs.msg import Header
# import time

# def open_gripper():
#     rospy.init_node('open_gripper_node', anonymous=True)
#     pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)
#     rospy.sleep(2)  # Wait for 2 seconds

#     gripper_msg = JointTrajectory()
#     gripper_msg.header = Header()
#     gripper_msg.joint_names = ['gripper_joint1', 'gripper_joint2']

#     point = JointTrajectoryPoint()
#     point.positions = [0.5, 0.5]
#     point.velocities = [0, 0]
#     point.accelerations = [0, 0]
#     point.effort = [0, 0]
#     point.time_from_start = rospy.Duration(10, 10)

#     gripper_msg.points.append(point)
#     pub.publish(gripper_msg)

# try:
#     open_gripper()
#     # Keep the node alive with rospy.spin()
#     rospy.spin()
# except rospy.ROSInterruptException:
#     pass



# # import rospy
# # from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# # from std_msgs.msg import Header
# # import time
# # print('hello')

# # def open_gripper():
# #     rospy.init_node('open_gripper_node', anonymous=True)
# #     pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)
# #     print('in the functiopn')
# #     rospy.sleep(2) 
# #     gripper_msg = JointTrajectory()
# #     gripper_msg.header = Header()
# #     gripper_msg.joint_names = ['gripper_joint1', 'gripper_joint2']

# #     point = JointTrajectoryPoint()
# #     point.positions = [0, 0]
# #     point.velocities = [0, 0]
# #     point.accelerations = [0, 0]
# #     point.effort = [0, 0]
# #     point.time_from_start = rospy.Duration(10, 10)

# #     gripper_msg.points.append(point)
# #     pub.publish(gripper_msg)

# # if __name__ == '__main':
# #     try:
# #       rospy.init_node('your_node_name')
# #       open_gripper()
# #       print('commond is publish')
# #       rospy.spin()
# #     except rospy.ROSInterruptException:
# #       print('error')


import rospy
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class RosTopicPublisher:
    def __init__(self, arm_position, gripper_position):
        rospy.init_node('rostopic_publisher')
        self.pub1 = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10, latch=True)
        self.pub2 = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10, latch=True)
        self.rate = rospy.Rate(1)  # 1 Hz rate for publishing
        self.arm_position = arm_position
        self.gripper_position = gripper_position

    def create_trajectory_message(self, joint_names, positions, velocities, accelerations, effort, secs, nsecs):
        trajectory_msg = JointTrajectory()
        trajectory_msg.header.seq = 0
        trajectory_msg.header.stamp.secs = 0
        trajectory_msg.header.stamp.nsecs = 0
        trajectory_msg.header.frame_id = ''
        trajectory_msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = velocities
        point.accelerations = accelerations
        point.effort = effort
        point.time_from_start.secs = secs
        point.time_from_start.nsecs = nsecs

        trajectory_msg.points.append(point)

        return trajectory_msg

    def publish_messages(self):
        # Create and publish the first message
        joint_names_1 = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        positions_1 = [math.radians(deg) for deg in self.arm_position]
        velocities_1 = [0, 0, 0, 0, 0, 0]
        accelerations_1 = [0, 0, 0, 0, 0, 0]
        effort_1 = [0, 0, 0, 0, 0, 0]
        secs_1 = 20
        nsecs_1 = 20

        trajectory_msg_1 = self.create_trajectory_message(joint_names_1, positions_1, velocities_1,
                                                          accelerations_1, effort_1, secs_1, nsecs_1)

        self.pub1.publish(trajectory_msg_1)
        rospy.loginfo('Published the first message')

        # Wait for 10 seconds
        

        # Create and publish the second message
        joint_names_2 = ['gripper_joint1', 'gripper_joint2']
        positions_2 =  [math.radians(deg) for deg in self.gripper_position]
        velocities_2 = [0, 0]
        accelerations_2 = [0, 0]
        effort_2 = [0, 0]
        secs_2 = 20
        nsecs_2 = 20

        trajectory_msg_2 = self.create_trajectory_message(joint_names_2, positions_2, velocities_2,
                                                          accelerations_2, effort_2, secs_2, nsecs_2)

        self.pub2.publish(trajectory_msg_2)
        rospy.loginfo('Published the second message')

        # Spin ROS
        # rospy.spin()


# picking position
arm_position = [0,115,-101, 0, 82 ,0]


    # placing position
# arm_position1= [0, -29, -40 ,271, -15]
arm_position1= [90, -34, -105, 0, 120 , 0]

# gripper_position = [90, -90]
gripper_position= [0, 0]

#ungrip
gripper_position1= [29.7, 29.7]

publisher = RosTopicPublisher(arm_position, gripper_position) 
publisher.publish_messages()
rospy.spin()








# rostopic pub /arm_controller/command trajectory_msgs/JointTrajectory "header:
#   seq: 0
#   stamp:
#     secs: 0
#     nsecs: 0
#   frame_id: ''
# joint_names:
# - 'gripper_joint1'
# points:
# - positions: [-0.1]
#   velocities: [0]
#   accelerations: [0]
#   effort: [0]
#   time_from_start: {secs: 1, nsecs: 1}" 

#   rostopic pub /gripper_controller/command trajectory_msgs/JointTrajectory "header:
#   rostopic pub /gripper_controller/command trajectory_msgs/JointTrajectory "header:
#   seq: 0
#   stamp:
#     secs: 0
#     nsecs: 0
#   frame_id: ''
# joint_names:
# - 'gripper_joint1'
# points:
# - positions: [0.5]
#   velocities: [0]
#   accelerations: [0]
#   effort: [0]
#   time_from_start: {secs: 1, nsecs: 1}" 

# gripper open angle =[30, 30]
# [0.52, 0.52]

# pick arm angle  =[0,115,-101, 0, 82 0]
# [0.0, 2.0, -1.76, 0.0, 1.43, 0.0]

# # [0.0, 2.0, -1.88, 0.0, 1.27, 0.0]

# gripper close =[0, 0]

# place =[90, -34, -105, 0, 120 , 0]
# [1.57, -0.59, -1.83, 0.0, 2.0, 0.0]






#   rostopic pub /arm_controller/command trajectory_msgs/JointTrajectory "header:
#   seq: 0
#   stamp:
#     secs: 0
#     nsecs: 0
#   frame_id: ''
# joint_names:
# - 'joint1'
# - 'joint2'
# - 'joint3'
# - 'joint4'
# - 'joint5'
# - 'joint6'
# points:
# - positions: [0.0, 2.0, -1.76, 0.0, 1.43, 0.0]
#   velocities: [0, 0, 0, 0, 0, 0]
#   accelerations: [0, 0, 0, 0, 0, 0]
#   effort: [0, 0, 0, 0, 0, 0]
#   time_from_start: {secs: 10, nsecs: 10}"


#   rostopic pub /gripper_controller/command trajectory_msgs/JointTrajectory "header:
#   seq: 0
#   stamp:
#     secs: 0
#     nsecs: 0
#   frame_id: ''
# joint_names:
# - 'gripper_joint1'
# - 'gripper_joint2'
# points:

# - positions: [0.52, 0.52]
#   velocities: [0, 0]
#   accelerations: [0, 0]
#   effort: [0, 0]
#   time_from_start: {secs: 10, nsecs: 10}"