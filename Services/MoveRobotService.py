#! /usr/bin/python3

from colorsys import rgb_to_yiq
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from math import pi
import math
# from moveit_commander.conversions import pose_to_list
import tf
from numpy import linalg as LA
import time

import json
from std_msgs.msg import String

class MoveRobotService():

    def __init__(self):


        rospy.Service('/move_robot_to', String, self.move_arm_to)

        rospy.loginfo("Game visualisation service started.")

        moveit_commander.roscpp_initialize(sys.argv)

        # robots kinematic model and the robots current joint states
        self.robot = moveit_commander.RobotCommander()

        # remote interface for getting, setting, and updating the robots internal understanding of the surrounding world
        self.scene = moveit_commander.PlanningSceneInterface()

        # interface to a planning group (group of joints).
        self.group_name = "gluon"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name, robot_description="/robot_description")
        self.move_group.set_pose_reference_frame('base_link_inertia')
        self.move_group.set_end_effector_link('tool0')

        self.move_group.clear_pose_targets()

        # self.board_position = [self.to_rad(-35), self.to_rad(-20), -self.to_rad(65), self.to_rad(45), -self.to_rad(90), -self.to_rad(45)]
        self.board_position = [self.to_rad(-22), self.to_rad(3), self.to_rad(-106), self.to_rad(-19), self.to_rad(-88),
                               self.to_rad(-35)]

        self.ctrl_c = False
        self.rate = rospy.Rate(10)  # 10hz
        rospy.on_shutdown(self.shutdownhook)

    def move_to_position(self,position):
        match position:
            case "home":
                self.go_home()
            case "board":
                self.move_above_board()
            case isinstance(position, dict):
                self.move_to_board_square(position)
            case _:
                raise ValueError("Invalid position")



    def move_arm_to(self,req):

        try:
            data = json.loads(req.data)  # Convert input string to a 2D list
        except json.JSONDecodeError:
            return String(json.dumps({"success": False, "error": "Invalid JSON"}))

        if "position" in data:
            rospy.loginfo("Position key exists!")
            position = data["position"]  # Safely access it
            try:
                self.move_to_position(position)
            except Exception as e:
                return String(json.dumps({"success": False,"error": e}))

            return String(json.dumps({"success": True}))
        else:
            required_keys = ["x", "y", "z"]

            # Check if all required keys exist
            missing_keys = [key for key in required_keys if key not in data]

            if missing_keys:
                return String(json.dumps({"success": False, "error": f"Missing keys: {', '.join(missing_keys)}"}))
            else:

                return String(json.dumps({"success": True}))

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

    def to_rad(self, degrees):
        return degrees * pi / 180

    def get_basic_info(self):

        # We can get the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")

    def axisangle_to_quaternion(self, axan):

        angle = LA.norm(axan)
        ax = axan / angle

        q = Quaternion()
        q.x = ax[0] * math.sin(angle / 2)
        q.y = ax[1] * math.sin(angle / 2)
        q.z = ax[2] * math.sin(angle / 2)
        q.w = math.cos(angle / 2)

        return q

    def moveJ(self, goal, velocity, acceleration):

        if velocity > 1:
            velocity = 1
        elif velocity < 0:
            velocity = 0.1

        if acceleration > 1:
            acceleration = 1
        elif acceleration < 0:
            acceleration = 0.1

        self.move_group.set_planner_id('PTP')
        self.move_group.set_max_velocity_scaling_factor(velocity)
        self.move_group.set_max_acceleration_scaling_factor(acceleration)
        # self.move_group.set_pose_reference_frame('base_link_inertia')

        # plan and execute
        self.move_group.go(goal, wait=True)

    def moveL(self, goal):

        self.move_group.set_planner_id('LIN')
        self.move_group.set_max_velocity_scaling_factor(0.5)
        self.move_group.set_max_acceleration_scaling_factor(0.2)
        # self.move_group.set_pose_reference_frame('base_link_inertia')

        self.move_group.clear_pose_targets()

        # plan and execute
        self.move_group.go(goal, wait=True)

    def go_home(self):

        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = 0

        # move to home position
        self.moveJ(joint_goal, 0.5, 0.2)

    def test_move(self):

        print("Go home.")
        self.go_home()

        # define joint goal
        print("Do moveJ.")
        qq_r = self.move_group.get_current_joint_values()
        qq_r[0] += 30 * pi / 180
        self.moveJ(qq_r, 0.5, 0.2)

        """
        # define cartesian goal - relative mode
        print("Do relative moveL.")
        x_r = self.move_group.get_current_pose()
        x_r.pose.position.z += 0.1
        self.moveL(x_r)

        # define cartesian goal - absolute move
        print("Do absolute moveL.")
        point_1 = PoseStamped()
        point_1.header.seq = 0
        point_1.header.stamp = rospy.Time.now()
        point_1.header.frame_id = "base_link_inertia"

        point_1.pose.position.x = 0.5
        point_1.pose.position.y = -0.2
        point_1.pose.position.z = 0.5
        # izracun z uporabo RPY kotov
        ax = 150 * pi/180
        ay = 20 * pi/180
        az = 60 * pi/180
        quat = tf.transformations.quaternion_from_euler(ax, ay, az, 'sxyz')      

        # izracun z uporabo axis angle
        # ax = [125*pi/180,80*pi/180,5*pi/180]
        # quat = self.axisangle_to_quaternion(ax)

        point_1.pose.orientation.x = quat[0]
        point_1.pose.orientation.y = quat[1]
        point_1.pose.orientation.z = quat[2]
        point_1.pose.orientation.w = quat[3]

        self.moveL(point_1)"""

    def move_above_board(self):

        print("Go home.")
        self.go_home()

        print("Moving above board.")
        qq_r = self.move_group.get_current_joint_values()
        qq_r = [x + y for x, y in zip(qq_r, self.board_position)]
        print(qq_r)
        self.moveJ(qq_r, 0.5, 0.2)

        time.sleep(10)

        print("Go home.")
        self.go_home()


if __name__ == '__main__':
    #rospy.init_node('moveit_gluon_test', anonymous=True)

    move_service = MoveRobotService()
    try:
        #move_service.get_basic_info()
        #move_service.move_above_board()
        pass
    except rospy.ROSInterruptException:
        pass