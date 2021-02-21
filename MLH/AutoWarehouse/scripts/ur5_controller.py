#!/usr/bin/env python

'''
Script to controll the ur5 robot
'''

from __future__ import print_function

import sys
import math
import yaml

import rospy
import actionlib
import moveit_commander
import moveit_msgs.msg
import rospkg

from geometry_msgs.msg import PoseStamped

from pkg_vb_sim.srv import vacuumGripper


class UR5(object):
    '''
    Class to control the ur5 robot

    Attributes
    ----------
    robot_name : str
        Name of the ur5 robot
    '''

    # pylint: disable=too-many-instance-attributes, assignment-from-no-return

    def __init__(self, robot_name):

        # Moveit Configuration

        self._robot_ns = '/' + robot_name
        self._planning_group = "manipulator"

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(
            robot_description=self._robot_ns + "/robot_description",
            ns=self._robot_ns
        )
        self._scene = moveit_commander.PlanningSceneInterface(
            ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(
            self._planning_group,
            robot_description=self._robot_ns + "/robot_description",
            ns=self._robot_ns
        )
        self._display_trajectory_publisher = rospy.Publisher(
            self._robot_ns + '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=1
        )
        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            self._robot_ns + '/execute_trajectory',
            moveit_msgs.msg.ExecuteTrajectoryAction
        )
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        _rp = rospkg.RosPack()
        self._pkg_path = _rp.get_path('pkg_task5')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        # self._file_path = self._pkg_path + '/config/tj/'
        self._computed_plan = ''

    # ------------------------------------------------
    # | Methods for controlling the movement of ur5  |
    # ------------------------------------------------

    def set_joint_angles(self, list_joint_angles, convert_to_radian=False):
        """
        Sets the 6 joint angles of the ur5 arm.

        Parameters
        ----------
        list_joint_angles : list
            List of 6 joint angles for the ur5 arm
        convert_to_radian : bool, optional
            Converts the given list of joint angles into radians (default is False)

        Returns
        -------
        bool
            True if joint angles are successfully set else False
        """

        if convert_to_radian:
            list_joint_angles = list(map(math.radians, list_joint_angles))

        self._group.set_joint_value_target(list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=False)
        rospy.sleep(3.0)

        if flag_plan:
            rospy.loginfo("[Success] Joint Angles Set")
        else:
            rospy.logerr("[Error] Failed Setting Joint Angles")

        return flag_plan

    def hard_set_joint_angles(self, list_joint_angles, convert_to_radian=False, max_attempts=5):
        """
        Attempts to set joint angles of the ur5 multiple time incase it fails.

        Parameters
        ----------
        list_joint_angles : list
            List of 6 joint angles for the ur5 arm
        convert_to_radian : bool, optional
            Converts the given list of joint angles into radians (default is False)
        max_attempts : int
            Number of times to attempt setting the joint angles

        Returns
        -------
        bool
            True if joint angles are successfully set else False
        """

        number_attempts = 0
        flag_success = False

        while number_attempts <= max_attempts and not flag_success:
            number_attempts += 1
            flag_success = self.set_joint_angles(
                list_joint_angles, convert_to_radian)

        return flag_success

    def moveit_play_planned_path_from_file(self, file_path, file_name):
        """
        Plays a saved trajectory.

        Parameters
        ----------
        file_path : str
            Path of the saved trajectory
        file_nmae : str
            Name of the saved trajectory file

        Returns
        -------
        bool
            True if trajectory is successfully played else False
        """

        trajectory_file = file_path + file_name

        with open(trajectory_file, 'r') as saved_path:
            loaded_plan = yaml.load(saved_path, Loader=yaml.Loader)

        ret = self._group.execute(loaded_plan)
        return ret

    def moveit_hard_play_planned_path_from_file(self, file_path, file_name, max_attempts):
        """
        Attempts to play a saved trajectory multiple time incase it fails.

        Parameters
        ----------
        file_path : str
            Path of the saved trajectory
        file_nmae : str
            Name of the saved trajectory file
        max_attempts : int
            Number of times to attempt playing the saved trajectory

        Returns
        -------
        bool
            True if trajectory is successfully played else False
        """

        number_attempts = 0
        flag_success = False

        while number_attempts <= max_attempts and not flag_success:

            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(
                file_path, file_name)

        return flag_success

    def go_to_pose(self, pose):
        """
        Moves the end effector of the ur5 arm to the given
        pose.

        Parameters
        ----------
        pose : geometry_msgs.msg.Pose
            Destination pose

        Returns
        -------
        bool
            True if pose is successfully set else False
        """

        self._group.set_pose_target(pose)
        flag_plan = self._group.go(wait=True)

        if flag_plan:
            rospy.loginfo("[Success] go_to_pose() ")
        else:
            rospy.logerr("[Error] Solution for Pose not Found ")

        return flag_plan

    def force_go_to_pose(self, pose, max_attempts=5):
        """
        Executes go_to_pose() multiple times untill pose
        is successfully set.

        Parameters
        ----------
        pose : geometry_msgs.msg.Pose
            Destination pose
        max_attempts : int, optional
            Number of times go_to_pose() will execute

        Returns
        -------
        bool
            True if pose is successfully set else False
        """

        number_attempts = 0
        flag_success = False

        while number_attempts <= max_attempts and not flag_success:

            number_attempts += 1
            flag_success = self.go_to_pose(pose)

        return flag_success

    # --------------------------------------------
    # | Methods for controlling the end effector |
    # --------------------------------------------

    def add_box_to_scene(self, obj_name, orientation, position, size, frame_id='world'):
        """
        Adds an object to the planning scene.

        Parameters
        ----------
        obj_name : str
            Name of the object to be used in the planning scene.
        orientation : float
            End effector group name
        position : list
            x,y,z coordinates of the object
        size : tuple
            Length, breadth and height of the object
        frame_id : str, optional
            Refrence frame for coordinates (default is 'world')
        """

        # pylint: disable=too-many-arguments

        obj_pose = PoseStamped()
        obj_pose.header.frame_id = frame_id
        obj_pose.pose.orientation.w = orientation
        obj_pose.pose.position.x, obj_pose.pose.position.y, obj_pose.pose.position.z = position

        self._scene.add_box(obj_name, obj_pose, size)

    def attach_obj_to_ee(self, obj_name, grasping_group='manipulator'):
        """
        Attaches an object to the end effector.

        Parameters
        ----------
        obj_name : str
            Name of the object in the planning scene
        grasping_group : str, optional
            End effector group name (default is 'manipulator')
        """

        # Attach the object to the EE
        touch_links = self._robot.get_link_names(group=grasping_group)
        self._scene.attach_box(
            self._eef_link, obj_name,
            touch_links=touch_links
        )
        # Start the vacuum gripper to attach the object in gazebo
        self.invoke_vacuum_gripper()

    def detach_obj_from_ee(self, obj_name):
        """
        Detaches an object from the end effector.

        Parameters
        ----------
        obj_name : str
            Name of the object in the planning scene
        """

        # Detach the object from EE
        self._scene.remove_attached_object(self._eef_link, name=obj_name)
        # Stop the vacuum gripper to drop the object in gazebo
        self.invoke_vacuum_gripper(state=False)

    # ----------------------------------------
    # | Methods to access different services |
    # ----------------------------------------

    def invoke_vacuum_gripper(self, state=True):
        """
        Invokes the vacuum gripper on the end effector of the ur5.

        Parameters
        ----------
        state : bool, optional
            Turns the vacuum gripper ON or OFF (default is True -> ON)

        Raises
        ------
        rospy.ServiceException
            If communication with remote service fails.
        """

        rospy.wait_for_service(
            '/eyrc/vb/ur5/activate_vacuum_gripper' + self._robot_ns)
        try:
            vacuum_gripper = rospy.ServiceProxy(
                '/eyrc/vb/ur5/activate_vacuum_gripper' + self._robot_ns, vacuumGripper)
        except rospy.ServiceException as exc:
            print(exc)
        vacuum_gripper(state)
