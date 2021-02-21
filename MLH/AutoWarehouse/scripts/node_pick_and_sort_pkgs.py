#!/usr/bin/env python
'''
File to opearte ur5_1
'''

from __future__ import print_function

import threading
import actionlib

import rospy
from geometry_msgs.msg import Pose

from pkg_vb_sim.srv import conveyorBeltPowerMsg

from pkg_task4.msg import msgLogicalCamAction
from ur5_controller import UR5


class Ur5PickAndSort(object):
    """
    Uses feed from logicalCamera2 to identify packages and
    sort them into their respective bins based on their color.
    """

    def __init__(self, robot_name):

        # ActionServer
        self._as = actionlib.ActionServer(
            '/logicalCam2_stream',
            msgLogicalCamAction,
            self.on_goal_from_client,
            self.cancel_current_goal,
            auto_start=False
        )

        # ur5_2 Controller
        self.ur5_2 = UR5(robot_name)

        # Sorting Bin Poses
        # pylint : disable=invalid-name
        self.BIN_POSES = {
            'red': [89, -57, 68, -101, -90, 180],
            'yellow': [14, -78, 98, -110, -90, 104],
            'green': [-111, -97, 117, -110, -90, 21],
        }

        # Current package under operation
        self._current_pkg = None
        # UR5 home pose
        self.ur5_2_home_pose = Pose()
        self.ur5_2_home_pose.position.x = -0.8
        self.ur5_2_home_pose.position.y = -0.05
        self.ur5_2_home_pose.position.z = 1.19
        self.ur5_2_home_pose.orientation.x = -0.5
        self.ur5_2_home_pose.orientation.y = -0.5
        self.ur5_2_home_pose.orientation.z = 0.5
        self.ur5_2_home_pose.orientation.w = 0.5

    def run(self):
        """
        Starts the ActionServer and moves the UR5_2 robot arm
        to home pose, so that the packages are picked as quickly
        as possible.
        """

        # Make the ur5 go below the logical camera
        self.ur5_2.force_go_to_pose(self.ur5_2_home_pose)

        # Start the Action Server
        self._as.start()
        rospy.loginfo("Started Ur5 ActionServer")

    def on_goal_from_client(self, goal_handle):
        """
        Callback method for ActionServer and goal validation. The
        ActionServer receives feed from logicalCamera2 and it continously
        populates this method until the package is in the range of logical
        Camera, but for sorting only one  goal_handle is required. So the
        first goal_handle is used for processing the goal and rest of the
        goals containing the same package information are rejected.

        Parameters
        ----------
        goal_handle :
        """

        goal = goal_handle.get_goal()
        # if we are currently working with a box, all the incoming goals
        # will get rejected which contain the same box information
        if goal.type == self._current_pkg:
            goal_handle.set_rejected()
            return
        else:
            self._current_pkg = goal.type
            goal_handle.set_accepted()

        # Start picking and sorting the box on a seperate thread
        processor_thread = threading.Thread(
            name='worker_goalProcessor',
            target=self.process_goal,
            args=[goal_handle]
        )

        processor_thread.start()
        # wait for the thread to complete
        processor_thread.join()

    def process_goal(self, goal_handle):
        """
        Method to process the incoming goal i.e pick the package from
        the conveyor belt and sort it into the correct bin corresponding
        to the color of the package.

        Parameters
        ----------
        goal_handle :
        """

        self.power_conveyor_belt(0)
        goal = goal_handle.get_goal()

        print(
            " -> Received Command to Sort\n    {} | {} Bin".format(goal.type, goal.color))

        # Add the Package to scene
        self.ur5_2.add_box_to_scene(
            goal.type,
            1.0,
            [
                goal.x, goal.y, goal.z
            ],
            (0.15, 0.15, 0.15),
        )

        print(" -> Attempting to pick {}".format(goal.type))
        self.ur5_2.attach_obj_to_ee(goal.type)

        self.ur5_2.hard_set_joint_angles(
            self.BIN_POSES[goal.color],
            True,
            50
        )

        threading.Thread(
            name='worker_conveyorOperator',
            target=self.power_conveyor_belt,
            args=[100]
        ).start()

        self.ur5_2.detach_obj_from_ee(goal.type)

        self.ur5_2.force_go_to_pose(self.ur5_2_home_pose, 50)

        print(" -> Package Placed Into Bin")

    @ staticmethod
    def power_conveyor_belt(power=40):
        """
        Controlls the conveyor belt by supplying wrench force to it.

        Parameters
        ----------
        power : int, optional
            The amount of power to be supplied. 0 is minimum
            and 100 is maximum (default is 40)

        Raises
        ------
        rospy.ServiceException
            If communication with remote service fails.
        """

        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        try:
            conveyor_belt = rospy.ServiceProxy(
                '/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
        except rospy.ServiceException as exc:
            print(exc)
        conveyor_belt(power)

    @staticmethod
    def cancel_current_goal(goal_handle):
        '''
        Utility method to deal with cancelled goals from ActionServer

        Parameters
        ----------
        goal_handle :
            Goal to be rejected
        '''
        goal_handle.set_rejected()


def main():
    '''
    Main method to run the node
    '''

    rospy.init_node('node_pick_and_sort')
    ur5_2 = Ur5PickAndSort('ur5_2')
    ur5_2.run()
    rospy.spin()


if __name__ == "__main__":
    main()
