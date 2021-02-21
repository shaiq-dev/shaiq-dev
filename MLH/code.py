#!/usr/bin/env python
'''
File to operate ur5_1
'''

import sys
import os
from os.path import dirname, abspath
import math
import threading
import time
import yaml
from collections import OrderedDict
from datetime import datetime as dt

import cv2
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode, ZBarSymbol


import rospy
import rospkg
import actionlib
from sensor_msgs.msg import Image

from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.msg import LogicalCameraImage

from pkg_ros_iot_bridge.msg import (
    msgRosIotAction,
    msgRosIotGoal
)

from pkg_task5.msg import (
    msgLogicalCamAction,
    msgLogicalCamGoal
)

from pkg_ros_iot_bridge.msg import msgMqttSub
from ur5_controller import UR5
from data import D


# Helper Classses

# https://stackoverflow.com/questions/39358092/range-as-dictionary-key-in-python
# It is used to avoid long messy if - else blocks
# This dict uses range as key
class RangeDict(dict):
    def __getitem__(self, item):
        if not isinstance(item, xrange):
            for key in self:
                if item in key:
                    return self[key]
            raise KeyError(item)
        else:
            return super(RangeDict, self).__getitem__(item)


class ProcessQr(object):
    '''
    ImageProcessing Class - Processes all the qr codes from the 2D Camera
    '''

    def __init__(self):
        self.bridge = CvBridge()

    @staticmethod
    def _get_package_name(h, v):
        '''
        Helper method to generate a package name
        '''
        hCoordinate = RangeDict({
            xrange(125, 135): '0',
            xrange(310, 320): '1',
            xrange(495, 505): '2'
        })

        vCoordinate = RangeDict({
            xrange(790, 800): '3',
            xrange(640, 650): '2',
            xrange(490, 500): '1',
            xrange(310, 320): '0'
        })

        pkg = "packagen{}{}".format(
            vCoordinate[v],
            hCoordinate[h]
        )

        return pkg

    def get_qr_data(self, image):
        '''
        Method to process the 2D camera image qr codes
        '''
        colors = {}

        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)

        qr_codes = decode(cv_image, symbols=[ZBarSymbol.QRCODE])

        for qr in qr_codes:
            (x, y, _, _) = qr.rect
            # Using x and y of the qr code, we generated the
            # package name
            pkg = self._get_package_name(x, y)
            colors[pkg] = qr.data

        return colors


# Main Controller Class
class Ur5DetectAndPlace(object):
    '''
    Class to identify the package colors and place them on the conveyor belt
    '''

    def __init__(self, robot_name):

        # Ur5 Controllers
        self.ur5_1 = UR5("ur5_1")
        self.ur5_1._group.set_planning_time(99)

        # Action Clients
        # Logical Camera 2
        self._ac = actionlib.ActionClient(
            '/logicalCam2_stream',
            msgLogicalCamAction
        )
        # ROS IOT Bridge
        self._ac_iot = actionlib.ActionClient(
            '/action_ros_iot',
            msgRosIotAction
        )
        self._ac_iot.wait_for_server()

        self.qr_processor = ProcessQr()

        # Saved Trajectories Paths
        self._pkg_path = self.ur5_1._pkg_path
        self._pick_path = os.path.join(
            self._pkg_path, 'config/saved_trajectories/pick/')
        self._place_path = os.path.join(
            self._pkg_path, 'config/saved_trajectories/place/')

        # Dict, which will be updated with package colors
        # detected from 2D camera
        self.PKG_COLORS = None

        # Pose of 12 packages
        self.PKG_POSE = {
            'packagen00': [0.28, -0.41, 1.91],
            'packagen01': [0.0, -0.41, 1.91],
            'packagen02': [-0.28, -0.41, 1.91],
            'packagen10': [0.28, -0.41, 1.64],
            'packagen11': [0.0, -0.41, 1.64],
            'packagen12': [-0.28, -0.41, 1.64],
            'packagen20': [0.28, -0.41, 1.42],
            'packagen21': [0.0, -0.41, 1.42],
            'packagen22': [-0.28, -0.41, 1.42],
            'packagen30': [0.28, -0.41, 1.19],
            'packagen31': [0.0, -0.41, 1.19],
            'packagen32': [-0.28, -0.41, 1.19],
        }

        # Different Joint States for ur5
        self.UR5_JOINTS = {
            'ABOVE_BELT_UR1': [9, -139, -57, -74, 91, -75]
        }

        self.in_execution = False
        self.orders = {}

    def run(self):

        # Instead of using rospy.Subscriber which continously sends images from 2D camera,
        # we used rospy.wait_for_message, which will receive a single message.
        img = rospy.wait_for_message("/eyrc/vb/camera_1/image_raw", Image)
        # Use this image to get color of packages
        colors = self.qr_processor.get_qr_data(img)
        # Store package name and colors in an OrderedDict so order is maintained
        self.PKG_COLORS = OrderedDict(
            sorted(colors.items(), key=lambda x: x[0]))

        # DEBUG:
        print("Package Identification Results: ")
        for p, c in self.PKG_COLORS.iteritems():
            print("{} | {}".format(p, c))
        rospy.loginfo("QR Decoding and Package Identification Done")

        # Update Inventory Sheet of warehouse Management Spreadsheet
        for p, c in self.PKG_COLORS.iteritems():
            data = D().create_inventory_data(p, c)
            # Update the Inventory Sheet
            self.send_goal_to_ros_iot_bridge(data)

        # Subscribe to '/ros_iot_bridge/mqtt/sub' to get incoming orders
        sub_topic = '/ros_iot_bridge/mqtt/sub'
        rospy.Subscriber(sub_topic, msgMqttSub, self.order_listener_callback)

        # Start Logical Camera 2 Subscriber
        threading.Thread(
            name="worker_logicCam2Sub",
            target=self.start_logical_camera2_subscriber
        ).start()

        # Add pickable objects to Rviz
        self.add_packages_to_rviz()

        # Start Conveyor Belt
        self.power_conveyor_belt(100)

        # Go to Home Pose inorder to play saved trajectories
        self.ur5_1.hard_set_joint_angles(
            self.UR5_JOINTS['ABOVE_BELT_UR1'], True)

        # Get all saved trajectories
        saved_trajectories_available = sorted(os.listdir(self._pick_path))

        current_order = None
        package = None
        STORED_PKGS = OrderedDict(
            sorted(self.PKG_COLORS.copy().items(), key=lambda x: x[0]))

        while True:
            if not self.orders:
                # Keep the loop running until an order comes
                continue
            else:

                # DEBUG
                print("Current Orders")
                print(self.orders)

                # Now pick the package with highest priority
                if len(self.orders) == 1:
                    current_order = next(iter(self.orders.values()))
                    del self.orders[next(iter(self.orders.keys()))]
                else:
                    for k, v in self.orders.items():
                        if v['item'] == D.OItem.HP:
                            current_order = v
                            del self.orders[k]
                            break
                    else:
                        for k, v in self.orders.items():
                            if v['item'] == D.OItem.MP:
                                current_order = v
                                del self.orders[k]
                                break
                        else:
                            current_order = next(iter(self.orders.values()))
                            del self.orders[next(iter(self.orders.keys()))]

                print("Order Selected")
                print(current_order)

                # Get a package with corresponding priority
                for p, c in STORED_PKGS.items():
                    if c == D.PColor[current_order['item']]:
                        # Check if trajectory exists
                        if "drop_to_{}.yaml".format(p[-2:]) in saved_trajectories_available:
                            package = p
                            del STORED_PKGS[p]
                            break
                else:
                    print(
                        "[WARN] Either package is not available or trajectory doesn't exist ")
                    continue

                # Start Picking the package
                _code = package[-2:]
                self.ur5_1.moveit_hard_play_planned_path_from_file(
                    self._pick_path,
                    "drop_to_{}.yaml".format(_code),
                    3
                )

                # Attach to EE
                self.ur5_1.attach_obj_to_ee(package)

                print(" -> '{}' Attached to ur5".format(package))

                # Go Above the Conveyor Belt
                self.ur5_1.moveit_hard_play_planned_path_from_file(
                    self._place_path,
                    "{}_to_drop.yaml".format(_code),
                    3
                )

                # Detach the Package
                self.ur5_1.detach_obj_from_ee(package)

                print(" -> '{}' Placed on Conveyor Belt".format(package))

    def send_goal_to_ros_iot_bridge(self, params):

        goal = msgRosIotGoal()
        goal.message = str(params)
        self._ac_iot.send_goal(goal)

    def order_listener_callback(self, msg):
        order = eval(str(msg.mqttMsg))
        self.orders[order['order_id']] = order

    def add_packages_to_rviz(self):
        '''
        Utility method to add pickable objects to RVIZ
        '''
        for k, v in self.PKG_POSE.items():
            self.ur5_1.add_box_to_scene(k, 1.0, v, (0.15, 0.15, 0.15))

    @ staticmethod
    def power_conveyor_belt(power=40):
        '''
        Method to power the conveyor belt
        @param power : Min = 0, Max = 100
        '''

        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        conveyor_belt = rospy.ServiceProxy(
            '/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
        conveyor_belt(power)

    # -----------------------------
    # | Logical Camera 2 Related |
    # ----------------------------

    def start_logical_camera2_subscriber(self):
        '''
        Utility method to start logical camera 2 subscriber
        '''

        logical_camera_topic = "/eyrc/vb/logical_camera_2"
        rospy.Subscriber(
            logical_camera_topic,
            LogicalCameraImage,
            self.logical_camera2_callback,
            queue_size=1
        )
        self._ac.wait_for_server()
        rospy.loginfo("Logical Camera 2 Listener Started")

    def logical_camera2_callback(self, msg):
        '''
        Logical Camera 2 Subscriber callback method
        '''

        # Packages which we don't need for evalvation
        _whitelisted = ["ur5"]

        # If any model has come in the range of logical camera,
        # get the latest model.
        # [-1] returns the last appended model in the models list
        box_model = msg.models[-1] if msg.models else None

        if box_model and box_model.type not in _whitelisted:
            # Just to stop the box near the ur5 arm, we
            # used y <= 0
            if box_model.pose.position.y <= 0:

                goal = msgLogicalCamGoal()

                # Get the Package Color we found from the 2D Camera Image
                goal.color = self.PKG_COLORS[box_model.type]
                goal.type = box_model.type
                goal.x, goal.y, goal.z = self.get_pkg_pickup_pose(box_model)

                self._ac.send_goal(goal)

    @staticmethod
    def get_pkg_pickup_pose(box_model):
        '''
        Utility method to create a msg for ur5_2 with the
        x, y, z coordinates of the package in world frame
        '''

        X_OFF = -0.8
        Y_OFF = -0.06
        Z_OFF = 0.99

        # NOTE:
        # Insted of translating the coordinates for each package
        # we mae the packages stop at a paticular coordinate
        # and pick it from there

        return X_OFF, Y_OFF, Z_OFF


def main(args):
    '''
    Main method to run the node
    '''

    rospy.init_node('node_detect_and_place_pkgs', anonymous=True)

    ur5 = Ur5DetectAndPlace('ur5_1')
    ur5.run()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
