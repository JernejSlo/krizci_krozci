#!/usr/bin/env python3
from random import random

import rospy
import json
from std_srvs.srv import Trigger, TriggerResponse

from RobotVisualUtils import RobotVisualUtils

class VisualizeBoardService(RobotVisualUtils):

    def __init__(self):
        super.__init__()

        # Define service
        rospy.Service('/get_game_board', Trigger, self.get_board_status)
        rospy.loginfo("Game visualisation service started.")

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

    def move_above_board(self):

        # call another service that moves the arm above board

        pass


    def grab_board_image(self):

        self.capture_realsense_image("./sense/")
        return self.load_board_image()


    def get_board_status(self):

        try:
            self.move_above_board()
            image = self.grab_board_image()
            move = self.determine_board(image)
            return Trigger({'success': True, 'message': json.dumps(move)})
        except Exception as e:
            return Trigger({'success': False, "message": json.dumps({"error": e})})




    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        # this code is run at ctrl + c
        self.ctrl_c = True

if __name__ == '__main__':
    # initialise node
    #rospy.init_node('led_actuator')
    # initialise class
    visualize_service = VisualizeBoardService()
    try:
        # loop
        rospy.spin()
    except rospy.ROSInterruptException:
        pass