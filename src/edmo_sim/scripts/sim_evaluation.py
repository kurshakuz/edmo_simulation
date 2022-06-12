#!/usr/bin/python3

import sys
import rospy

from cpg_control_multithread import SERVOMIN, SERVOMAX, constrain, map_range, Oscillator, CPGController
from interfaces.motors import PublisherMotors
from interfaces.clock import ListenerClock


# Execute the application
if __name__ == "__main__":
    rospy.init_node("EDMOEvaluationServer")
    node_name = 'motor_command_pub'
    topic_1 = 'edmo_snake_controller/Rev1_position_controller/command'
    topic_2 = 'edmo_snake_controller/Rev8_position_controller/command'
    topic_3 = 'edmo_snake_controller/Rev13_position_controller/command'

    robot_1 = 'edmo1'
    robot_2 = 'edmo2'

    robot_1_node_name = robot_1 + '_' + node_name
    robot_1_topic_1 = robot_1 + '/' + topic_1
    robot_1_topic_2 = robot_1 + '/' + topic_2
    robot_1_topic_3 = robot_1 + '/' + topic_3

    robot_2_node_name = robot_2 + '_' + node_name
    robot_2_topic_1 = robot_2 + '/' + topic_1
    robot_2_topic_2 = robot_2 + '/' + topic_2
    robot_2_topic_3 = robot_2 + '/' + topic_3

    # if node_name != None:
    #     rospy.init_node(node_name, anonymous=True)
    # self.rate = rospy.Rate(40)
    # self.pub_rev_1 = rospy.Publisher(pub_topic_1, Float64, queue_size=1)
    # self.pub_rev_8 = rospy.Publisher(pub_topic_2, Float64, queue_size=1)
    # self.pub_rev_13 = rospy.Publisher(pub_topic_3, Float64, queue_size=1)

    # controller_1 = CPGController(robot_1_topic_1, robot_1_topic_2, robot_1_topic_3, None)
    # controller_2 = CPGController(robot_2_topic_1, robot_2_topic_2, robot_2_topic_3, None)

    clock = [ListenerClock("edmo1_clock"),
             ListenerClock("edmo2_clock")]
                
    motors_1 = [PublisherMotors(robot_1_topic_1, clock[0]),
                PublisherMotors(robot_1_topic_2, clock[0]),
                PublisherMotors(robot_1_topic_3, clock[0])]

    controller_1 = CPGController(motors_1)

    controller_1.update_controller(freq = 0.37, weight = 0.025, targetAmplitudes = [31,18,34], targetOffsets=[64,0,58], phaseBiases=[[0.0, 90.0, 0.0], [-90.0, 0.0, 34.0], [0.0, -34.0, 0.0]])
    # controller_2.update_controller(freq = 0.37, weight = 0.025, targetAmplitudes = [31,18,34], targetOffsets=[34,0,-45], phaseBiases=[[0.0, 42.0, 0.0], [-42.0, 0.0, 34.0], [0.0, -34.0, 0.0]])

    try:
        controller_1.publish_positions()
        # controller_2.publish_positions()
    except rospy.ROSInterruptException:
        pass
