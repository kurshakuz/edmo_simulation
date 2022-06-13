#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import math
import json

from cpg_control import SERVOMIN, SERVOMAX, constrain, map_range, Oscillator, CPGController
from core.SimulatorServerConnector import SimulatorServerConnector

class CPGControllerConnected(CPGController):
    def __init__(self, node_name, pub_topic_1, pub_topic_2, pub_topic_3):
        super().__init__(node_name, pub_topic_1, pub_topic_2, pub_topic_3)
        self.connector = SimulatorServerConnector()
        self.connector.register_rcv_listener(self.update_from_json)
        self.connector.connect(host='localhost')
        self.received_json = False
        self.recording_state = False
        self.finished_recording = False
        self.last_received_time = 0.0
        self.sim_task_id = 0

    def update_from_json(self, msg):
        # TODO remove received_json
        print('Received JSON input from ', msg['name'])
        targetAmplitudes = []
        targetOffsets = []
        for i in range(len(msg['modules'])):
            targetAmplitudes.append(msg['modules'][i]['amplitude'])
            targetOffsets.append(msg['modules'][i]['offset'])
        self.update_controller(msg['frequency'], msg['weight'], targetAmplitudes, targetOffsets, msg['phase_bias_matrix'])
        self.last_received_time = rospy.get_time()
        print(self.last_received_time)

        self.received_json = True
        self.recording_state = True
        self.sim_task_id = msg['sim_task_id']

        self.x_positions = []
        self.y_positions = []

    def positionCallback(self, data):
        if self.recording_state and not self.finished_recording:
            self.x_positions.append(data.pose.pose.position.x)
            self.y_positions.append(data.pose.pose.position.y)
        if not self.recording_state and self.finished_recording:
            # self.connector.respond_result()
            print(self.x_positions)
            print(self.y_positions)
            self.finished_recording = False

    def publish_positions(self):
        # sample configuration
        # self.update_controller(freq = 0.37, weight = 0.025, targetAmplitudes = [31,18,34], targetOffsets=[34,0,-45], phaseBiases=[[0.0, 42.0, 0.0], [-42.0, 0.0, 34.0], [0.0, -34.0, 0.0]])

        # fastest configuration
        # self.update_controller(freq = 0.37, weight = 0.025, targetAmplitudes = [31,18,34], targetOffsets=[64,0,58], phaseBiases=[[0.0, 90.0, 0.0], [-90.0, 0.0, 34.0], [0.0, -34.0, 0.0]])

        # zero position
        # self.update_controller(freq = 0.25, weight = 0.025, targetAmplitudes = [0,0,0], targetOffsets=[0,0,0], phaseBiases=[[0.0, 1.0, 0.0], [1.0, 0.0, 1.0], [0.0, 1.0, 0.0]], convert=False)
        # self.received_json = True

        while not rospy.is_shutdown():
            if self.received_json:
                current_time = rospy.get_time()
                if current_time - (self.last_received_time + 10) >= 0:
                    print('10 secs passed')
                    self.received_json = False
                    self.recording_state = False
                    self.finished_recording = True

            rateOfFrequency = self.c * (self.targetFrequency - self.frequency)
            self.frequency = self.frequency + rateOfFrequency * (self.timeStep) / 1000.0

            # one oscillator for now
            for i in range(self.num_oscillators):
                self.osc[i].rateOfOffset = self.c * (self.osc[i].targetOffset - self.osc[i].offset)
                self.osc[i].offset = self.osc[i].offset + self.osc[i].rateOfOffset * (self.timeStep) / 1000.0

                # change amplitude for oscillator i
                self.osc[i].rateOfAmplitude = self.a * (self.osc[i].targetAmplitude - self.osc[i].amplitude)
                self.osc[i].amplitude = self.osc[i].amplitude + self.osc[i].rateOfAmplitude * (self.timeStep) / 1000.0

                # compute new rate of phase
                sum = 2 * math.pi * self.frequency
                for j in range(self.num_oscillators):
                    if (i != j):
                        # NOTE: here w is used as the coupling factor for all oscillators
                        # sum = sum + w * osc[i].amplitude * sin(osc[j].phase - osc[i].phase - osc[i].phaseBias[j])
                        # line below actually accounts for the topology of the network
                        if (self.osc[i].coupling[j] != 0):
                            sum = sum + self.w * self.osc[i].amplitude * math.sin(self.osc[j].phase - self.osc[i].phase - self.osc[i].phaseBias[j])

                self.osc[i].rateOfPhase = sum
                # compute new phase using Euler integration
                self.osc[i].phase = self.osc[i].phase + self.osc[i].rateOfPhase * (self.timeStep) / 1000.0

                # compute new angular position
                self.osc[i].pos = self.osc[i].amplitude * math.sin(self.osc[i].phase) + self.osc[i].offset

                # set motor to new position
                # print(self.osc[i].pos)
                self.osc[i].angle_motor = map_range(self.osc[i].pos, 0, 180, SERVOMIN[i], SERVOMAX[i])
                self.osc[i].angle_motor = constrain(self.osc[i].angle_motor, SERVOMIN[i], SERVOMAX[i])
                # print(self.osc[i].angle_motor)

                if self.received_json:
                    if i == 0:
                        self.pub_rev_13.publish(self.osc[i].angle_motor)
                    elif i == 1:
                        self.pub_rev_8.publish(self.osc[i].angle_motor)
                    else:
                        self.pub_rev_1.publish(self.osc[i].angle_motor)

            self.rate.sleep()

if __name__ == '__main__':
    node_name = 'motor_command_pub'
    pub_topic_1 = 'edmo_snake_controller/Rev1_position_controller/command'
    pub_topic_2 = 'edmo_snake_controller/Rev8_position_controller/command'
    pub_topic_3 = 'edmo_snake_controller/Rev13_position_controller/command'
    controller = CPGControllerConnected(node_name, pub_topic_1, pub_topic_2, pub_topic_3)
    rospy.Subscriber('edmo_snake/state', Odometry, controller.positionCallback)
    try:
        controller.publish_positions()
    except rospy.ROSInterruptException:
        # controller.connector
        pass