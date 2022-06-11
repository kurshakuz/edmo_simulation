#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import math
import json

from cpg_control import SERVOMIN, SERVOMAX, constrain, map_range, Oscillator, CPGController
from core.TCPClient import TCPClient

class SimulatorServerConnector():
    def __init__(self):
        self.tcp_client = TCPClient()

    # 192.168.114.222
    def connect(self, host="localhost", port=65343):
        connection_successful = self.tcp_client.connect(host, port)

        if not connection_successful:
            print("Connection to server could not be established")
            return

    def register_rcv_listener(self, fnct):
        self.tcp_client.add_received_message_listener(lambda host, port, msg: fnct(json.loads(msg)))

class CPGControllerConnected(CPGController):
    def __init__(self):
        super().__init__()
        self.connector = SimulatorServerConnector()
        self.connector.register_rcv_listener(self.update_from_json)
        self.connector.connect()
        self.received_json = False

    def update_from_json(self, msg):
        self.received_json = True
        print(msg['name'])
        targetAmplitudes = []
        targetOffsets = []
        for i in range(len(msg['modules'])):
            targetAmplitudes.append(msg['modules'][i]['amplitude'])
            targetOffsets.append(90+msg['modules'][i]['offset'])
        self.update_controller(msg['frequency'], msg['weight'], targetAmplitudes, targetOffsets, msg['phase_bias_matrix'])

    def publish_positions(self):
        rospy.init_node('motor_command_pub', anonymous=True)
        rate = rospy.Rate(40) # 10hz

        pub_rev_1 = rospy.Publisher('/edmo_snake_controller/Rev1_position_controller/command', Float64, queue_size=1)
        pub_rev_8 = rospy.Publisher('/edmo_snake_controller/Rev8_position_controller/command', Float64, queue_size=1)
        pub_rev_13 = rospy.Publisher('/edmo_snake_controller/Rev13_position_controller/command', Float64, queue_size=1)

        # sample configuration
        # self.update_controller(freq = 0.37, weight = 0.025, targetAmplitudes = [31,18,34], targetOffsets=[34,0,-45], phaseBiases=[[0.0, 42.0, 0.0], [-42.0, 0.0, 34.0], [0.0, -34.0, 0.0]])

        # fastest configuration
        self.update_controller(freq = 0.37, weight = 0.025, targetAmplitudes = [31,18,34], targetOffsets=[64,0,58], phaseBiases=[[0.0, 90.0, 0.0], [-90.0, 0.0, 34.0], [0.0, -34.0, 0.0]])

        # zero position
        # self.update_controller(freq = 0.25, weight = 0.025, targetAmplitudes = [0,0,0], targetOffsets=[0,0,0], phaseBiases=[[0.0, 1.0, 0.0], [1.0, 0.0, 1.0], [0.0, 1.0, 0.0]], convert=False)

        self.received_json = True

        while not rospy.is_shutdown():
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
                        pub_rev_13.publish(self.osc[i].angle_motor)
                    elif i == 1:
                        pub_rev_8.publish(self.osc[i].angle_motor)
                    else:
                        pub_rev_1.publish(self.osc[i].angle_motor)

            rate.sleep()

if __name__ == '__main__':
    controller = CPGControllerConnected()
    try:
        controller.publish_positions()
    except rospy.ROSInterruptException:
        # controller.connector
        pass