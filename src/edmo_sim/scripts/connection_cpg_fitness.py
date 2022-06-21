#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import math
import json
from sklearn.linear_model import LinearRegression
import numpy as np

from cpg_control import SERVOMIN, SERVOMAX, constrain, map_range, Oscillator, CPGController
from core.SimulatorServerConnector import SimulatorServerConnector

recording_time = 20

def euclidean_distance(x1: int, x2: int, y1: int, y2: int) -> float:
    return np.linalg.norm(np.array((x1, y1)) - np.array((x2, y2)))

def estimate_speed_regression(checkpoint_distances: list, elapsed_time: float) -> float:
    linear = LinearRegression()
    linear.fit(np.array(range(len(checkpoint_distances))).reshape(-1, 1), np.array(checkpoint_distances))
    speed = linear.coef_[0] * len(checkpoint_distances) / elapsed_time
    speed = np.clip(speed, 0.0, None)
    return speed

def estimate_speed_end2end(first_dist, last_dist, elapsed_time):
    speed = (last_dist - first_dist) / elapsed_time
    return speed

def convert_coords_to_distances(x_coords, y_coords):
    x_origin = x_coords[0]
    y_origin = y_coords[0]
    checkpoint_distances = []
    for i in range(0, len(x_coords)):
        checkpoint_distances.append(euclidean_distance(x_coords[i], x_origin, y_coords[i], y_origin))
    return checkpoint_distances

class CPGControllerConnected(CPGController):
    def __init__(self, node_name, pub_topic_1, pub_topic_2, pub_topic_3):
        super().__init__(node_name, pub_topic_1, pub_topic_2, pub_topic_3)
        self.connector = SimulatorServerConnector()
        self.connector.register_rcv_listener(self.update_from_json)
        host = '192.168.114.222'
        # host = 'localhost'
        self.connector.connect(host=host)
        rospy.loginfo(f'Connected to host {host}')
        self.received_json = False
        self.recording_state = False
        self.finished_recording = False
        self.last_received_time = 0.0
        self.sim_task_id = 0

    def update_from_json(self, msg):
        # TODO remove received_json
        rospy.loginfo(f'Received JSON input from {msg["name"]}')
        targetAmplitudes = []
        targetOffsets = []
        for i in range(len(msg['modules'])):
            targetAmplitudes.append(msg['modules'][i]['amplitude'])
            targetOffsets.append(msg['modules'][i]['offset'])
        self.update_controller(msg['frequency'], msg['weight'], targetAmplitudes, targetOffsets, msg['phase_bias_matrix'])
        self.last_received_time = rospy.get_time()
        rospy.loginfo(f'Received command at second: {self.last_received_time}')

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
            checkpoint_distances = convert_coords_to_distances(self.x_positions, self.y_positions)
            result = estimate_speed_regression(checkpoint_distances, recording_time) * 100
            # result = estimate_speed_end2end(checkpoint_distances[0], checkpoint_distances[len(checkpoint_distances)-1], recording_time) * 100
            self.connector.respond_result(self.sim_task_id, result)
            rospy.loginfo(f'Returned command at second: {rospy.get_time()}')
            rospy.loginfo(f'Returned fitness: {result}')
            self.finished_recording = False

    def publish_positions(self):
        while not rospy.is_shutdown():
            if self.received_json:
                current_time = rospy.get_time()
                if current_time - (self.last_received_time + recording_time) >= 0:
                    rospy.loginfo('10 secs passed')
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
                self.osc[i].pos += self.osc[i].calib
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