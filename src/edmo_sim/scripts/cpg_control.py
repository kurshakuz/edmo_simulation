#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import math

SERVOMIN = [-1.70, -1.70, -1.47]
SERVOMAX = [1.70, 1.70, 1.60]

def constrain(n, minn, maxn):
    return max(min(maxn, n), minn)

def map_range(old_value, old_min, old_max, new_min, new_max):
    old_range = (old_max - old_min)
    new_range = (new_max - new_min)
    new_value = (((old_value - old_min) * new_range) / old_range) + new_min
    return new_value

class Oscillator:
    def __init__(self, motor_num):
        self.phase = 0                     # phase of the oscillation
        self.amplitude = 0                 # amplitude of the oscillation
        self.targetAmplitude = 0           # amplitude to gradually change to
        self.offset = 90                   # offset for the oscillation (in range of servo 0-180)
        self.targetOffset = 90             # added parameter to offset smoothly
        self.rateOfPhase = 0               # current rate of change of the phase parameter
        self.rateOfAmplitude = 0           # current rate of change of the amplitude parameter
        self.rateOfOffset = 0              # current rate of change of the offset parameter
        self.pos = 0                       # oscillator output = servos angular position
        self.phaseBias = [0, 0, 0]         # controls pairwise coupling phase bias
        if motor_num == 0:                 # controls topology of the network
            self.coupling = [0, 1, 0]
            self.calib = 8
        elif motor_num == 1:
            self.coupling = [1, 0, 1]
            self.calib = 2
        else:
            self.coupling = [0, 1, 0]
            self.calib = 0
        self.angle_motor = 0               # mapped motor value

    def update_oscillator(self, targetAmplitude, targetOffset, phaseBias, convert=True):
        self.targetAmplitude = targetAmplitude
        self.targetOffset = targetOffset + 90
        phaseBiasRad = []
        for val in phaseBias:
            if convert:
                phaseBiasRad.append((math.pi * val)/180)
            else:
                phaseBiasRad.append(val)
        self.phaseBias = phaseBiasRad
        return self.targetAmplitude, self.targetOffset, self.phaseBias

class CPGController:
    def __init__(self, node_name, pub_topic_1, pub_topic_2, pub_topic_3):
        self.frequency = 0.5
        self.rateOfFrequency = 0
        self.targetFrequency = 0.5
        self.timeStep = 25

        self.w = 0.025 # we assume that all oscillators use same coupling weight
        self.a = 1 # we assume that all oscillators use same adaptation rate for amplitude
        self.c = 0.5 # adaptation rate for frequency and offset
        self.pose = [0, 0, 0]
        self.num_oscillators = len(self.pose)
        self.osc = []
        for i in range(3):
            self.osc.append(Oscillator(i))

        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(40) # 10hz

        self.pub_rev_1 = rospy.Publisher(pub_topic_1, Float64, queue_size=1)
        self.pub_rev_8 = rospy.Publisher(pub_topic_2, Float64, queue_size=1)
        self.pub_rev_13 = rospy.Publisher(pub_topic_3, Float64, queue_size=1)

    def update_controller(self, freq, weight, targetAmplitudes, targetOffsets, phaseBiases, convert=True):
        self.targetFrequency = freq
        self.w = weight
        oscillator_values = []
        for i in range(self.num_oscillators):
            oscillator_value = self.osc[i].update_oscillator(targetAmplitudes[i], targetOffsets[i], phaseBiases[i], convert=convert)
            oscillator_values.append(oscillator_value)
        print('New params: ', self.targetFrequency, self.w, oscillator_values)

    def publish_positions(self):
        # sample configuration
        # self.update_controller(freq = 0.37, weight = 0.025, targetAmplitudes = [31,18,34], targetOffsets=[34,0,-45], phaseBiases=[[0.0, 42.0, 0.0], [-42.0, 0.0, 34.0], [0.0, -34.0, 0.0]])

        # fastest configuration
        self.update_controller(freq = 0.37, weight = 0.025, targetAmplitudes = [31,18,34], targetOffsets=[64,0,58], phaseBiases=[[0.0, 90.0, 0.0], [-90.0, 0.0, 34.0], [0.0, -34.0, 0.0]])

        # zero position
        # self.update_controller(freq = 0.25, weight = 0.025, targetAmplitudes = [0,0,0], targetOffsets=[0,0,0], phaseBiases=[[0.0, 1.0, 0.0], [1.0, 0.0, 1.0], [0.0, 1.0, 0.0]], convert=False)

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
                self.osc[i].pos += self.osc[i].calib
                self.osc[i].angle_motor = map_range(self.osc[i].pos, 0, 180, SERVOMIN[i], SERVOMAX[i])
                self.osc[i].angle_motor = constrain(self.osc[i].angle_motor, SERVOMIN[i], SERVOMAX[i])
                # print(self.osc[i].angle_motor)
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
    controller = CPGController(node_name, pub_topic_1, pub_topic_2, pub_topic_3)
    try:
        controller.publish_positions()
    except rospy.ROSInterruptException:
        pass