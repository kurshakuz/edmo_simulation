#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64, String
import math

# SERVOMIN = [108, 108, 108] # this is the 'minimum' pulse length count (out of 4096)
# SERVOMAX = [504, 498, 474] # this is the 'maximum' pulse length count (out of 4096)

SERVOMIN = [-1.47, -1.47, -1.47] # this is the 'minimum' pulse length count (out of 4096)
SERVOMAX = [1.47, 1.47, 1.47] # this is the 'maximum' pulse length count (out of 4096)


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
        # controls topology of the network
        if motor_num == 0:
            self.coupling = [0, 1, 0]
            self.set_calib(-3)
        elif motor_num == 1:
            self.coupling = [1, 0, 1]
            self.set_calib(-4)
        else:
            self.coupling = [0, 1, 0]
            self.set_calib(-10)
        self.angle_motor = 0               # mapped motor value

    def zero_calib(self):
        self.calib = 0

    def set_calib(self, val):
        self.calib = val

    def update_oscillator(self, targetAmplitude, targetOffset, phaseBias):
        self.targetAmplitude = targetAmplitude
        self.targetOffset = targetOffset
        self.phaseBias = phaseBias

class CPGController:
    def __init__(self):
        self.frequency = 0.5
        self.rateOfFrequency = 0
        self.targetFrequency = 0.5
        self.timeStep = 25

        self.w = 0.025 # we assume that all oscillators use same coupling weight
        self.a = 1 # we assume that all oscillators use same adaptation rate for amplitude
        self.c = 0.5 # adaptation rate for frequency and offset
        self.pose = [0, 0, 0]
        self.num_oscillators = len(self.pose)
        self.left_end = -90
        self.right_end = 90
        self.osc = []
        for i in range(3):
            self.osc.append(Oscillator(i))

    def update_controller(self, freq, weight, targetAmplitudes, targetOffsets, phaseBiases):
        self.frequency = freq
        self.w = weight
        for i in range(self.num_oscillators):
            self.osc[i].update_oscillator(targetAmplitudes[i], targetOffsets[i], phaseBiases[i])

    def publish_positions(self):
        rospy.init_node('motor_command_pub', anonymous=True)
        rate = rospy.Rate(20) # 10hz

        pub_rev_1 = rospy.Publisher('/edmo_snake_controller/Rev1_position_controller/command', Float64, queue_size=1)
        pub_rev_8 = rospy.Publisher('/edmo_snake_controller/Rev8_position_controller/command', Float64, queue_size=1)
        pub_rev_13 = rospy.Publisher('/edmo_snake_controller/Rev13_position_controller/command', Float64, queue_size=1)

        # NEED TO RECEIVE AND CHANGE VARIABLES HERE
        self.update_controller(freq = 0.37, weight = 0.025, targetAmplitudes = [31,18,34], targetOffsets=[34,0,-45], phaseBiases=[[0.0, 42.0, 0.0], [-42.0, 0.0, 34.0], [0.0, -34.0, 0.0]])

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
                self.osc[i].pos += self.osc[i].calib
                # self.osc[i].angle_motor = map_range(self.osc[i].pos, 0, 180, SERVOMIN[i], SERVOMAX[i])
                self.osc[i].angle_motor = map_range(self.osc[i].pos, -99, 99, SERVOMIN[i], SERVOMAX[i])
                self.osc[i].angle_motor = constrain(self.osc[i].angle_motor, SERVOMIN[i], SERVOMAX[i])
                print(self.osc[i].angle_motor)
                if i == 0:
                    pub_rev_1.publish(self.osc[i].angle_motor)
                elif i == 1:
                    pub_rev_8.publish(self.osc[i].angle_motor)
                else:
                    pub_rev_13.publish(self.osc[i].angle_motor)

            hello_str = "%s" % rospy.get_time()
            rospy.loginfo(hello_str)
            rate.sleep()

if __name__ == '__main__':
    controller = CPGController()
    try:
        controller.publish_positions()
    except rospy.ROSInterruptException:
        pass