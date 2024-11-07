#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import random, math
import pickle
import sys
import base64
from os import path

file_name = "saved_Q.pkl"

class RoboCrawler:
    def __init__(self):
        self.ev3 = EV3Brick()
        self.ax11 = Motor(Port.A)
        self.ax12 = Motor(Port.D)
        self.ax2 = Motor(Port.B)
        self.endstop_ax1 = TouchSensor(Port.S4)
        # self.endstop_ax2 = TouchSensor(Port.S1)
        self.ultrasonic = UltrasonicSensor(Port.S3)
        self.gyro = GyroSensor(Port.S2)

        self.Homing_Speed = 80
        self.ax1_speed = self.Homing_Speed
        self.ax2_speed = self.Homing_Speed*8

        self.ax1_angle_limits = [0,100]
        self.ax2_angle_limits = [0,4500]
    
    def TestSensor(self, sensor, seconds):
        watch = StopWatch()

        while watch.time() < seconds*1000:
            wait(100)
            # (done with if else because match case is not supported)
            if sensor == "endstop1":
                print("endstop1 state is: " + str(self.endstop_ax1.pressed()))
            elif sensor == "endstop2":
                print("endstop2 state is: " + str(self.endstop_ax2.pressed()))
            elif sensor == "motor11":
                print("Motor11 encoder state is: " + str(self.ax11.angle()))
            elif sensor == "motor12":
                print("Motor12 encoder state is: " + str(self.ax12.angle()))
            elif sensor == "motor2":
                print("Motor2 encoder state is: " + str(self.ax2.angle()))
            elif sensor == "ultrasonic":
                print("ultrasonic distance state is: " + str(self.ultrasonic.distance()))
            elif sensor == "gyro":
                print("gyro angle state is: " + str(self.gyro.angle()))
            else:
                print("Unknown sensor")  # Default case if no match


                


    def Homing(self):

        # Axis 1 
        while(True):
            self.ax11.run(-self.Homing_Speed)
            self.ax12.run(-self.Homing_Speed)            
            if(self.endstop_ax1.pressed()):
                self.ax11.hold()
                self.ax12.hold()
                self.ax11.reset_angle(0)
                self.ax12.reset_angle(0)

                self.ev3.speaker.beep()
                break; 
        
        # Axis 2

        self.ax2.run_until_stalled(-self.Homing_Speed*10, then=Stop.HOLD, duty_limit=40)
        self.ax2.reset_angle(0)
        self.ev3.speaker.beep()
        # while(True):
        #     self.ax2.run(-self.Homing_Speed*10)
        #     if(self.endstop_ax2.pressed()):
        #         self.ax2.hold()
        #         self.ax2.reset_angle(0)
        #         self.ev3.speaker.beep()
        #         break; 
    
    def go_to_angle(self, angle1=-1, angle2=-1):

        if(angle1 < 0):
            if(angle1 < -1):
                print("NEGATIVE ANGLE SENT TO AX1: ", angle1)
            angle1 = self.ax11.angle()


        if(angle2 < 0):
            if(angle2 < -1):
                print("NEGATIVE ANGLE SENT TO AX2: ", angle2)
            angle2 = self.ax2.angle()


        self.ax11.run_target(self.ax1_speed, angle1, then=Stop.HOLD, wait=False)                
        self.ax12.run_target(self.ax1_speed, angle1, then=Stop.HOLD, wait=True)
        self.ax2.run_target(self.ax2_speed*8, angle2, then=Stop.HOLD, wait=True)

    def Walk(self, steps):

        for i in range(steps):
            self.go_to_angle(70,4000)
            self.go_to_angle(30,1000)

    def distance_mm(self): #sensor has to be angled up a bit#
        # return self.ultrasonic.distance() * math.sin(math.pi/2 - self.gyro.angle()*math.pi/180)
        return self.ultrasonic.distance()

    def beep(self, n):
        for i in range(n):
            wait(300)
            self.ev3.speaker.beep()

    def getState(self):
        return(self.ax11.angle(), self.ax2.angle())
    
class QLearningAgent:
    def __init__(self, start_epsilon, end_epsilon, exp_multiplier, alpha, gamma, Q={}):
        self.robot = RoboCrawler()
        self.n_steps = 10
        self.ax1_step_length = int(self.robot.ax1_angle_limits[1] / self.n_steps)
        self.ax2_step_length = int(self.robot.ax2_angle_limits[1] / self.n_steps)

        self.Q = Q
        print("Previous Q matrix: ", Q)
        self.inital_epsilon = start_epsilon
        self.end_epsilon = end_epsilon
        self.exp_multiplier = exp_multiplier
        self.epsilon = 1

        self.alpha = alpha  # learning rate
        self.gamma = gamma  # discount factor

        self.AllActions = {'ax1_down': 1,
                           'ax1_up': -1,
                           'ax2_down': 1,
                           'ax2_up': -1}

    def getPossibleActions(self, state):
        possible_actions = []
        if state[0] + self.ax1_step_length < self.robot.ax1_angle_limits[1]:
            possible_actions.append('ax1_down')
        if state[0] - self.ax1_step_length > self.robot.ax1_angle_limits[0]:
            possible_actions.append('ax1_up')
        if state[1] + self.ax2_step_length < self.robot.ax2_angle_limits[1]:
            possible_actions.append('ax2_down')
        if state[1] - self.ax2_step_length > self.robot.ax2_angle_limits[0]:
            possible_actions.append('ax2_up')
        return possible_actions

    def getQValue(self, state, action):
        return self.Q.get((state, action), 0.0)

    def getValue(self, state):
        possible_actions = self.getPossibleActions(state)
        if not possible_actions:
            return 0.0
        return max(self.getQValue(state, action) for action in possible_actions)

    def getPolicy(self, state):
        possible_actions = self.getPossibleActions(state)
        if not possible_actions:
            return None
        return max(possible_actions, key=lambda action: self.getQValue(state, action))

    def getAction(self, state):
        possible_actions = self.getPossibleActions(state)
        if not possible_actions:
            return None
        if random.random() < self.epsilon:
            return [1, random.choice(possible_actions)]
        else:
            return [0, self.getPolicy(state)]

    def step(self, state, action):
        start_distance = self.robot.distance_mm()

        if action == 'ax1_down' or action == 'ax1_up':
            next_ax1_angle = state[0] + self.AllActions[action] * self.ax1_step_length
            self.robot.go_to_angle(angle1=next_ax1_angle)
            state = (next_ax1_angle, state[1])
        elif action == 'ax2_down' or action == 'ax2_up':
            next_ax2_angle = state[1] + self.AllActions[action] * self.ax2_step_length
            self.robot.go_to_angle(angle2=next_ax2_angle)
            state = (state[0], next_ax2_angle)
        else:
            print("NOT A VALID ACTION")

        total_distance = self.robot.distance_mm() - start_distance
        if abs(total_distance) < 10:
            total_distance = 0

        reward = total_distance / 10 if total_distance > 0 else total_distance
        wait(200)
        return [state, reward]

    def update(self, state, action, nextState, cumulative_reward):
        if (state, action) not in self.Q:
            self.Q[(state, action)] = 0.0
        nextStateValue = self.getValue(nextState)
        self.Q[(state, action)] += self.alpha * (cumulative_reward + self.gamma * nextStateValue - self.Q[(state, action)])

    def QLearning(self, number_of_episodes):
        self.robot.Homing()
        for i in range(number_of_episodes):
            state = (0, 0)
            cumulative_reward = 0  # Initialize cumulative reward for this episode
            episode_transitions = []  # Store transitions for the episode

            # Decay epsilon
            if self.inital_epsilon * math.exp(-(i / number_of_episodes) * self.exp_multiplier) > self.end_epsilon:
                self.epsilon = self.inital_epsilon * math.exp(-(i / number_of_episodes) * self.exp_multiplier)
            else:
                self.epsilon = self.end_epsilon

            # Run the episode
            for _ in range(35):  # Set a fixed number of steps per episode or terminate based on a condition
                israndom, action = self.getAction(state)
                next_state, reward = self.step(state, action)
                episode_transitions.append((state, action, next_state))
                cumulative_reward += reward
                state = next_state

            # Update Q-values based on the cumulative reward after each episode
            for state, action, next_state in episode_transitions:
                self.update(state, action, next_state, cumulative_reward)

            print("Episode", i, "| Epsilon: ", self.epsilon, "| Cumulative Reward: ", cumulative_reward, "| Random: ", israndom)

        return self.Q

Q = {}
print("Previous Q matrix: ", path.exists(file_name))
if path.exists(file_name):
    with open(file_name, 'rb') as file:
        data = pickle.load(file)
        Q = data["Q"]

if Q:
    print(Q)

# agent = QLearningAgent(0.85, 0.2, 1, 0.8, 0.70, Q if Q else {})
# Q = agent.QLearning(100)