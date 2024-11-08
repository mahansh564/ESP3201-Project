import socket
from RobotCrawler import RoboCrawler
import random
import math
import pickle

class QLearningAgent:
    def __init__(self, conn, start_epsilon, end_epsilon, exp_multiplier,  alpha, gamma, Q = {}):
        self.robot = RoboCrawler(conn)
        self.n_steps = 10
        self.ax1_step_length = int(self.robot.ax1_angle_limits[1]/self.n_steps)
        self.ax2_step_length = int(self.robot.ax2_angle_limits[1]/self.n_steps)
        
        self.Q = Q
        print("Previous Q matrix: ", Q)
        self.inital_epsilon = start_epsilon #(start exploration prob)
        self.end_epsilon = end_epsilon #( end exploration prob)
        self.exp_multiplier = exp_multiplier
        self.epsilon = 1 
       
        self.alpha = alpha #(learning rate)
        self.gamma = gamma #(discount rate)

        self.AllActions = {'ax1_down':1,
                           'ax1_up':-1,
                           'ax2_down':1,
                           'ax2_up':-1}
    
        

    def getPossibleActions(self, state):

        possible_actions = []
        if(state[0] + self.ax1_step_length < self.robot.ax1_angle_limits[1]):
            possible_actions.append('ax1_down')
        if(state[0] - self.ax1_step_length > self.robot.ax1_angle_limits[0]):
            possible_actions.append('ax1_up')        
        if(state[1] + self.ax2_step_length < self.robot.ax2_angle_limits[1]):
            possible_actions.append('ax2_down')                            
        if(state[1] - self.ax2_step_length > self.robot.ax2_angle_limits[0]):
            possible_actions.append('ax2_up')                            
                             
        return possible_actions

    
    def getQValue(self, state, action):
        if (state,action) not in self.Q:
            return 0.0
        else:
            return self.Q[(state,action)]
    
    def getValue(self, state):

        if len(self.getPossibleActions(state)) == 0:
            return 0.0

        max_q = []
        
        for a in self.getPossibleActions(state):

            if len(max_q) == 0 or self.getQValue(state, a) > max_q[-1]:
                max_q.append(self.getQValue(state, a))
        
        return max_q[-1]   


    def getPolicy(self, state):
        if len(self.getPossibleActions(state)) == 0:
            return 0.0
        
        max_q = []
        max_q_a = []
        
        for a in self.getPossibleActions(state):
            if len(max_q) == 0 or self.getQValue(state, a) > max_q[-1]:
                max_q.append(self.getQValue(state, a))
                max_q_a.append(a)
        
        if(max_q.count(max(max_q))>1):
            action = random.choice(max_q_a[-max_q.count(max(max_q)):])
            
        else:
            action = max_q_a[-1]
            
        return action



    def getAction(self, state):
        if len(self.getPossibleActions(state)) == 0:
            return None
        
        if random.random() < self.epsilon:
            return [1 ,random.choice(self.getPossibleActions(state))]
        else:
            return [0, self.getPolicy(state)]
        
    def update(self, state, action, nextState, reward):
        
        if (state,action) not in self.Q:
            self.Q[(state,action)] = 0.0
        nextStateValue = self.getValue(nextState)
        if nextStateValue == None:
            nextStateValue = 0.0


        self.Q[(state,action)] = self.Q[(state,action)] + self.alpha*(reward + self.gamma*nextStateValue - self.Q[(state,action)])

    def step(self, state, action):

        start_distance = self.robot.distance_mm()
        
        if(action == 'ax1_down' or action == 'ax1_up'):
            next_ax1_angle = state[0] + self.AllActions[action]*self.ax1_step_length
            self.robot.go_to_angle(angle1=next_ax1_angle)
            state = (next_ax1_angle,state[1])
               
        elif(action == 'ax2_down' or action == 'ax2_up'):
            next_ax2_angle = state[1] + self.AllActions[action]*self.ax2_step_length
            self.robot.go_to_angle(angle2=next_ax2_angle)
            state = (state[0],next_ax2_angle)
        else:
            print("NOT A VALID ACTION")
            
        total_distance = self.robot.distance_mm() - start_distance
        
        if abs(total_distance) < 10: #ignore small values created by inertia of arm
            total_distance = 0

        if total_distance > 0:
            reward = total_distance/10
        else:
            reward = total_distance #negative penalized more

        #wait(200) #wait for motion to finish completly
        return [state, reward]
    
    def QLearning(self,number_of_episodes):
        self.robot.Homing()
        state = (0,0)

         # Initialize socket connection to the plotting server
        plot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        plot_socket.connect(('localhost', 65432))
        for i in range(number_of_episodes):

            if(self.inital_epsilon*math.exp(-(i/number_of_episodes)*self.exp_multiplier) > self.end_epsilon):
                self.epsilon = self.inital_epsilon*math.exp(-(i/number_of_episodes)*self.exp_multiplier)
            else:
                self.epsilon = self.end_epsilon

            
            [israndom, action] = self.getAction(state)
            [new_state, reward] = self.step(state,action)
            self.update(state,action,new_state,reward)
            state = new_state
            # Send the Q matrix to the plotting server
            serialized_Q = pickle.dumps(self.Q)
            plot_socket.sendall(serialized_Q)
            print("episode: ", i, "//epsilon: ", round(self.epsilon,1),"//action: ", action,"//random: ", israndom,"//state: ", state, "//reward: ", reward)

        plot_socket.close()
        return self.Q
