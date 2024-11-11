import socket
from RobotCrawler import RoboCrawler
import random
import math
import pickle
import os

class QLearningAgent:
    def __init__(self, conn, start_epsilon, end_epsilon, exp_multiplier,  alpha, gamma, Q = {}, init_ax1 = 0, init_ax2 = 0):
        self.robot = RoboCrawler(conn)
        self.n_steps = 5
        self.ax1_step_length = int(self.robot.ax1_angle_limits[1]/(self.n_steps))
        self.ax2_step_length = int(self.robot.ax2_angle_limits[1]/(self.n_steps))

        self.init_ax1 = init_ax1
        self.init_ax2 = init_ax2
        
        self.Q = Q
        if Q:
            print("Previous Q matrix used")
        else:
            for a in ['ax1_down','ax1_up','ax2_down','ax2_up']:
                for i in range(self.n_steps+1):
                    for j in range(self.n_steps+1):
                        Q[((i*self.ax1_step_length, j*self.ax2_step_length),a)] = 0

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
    
        
    def server(self, port = 65433):
        plot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        plot_socket.connect_ex(('localhost', port))
        return plot_socket

    def pickle_data(self, action, israndom, state, new_state, reward, next_action = None):
        tx_data = {}
        tx_data['Q'] = self.Q
        tx_data['current_action'] = action
        tx_data['is_random'] = israndom
        tx_data['state'] = state
        tx_data['new_state'] = new_state
        tx_data['reward'] = reward
        tx_data['epsilon'] = self.epsilon

        if next_action != None:
            tx_data['next_action'] = next_action

        serialized_Q = pickle.dumps(tx_data)
        return serialized_Q

    def getPossibleActions(self, state):

        possible_actions = []
        if(state[0] + self.ax1_step_length <= self.robot.ax1_angle_limits[1]):
            possible_actions.append('ax1_down')
        if(state[0] - self.ax1_step_length >= self.robot.ax1_angle_limits[0]):
            possible_actions.append('ax1_up')        
        if(state[1] + self.ax2_step_length <= self.robot.ax2_angle_limits[1]):
            possible_actions.append('ax2_down')                            
        if(state[1] - self.ax2_step_length >= self.robot.ax2_angle_limits[0]):
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
        if(self.robot.distance_mm() > 500) or (self.robot.distance_mm() < 50):
            input("RESET ROBOT POSITION THEN PRESS ENTER:")
        
        if abs(total_distance) < 10: #ignore small values created by inertia of arm
            total_distance = 0

        if total_distance > 0:
            reward = total_distance
        else:
            reward = total_distance*5 #negative penalized more

        #wait(200) #wait for motion to finish completly
        return [state, reward]
    
    def run(self, number_of_steps):
        #plot_socket = self.server()
        plot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        plot_socket.connect(('localhost', 65433))
        self.robot.Homing()
        self.epsilon = 0
        state = (self.init_ax1,self.init_ax2)
        self.robot.go_to_angle(self.init_ax1,self.init_ax2)
        for i in range(number_of_steps):
            old_state = state
            [israndom, action] = self.getAction(state)
            [new_state, reward] = self.step(state,action)
            next_action = self.getPolicy(new_state)
            state = new_state
            print("step: ", i, "//action: ", action, "//random: ", israndom,"//state: ", state, "//reward: ", reward)

            serialized_Q = self.pickle_data(action, israndom, old_state, new_state, reward, next_action)
            plot_socket.sendall(serialized_Q)
        
        plot_socket.close()
        self.robot.ax11.stop_action = 'coast'
        self.robot.ax12.stop_action = 'coast'
        self.robot.ax2.stop_action = 'coast'
        self.robot.ax11.stop()
        self.robot.ax12.stop()
        self.robot.ax2.stop()

        return self.Q
                

    def QLearning(self,number_of_episodes):
        self.robot.Homing()
        state = (self.init_ax1,self.init_ax2)
        self.robot.go_to_angle(self.init_ax1,self.init_ax2)

        # Initialize socket connection to the plotting server
        plot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        plot_socket.connect(('localhost', 65433))
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

            serialized_Q = self.pickle_data(action, israndom, state, new_state, reward)

            if i%5 == 0:
                plot_socket.sendall(serialized_Q)
                

            print("episode: ", i, "//epsilon: ", round(self.epsilon,1),"//action: ", action,"//random: ", israndom,"//state: ", state, "//reward: ", reward)

        

        filename = os.path.join('Q-Saves', 'Q_Save.pkl')
        with open(filename, 'wb') as file:
            pickle.dump(self.Q, file)

        plot_socket.close()
        self.robot.ax11.stop_action = 'coast'
        self.robot.ax12.stop_action = 'coast'
        self.robot.ax2.stop_action = 'coast'
        self.robot.ax11.stop()
        self.robot.ax12.stop()
        self.robot.ax2.stop()

        return self.Q
