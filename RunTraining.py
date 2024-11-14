import rpyc

# Create a RPyC connection to the remote ev3dev device.
# Use the hostname or IP address of the ev3dev device.
# If this fails, verify your IP connectivty via ``ping X.X.X.X``
conn = rpyc.classic.connect('ev3dev.local')
# conn = rpyc.classic.connect('169.254.244.90')

conn.execute("print('Hello Slave. I am your master!')")

from RobotCrawler import RoboCrawler
from QLearningAgent import QLearningAgent
import pickle
import os 

Q = {}

optimal_Q = 'Q_Save 5x5 optimized.pkl'
filename = os.path.join('Q-Saves', optimal_Q if optimal_Q else 'Q_Save.pkl')
with open(filename, 'rb') as file:
    Q = pickle.load(file)

agent = QLearningAgent(conn, 0.85, 0.1, 1, 0.5, 0.75, Q if Q else {})#, 80, 2700)
# Q = agent.QLearning(2000)
Q = agent.run(100)
