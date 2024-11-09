import rpyc

# Create a RPyC connection to the remote ev3dev device.
# Use the hostname or IP address of the ev3dev device.
# If this fails, verify your IP connectivty via ``ping X.X.X.X``
conn = rpyc.classic.connect('ev3dev.local')
conn.execute("print('Hello Slave. I am your master!')")

from RobotCrawler import RoboCrawler
from QLearningAgent import QLearningAgent
import pickle


# r = RoboCrawler(conn)
# r.Homing()
# r.Walk(10)




with open('Q_Save.pkl', 'rb') as file:
    Q = pickle.load(file)

agent = QLearningAgent(conn, 0.95, 0.2, 1.2, 0.75, 0.95, Q if Q else {})
Q = agent.QLearning(1000)

