import rpyc

# Create a RPyC connection to the remote ev3dev device.
# Use the hostname or IP address of the ev3dev device.
# If this fails, verify your IP connectivty via ``ping X.X.X.X``
conn = rpyc.classic.connect('ev3dev.local')
conn.execute("print('Hello Slave. I am your master!')")

from RobotCrawler import RoboCrawler
from QLearningAgent import QLearningAgent


# r = RoboCrawler(conn)
# r.Homing()
# r.Walk(10)
Q = {}
agent = QLearningAgent(conn, 0.85, 0.2, 1, 0.5, 0.75, Q if Q else {})
Q = agent.QLearning(100)