import pickle
import os


filename = os.path.join('Q-Saves', 'Q_Save.pkl')
with open(filename, 'rb') as file:
    Q = pickle.load(file)

print(Q)








# Q = 


# filename = os.path.join('Q-Saves', 'Q_Save.pkl')
# with open(filename, 'wb') as file:
#     pickle.dump(Q, file)