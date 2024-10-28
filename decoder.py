import base64
import pickle

# Replace with the actual Base64 data you received

user_input = input("Enter the copied binary (press return to select hardcoded binary): ")

if not user_input:  # If the input is empty
    user_input = "eydRJzogeygoMjAsIDQ1MCksICdheDFfZG93bicpOiAwLjAsICgoMzAsIDQ1MCksICdheDJfZG93bicpOiAwLjAsICgoNDAsIDkwMCksICdheDFfZG93bicpOiAwLjAsICgoNTAsIDkwMCksICdheDFfZG93bicpOiAwLjAsICgoMzAsIDkwMCksICdheDFfZG93bicpOiAwLjAsICgoMCwgNDUwKSwgJ2F4MV9kb3duJyk6IDAuMCwgKCgwLCA5MDApLCAnYXgyX3VwJyk6IDAuMCwgKCgwLCAwKSwgJ2F4Ml9kb3duJyk6IDAuMCwgKCgxMCwgNDUwKSwgJ2F4MV9kb3duJyk6IDAuMCwgKCgwLCA0NTApLCAnYXgyX2Rvd24nKTogMC4wfX0=----------"

# Decode the data and write it to a binary file
binary_data = base64.b64decode(user_input)
with open("received_data.pkl", "wb") as file:
    pickle.dump(binary_data, file)

with open("received_data.pkl", "rb") as file:
    Q = pickle.load(file)
    print(Q)