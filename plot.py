import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import socket
import pickle

def plot_matrix(Q):
    keys = list(Q.keys())
    values = list(Q.values())

    x_vals = [k[0][0] for k in keys]
    y_vals = [k[0][1] for k in keys]

    # Get unique x and y values for the axes
    x_unique = sorted(set(x_vals))
    y_unique = sorted(set(y_vals))
    
    # Create a new array where each original cell is represented by a 2x2 subgrid
    expanded_heatmap_data = np.zeros((len(y_unique) * 2, len(x_unique) * 2))
    transformed_Q = {}

    for (ax_tuple, label), value in Q.items():
        if ax_tuple not in transformed_Q:
            transformed_Q[ax_tuple] = {}
        transformed_Q[ax_tuple][label] = value

    # Populate the expanded data array with the values in each 2x2 subgrid
    for ((x, y), ax), value in Q.items():
        x_index = x_unique.index(x) * 2
        y_index = y_unique.index(y) * 2
        expanded_heatmap_data[y_index, x_index] = transformed_Q[(x, y)].get('ax1_down', 0)
        expanded_heatmap_data[y_index+1, x_index] = transformed_Q[(x, y)].get('ax1_up', 0)
        expanded_heatmap_data[y_index, x_index+1] = transformed_Q[(x, y)].get('ax2_down', 0)
        expanded_heatmap_data[y_index+1, x_index+1] = transformed_Q[(x, y)].get('ax2_up', 0)

    # Plot heatmap with centered labels
    plt.figure(figsize=(20, 15), dpi=100)
    sns.heatmap(
        expanded_heatmap_data, annot=True, fmt=".1f", cmap="RdYlGn", cbar=True,
        xticklabels=[f"{x}\n" for x in x_unique],
        yticklabels=[f"{y}\n" for y in y_unique],
        square=True  # Maintain square cells for better alignment
    )
    
    plt.xticks(np.arange(0.5, len(x_unique) * 2, 2), x_unique)
    plt.yticks(np.arange(0.5, len(y_unique) * 2, 2), y_unique)

    plt.xlabel("X values")
    plt.ylabel("Y values")
    plt.show()
    plt.pause(0.01)

def start_server():
    plt.ion()  # Enable interactive mode for live updates
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('localhost', 65432))
    server_socket.listen(1)
    print("Plotting server is waiting for data...")

    conn, addr = server_socket.accept()
    print("Connected by", addr)

    while True:
        data = conn.recv(4096)
        if not data:
            break
        Q = pickle.loads(data)
        if plt.get_fignums():
            plt.close()
        plot_matrix(Q)
    conn.close()
    server_socket.close()

if __name__ == "__main__":
    start_server()
