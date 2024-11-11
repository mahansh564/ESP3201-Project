import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import socket
import pickle
import os
import matplotlib.colors as mcolors
from matplotlib.patches import Polygon


def plot_matrix(Q, i, folder='plots'):
    # Create the directory if it doesn't exist
    if not os.path.exists(folder):
        os.makedirs(folder)

    grid_size = (7, 7)

    # Create a 2D array to store the Q-values for each cell's actions
    q_values_grid = np.zeros((grid_size[0], grid_size[1], 4))  # 4 actions for each state (up, down, left, right)

    # Define the actions
    actions = ['ax1_down', 'ax1_up', 'ax2_down', 'ax2_up']
    ax1_angle_limits = [0, 100]
    ax2_angle_limits = [0, 4500]

    # Map states to grid indices
    def map_state_to_grid(ax1, ax2):
        ax1_idx = int((ax1 - ax1_angle_limits[0]) / (ax1_angle_limits[1] - ax1_angle_limits[0]) * (grid_size[0] - 2))
        ax2_idx = int((ax2 - ax2_angle_limits[0]) / (ax2_angle_limits[1] - ax2_angle_limits[0]) * (grid_size[1] - 2))
        return ax1_idx, ax2_idx

    # Fill the grid with Q-values
    for (state, action), value in Q.items():
        ax1, ax2 = state
        ax1_idx, ax2_idx = map_state_to_grid(ax1, ax2)
        action_index = actions.index(action)
        q_values_grid[ax1_idx, ax2_idx, action_index] = value

    # Normalize the Q-values to map them between 0 and 1 for colormap usage
    min_q_value = np.min(q_values_grid)
    max_q_value = np.max(q_values_grid)
    norm = mcolors.Normalize(vmin=min_q_value, vmax=max_q_value)

    # Create the colormap (red to green)
    cmap = mcolors.LinearSegmentedColormap.from_list("redgreen", ["red", "green"])

    # Plot the grid
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect('equal')  # Ensure the cells are square

    # Create the grid
    for ax2_idx in range(grid_size[1]):
        for ax1_idx in range(grid_size[0]):
            # Get Q-values for the current cell and actions
            q_values = q_values_grid[ax1_idx, ax2_idx, :]

            # Coordinates for the four triangles in the current cell
            x_pos = ax1_idx
            y_pos = ax2_idx

            # Define the four corners of the cell
            cell_center = (x_pos + 0.5, y_pos + 0.5)
            left_bottom = (x_pos, y_pos)
            right_bottom = (x_pos + 1, y_pos)
            top_left = (x_pos, y_pos + 1)
            top_right = (x_pos + 1, y_pos + 1)

            # Draw each triangle for the actions
            triangles = [
                [cell_center, right_bottom, top_right], #ax1_down
                [cell_center, left_bottom, top_left],   # ax1_up
                [cell_center, top_right, top_left], # ax2_up
                [cell_center, right_bottom, left_bottom]  # ax2_down
            ]
            
            # Plot each triangle with the corresponding color
            for idx, triangle in enumerate(triangles):
                value = q_values[idx]
                # If the Q-value is 0.0, set the color to white
                if value == 0.0:
                    color = 'white'
                else:
                    color = cmap(norm(value))  # Get the corresponding color for the Q-value
                poly = Polygon(triangle, closed=True, facecolor=color, edgecolor='black')
                ax.add_patch(poly)

                # Place the Q-value text in the center of the triangle
                text_x = np.mean([triangle[0][0], triangle[1][0], triangle[2][0]])
                text_y = np.mean([triangle[0][1], triangle[1][1], triangle[2][1]])
                ax.text(text_x, text_y, f"{value:.0f}", ha='center', va='center', fontsize=8, color='black' if value != 0.0 else 'white')

    # Set labels for the axes to represent ax1 and ax2
    ax.set_xticks(np.arange(0.5, grid_size[1], 1))  # Set ticks at the center of each cell (x)
    ax.set_yticks(np.arange(0.5, grid_size[0], 1))  # Set ticks at the center of each cell (y)
    ax.set_xticklabels([f"{(i)*20}" for i in range(grid_size[1]-1)] + [''])
    ax.set_yticklabels([f"{(i)*900}" for i in range(grid_size[0]-1)] + [''])
    plt.title('Episode ' + str(5 * i))

    # Save the figure to the specified folder
    plot_filename = os.path.join(folder, 'q_matrix_plot_' + str(i) + '.png')
    plot_filename_sec = os.path.join(folder, 'q_matrix_plot.png')
    # plt.savefig(plot_filename)
    plt.savefig(plot_filename_sec)

    # Close the plot to free up memory
    plt.close(fig)



def start_server():
    plt.ion()  # Enable interactive mode for live updates
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('localhost', 65433))
    server_socket.listen(1)
    print("Plotting server is waiting for data...")

    conn, addr = server_socket.accept()
    print("Connected by", addr)

    i = 0
    while True:
        data = conn.recv(20000)
        if not data:
            print("No Data")
            break
        
        Q = pickle.loads(data)

        if plt.get_fignums():
            plt.close()

        plot_matrix(Q=Q,i=i)
        i += 1
    
    
    conn.close()
    server_socket.close()

    

if __name__ == "__main__":
    start_server()
