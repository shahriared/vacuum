import numpy as np
import matplotlib.pyplot as plt

# Room dimensions (in meters)
room_width = 5
room_height = 7

# Grid cell size (in meters)
cell_size = 0.1  # 10 cm 

# Calculate grid dimensions in cells
grid_width = int(room_width / cell_size)
grid_height = int(room_height / cell_size)

# Create the grid
grid = np.zeros((grid_height, grid_width), dtype=int)

# Number of obstacles
num_obstacles = 5

# Minimum and maximum obstacle size (in cells)
min_obstacle_size = 2
max_obstacle_size = 5  

def is_valid_placement(obstacle_x, obstacle_y, obstacle_size):
  """
  Checks if the obstacle placement is valid (doesn't go outside the grid or overlap existing obstacles)
  """
  # Adjust the check for the bottom and right border
  return (obstacle_x + obstacle_size <= grid_width and 
          obstacle_y + obstacle_size <= grid_height and
          not np.any(grid[obstacle_y:obstacle_y + obstacle_size, obstacle_x:obstacle_x + obstacle_size] == 1))

for _ in range(num_obstacles):
  # Keep trying random placements until a valid one is found 
  while True:
    # Random obstacle size
    obstacle_size = np.random.randint(min_obstacle_size, max_obstacle_size + 1)

    # Random center coordinates within a designated area (avoid extreme borders)
    buffer_zone = 2  # Cells to leave clear from the borders
    max_center_x = grid_width - obstacle_size - buffer_zone
    max_center_y = grid_height - obstacle_size - buffer_zone
    obstacle_x = np.random.randint(buffer_zone, max_center_x + 1)
    obstacle_y = np.random.randint(buffer_zone, max_center_y + 1)

    # Check if placement is valid
    if is_valid_placement(obstacle_x, obstacle_y, obstacle_size):
      # Set the obstacle cells in the grid
      grid[obstacle_y:obstacle_y + obstacle_size, obstacle_x:obstacle_x + obstacle_size] = 1
      break  # Exit the loop if valid placement is found 

# Visualization
plt.imshow(grid, cmap='gray')  
plt.show()
