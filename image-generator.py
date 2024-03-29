from PIL import Image, ImageDraw
import random

# Define image dimensions
width = 48
height = 40

# Create a new black image
image = Image.new("L", (width, height), color=0)  # L for greyscale, 0 for black

# Create a drawer object
draw = ImageDraw.Draw(image)

# Function to draw random walls within the image
def draw_random_walls(max_walls=5):
  for _ in range(max_walls):
    # Define random starting and ending points for the wall
    x1 = random.randint(0, width)
    y1 = random.randint(0, height)
    x2 = random.randint(0, width)
    y2 = random.randint(0, height)
    # Draw a line representing the wall
    draw.line((x1, y1, x2, y2), fill=255, width=2)  # 255 for white, width for thickness

# Draw random walls to create the room layout
draw_random_walls()

# Display or save the image
# image.show()  # uncomment to display the image
image.save("room_random_walls.png")  # save as room_random_walls.png

print("Room image with random walls (connected white space) generated as room_random_walls.png")

import random  # Import for random number generation
