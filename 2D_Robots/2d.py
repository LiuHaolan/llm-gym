import random

def check_overlap(vehicle1, vehicle2):
    # Expand the bounding box slightly to ensure a gap between vehicles
    expansion = 0.1
    v1_x1, v1_y1 = vehicle1[0] - vehicle1[3]/2 - expansion, vehicle1[1] - vehicle1[4]/2 - expansion
    v1_x2, v1_y2 = vehicle1[0] + vehicle1[3]/2 + expansion, vehicle1[1] + vehicle1[4]/2 + expansion
    v2_x1, v2_y1 = vehicle2[0] - vehicle2[3]/2 - expansion, vehicle2[1] - vehicle2[4]/2 - expansion
    v2_x2, v2_y2 = vehicle2[0] + vehicle2[3]/2 + expansion, vehicle2[1] + vehicle2[4]/2 + expansion

    # Check if there's any overlap
    overlap_x = v1_x1 < v2_x2 and v1_x2 > v2_x1
    overlap_y = v1_y1 < v2_y2 and v1_y2 > v2_y1
    return overlap_x and overlap_y



# Function to generate a random vehicle
def generate_vehicle():
    center_x = random.uniform(*x_range)
    center_y = random.uniform(*y_range)
    yaw = random.uniform(*yaw_range)
    width = random.uniform(*width_range)
    height = random.uniform(*height_range)
    return (center_x, center_y, yaw, width, height)




import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.transforms import Affine2D

def visualize_vehicles(vehicles, x_range, y_range):
    # Set up the plotting environment
    plt.figure(figsize=(10, 10))
    ax = plt.gca()

    # Set limits according to the 2D plane boundaries
    ax.set_xlim(x_range)
    ax.set_ylim(y_range)

    # Draw each vehicle
    for vehicle in vehicles:
        center_x, center_y, yaw, width, height = vehicle

        # Create a rectangle with the vehicle's dimensions
        rect = patches.Rectangle((-width / 2, -height / 2), width, height, linewidth=1, edgecolor='r', facecolor='none')

        # Rotate and translate the rectangle according to the vehicle's yaw and center position
        t = Affine2D().rotate_deg(-yaw) + Affine2D().translate(center_x, center_y) + ax.transData
        rect.set_transform(t)

        # Add the rectangle to the plot
        ax.add_patch(rect)

    # Additional plot settings
    ax.set_aspect('equal', adjustable='box')
    plt.title('Randomly Placed Vehicles on 2D Plane')
    plt.xlabel('X coordinate')
    plt.ylabel('Y coordinate')
    plt.grid(True)

    # Display the plot
    plt.show()

# Call the function with the generated vehicles and plane boundaries
#visualize_vehicles(vehicles, x_range, y_range)

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.transforms import Affine2D
import random

def is_position_safe(position, vehicles, robot_radius):
    for vehicle in vehicles:
        center_x, center_y, _, width, height = vehicle
        expanded_width = width / 2 + robot_radius
        expanded_height = height / 2 + robot_radius
        if abs(position[0] - center_x) < expanded_width and abs(position[1] - center_y) < expanded_height:
            return False
    return True

def generate_safe_position(x_range, y_range, vehicles, robot_radius):
    while True:
        position = (random.uniform(*x_range), random.uniform(*y_range))
        if is_position_safe(position, vehicles, robot_radius):
            return position

def is_path_safe(original_position, destination_position, vehicles, robot_radius):
    for vehicle in vehicles:
        center_x, center_y, _, width, height = vehicle
        expanded_width = width / 2 + robot_radius
        expanded_height = height / 2 + robot_radius
        if abs(center_x - (original_position[0] + destination_position[0]) / 2) < expanded_width + abs(original_position[0] - destination_position[0]) / 2 and \
           abs(center_y - (original_position[1] + destination_position[1]) / 2) < expanded_height + abs(original_position[1] - destination_position[1]) / 2:
            return False
    return True

def visualize_scenario(vehicles, original_position, destination_position, path=None):
    plt.figure(figsize=(12, 12))
    ax = plt.gca()
    ax.set_xlim(x_range)
    ax.set_ylim(y_range)

    for vehicle in vehicles:
        center_x, center_y, yaw, width, height = vehicle
        rect = patches.Rectangle((-width / 2, -height / 2), width, height, linewidth=1, edgecolor='r', facecolor='none')
        t = Affine2D().rotate_deg(-yaw) + Affine2D().translate(center_x, center_y) + ax.transData
        rect.set_transform(t)
        ax.add_patch(rect)

    ax.plot(*original_position, 'go', markersize=10, label='Original Position')
    ax.plot(*destination_position, 'bo', markersize=10, label='Destination Position')

    # visualize path 
    if path is not None:
        ax.plot([path[0][0], path[1][0]], [path[0][1], path[1][1]], 'k--', label='Path')

    ax.set_aspect('equal', adjustable='box')
    plt.title('Scenario with Vehicles, Original and Destination Positions, and Path')
    plt.xlabel('X coordinate')
    plt.ylabel('Y coordinate')
    plt.legend()
    plt.grid(True)
    plt.show()

# Generate vehicles ensuring no overlap
vehicles = []
attempts_per_vehicle = 100  # Maximum attempts to place each vehicle to avoid infinite loops

# Parameters for vehicle generation
min_vehicles, max_vehicles = 20, 50
x_range, y_range = (-100, 100), (-100, 100)
yaw_range = (0, 360)
width_range, height_range = (1, 5), (1, 5)

# Randomly decide the number of vehicles
num_vehicles = random.randint(min_vehicles, max_vehicles)

for _ in range(num_vehicles):
    for _ in range(attempts_per_vehicle):
        new_vehicle = generate_vehicle()
        if all(not check_overlap(new_vehicle, existing_vehicle) for existing_vehicle in vehicles):
            vehicles.append(new_vehicle)
            break

# Robot radius
robot_radius = 4

# Generate safe original and destination positions
original_position = generate_safe_position(x_range, y_range, vehicles, robot_radius)
destination_position = generate_safe_position(x_range, y_range, vehicles, robot_radius)

path = None
# Example usage
# uncomment the path generation for your life
# path = generate_rrt_star_path(original_position, destination_position, vehicles, robot_radius)

# Call the visualization function
visualize_scenario(vehicles, original_position, destination_position, path)

