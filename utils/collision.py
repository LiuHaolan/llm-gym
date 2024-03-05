import math

def rotate_point(x, y, angle):
    """Rotate a point counterclockwise by a given angle around the origin."""
    x_new = x * math.cos(angle) - y * math.sin(angle)
    y_new = x * math.sin(angle) + y * math.cos(angle)
    return x_new, y_new

def check_collision(rect_x, rect_y, yaw, length, width, circle_x, circle_y, radius):
    # Translate and rotate circle's center to align the rectangle with axes
    translated_x = circle_x - rect_x
    translated_y = circle_y - rect_y
    rotated_x, rotated_y = rotate_point(translated_x, translated_y, -yaw)
    
    # Compute the half dimensions of the rectangle
    half_length = length / 2
    half_width = width / 2
    
    # Find the closest point to the circle within the rectangle
    closest_x = max(-half_length, min(rotated_x, half_length))
    closest_y = max(-half_width, min(rotated_y, half_width))
    
    # Calculate the distance between the circle's center and this closest point
    distance_x = rotated_x - closest_x
    distance_y = rotated_y - closest_y
    
    # If the distance is less than the circle's radius, they are colliding
    distance = math.sqrt(distance_x**2 + distance_y**2)
    return distance < radius

if __name__ == '__main__':
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    import numpy as np
    import random
    import math

    # Assuming the check_collision function is defined as above

    def plot_rectangle(ax, x, y, yaw, length, width, color='b'):
        """Plot a rotated rectangle."""
        # Corners of the rectangle before rotation
        corners = np.array([[-length/2, -width/2],
                            [-length/2, width/2],
                            [length/2, width/2],
                            [length/2, -width/2],
                            [-length/2, -width/2]])
        
        # Rotate and translate corners
        t_corners = [rotate_point(cx, cy, yaw) for cx, cy in corners]
        t_corners = np.array([[cx + x, cy + y] for cx, cy in t_corners])
        
        # Create a polygon and add it to the plot
        poly = patches.Polygon(t_corners, closed=True, edgecolor=color, fill=False)
        ax.add_patch(poly)

    def plot_circle(ax, x, y, radius, color='r'):
        """Plot a circle."""
        circle = patches.Circle((x, y), radius, edgecolor=color, fill=False)
        ax.add_patch(circle)

    # Set up the plot
    fig, axs = plt.subplots(2, 3, figsize=(15, 10)) # Adjust the figure size as needed
    axs = axs.flatten()
    #ax.set_aspect('equal')
    #ax.set_xlim(-10, 10)
    #ax.set_ylim(-10, 10)

    # Generate and plot random test cases
    for i, ax in enumerate(axs):
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
        rect_x = random.uniform(-5, 5)
        rect_y = random.uniform(-5, 5)
        yaw = random.uniform(0, 2*math.pi)
        length = random.uniform(1, 3)
        width = random.uniform(1, 3)
        circle_x = random.uniform(-5, 5)
        circle_y = random.uniform(-5, 5)
        radius = random.uniform(1, 4)
        
        collision = check_collision(rect_x, rect_y, yaw, length, width, circle_x, circle_y, radius)
        
        # Plot rectangle and circle
        plot_rectangle(ax, rect_x, rect_y, yaw, length, width, 'b' if collision else 'g')
        plot_circle(ax, circle_x, circle_y, radius, 'r' if collision else 'y')
        
        # Display collision result
        result_text = "Collision" if collision else "No Collision"
        ax.text(rect_x, rect_y, result_text, horizontalalignment='center')

    plt.show()


