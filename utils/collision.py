import math

def rotate_point_origin(x, y, angle):
    """Rotate a point counterclockwise by a given angle around the origin."""
    x_new = x * math.cos(angle) - y * math.sin(angle)
    y_new = x * math.sin(angle) + y * math.cos(angle)
    return x_new, y_new

def check_collision(rect_x, rect_y, yaw, length, width, circle_x, circle_y, radius):
    # Translate and rotate circle's center to align the rectangle with axes
    translated_x = circle_x - rect_x
    translated_y = circle_y - rect_y
    rotated_x, rotated_y = rotate_point_origin(translated_x, translated_y, -yaw)
    
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

import numpy as np

def rotate_point(cx, cy, angle, px, py):
    """Rotate a point around a given point.
    cx, cy - rotation center, angle - rotation angle, px, py - point coordinates"""
    s = np.sin(angle)
    c = np.cos(angle)
    # translate point back to origin:
    px -= cx
    py -= cy
    # rotate point
    xnew = px * c - py * s
    ynew = px * s + py * c
    # translate point back:
    px = xnew + cx
    py = ynew + cy
    return px, py

def get_rectangle_vertices(cx, cy, yaw, width, height):
    """Get the vertices of a rotated rectangle."""
    corners = [(-width / 2, -height / 2), (width / 2, -height / 2),
               (width / 2, height / 2), (-width / 2, height / 2)]
    vertices = [rotate_point(0, 0, yaw, x, y) for x, y in corners]
    vertices = [(x + cx, y + cy) for x, y in vertices]
    return vertices

def project_polygon(axis, vertices):
    """Project all vertices of a polygon on an axis and return the min and max projection."""
    dots = [np.dot(axis, vertex) for vertex in vertices]
    return min(dots), max(dots)

def axis_overlap(axis, poly1, poly2):
    """Check if the projections of two polygons on an axis overlap."""
    min1, max1 = project_polygon(axis, poly1)
    min2, max2 = project_polygon(axis, poly2)
    return max1 >= min2 and max2 >= min1

def rectangles_overlap(rect1, rect2):
    """Check if two rectangles overlap."""
    vertices1 = get_rectangle_vertices(*rect1)
    vertices2 = get_rectangle_vertices(*rect2)
    
    axes = []
    for rect in [vertices1, vertices2]:
        for i in range(len(rect)):
            # Get the current and next vertex
            p1, p2 = rect[i], rect[(i+1) % len(rect)]
            # Compute the edge vector (p2 - p1)
            edge = np.subtract(p2, p1)
            # Get a perpendicular vector to the edge
            normal = (-edge[1], edge[0])
            axes.append(normal)
    
    for axis in axes:
        if not axis_overlap(axis, vertices1, vertices2):
            return False
    return True

def test_rectangle_2():
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    import numpy as np
    import random
    import math

    # Assuming the check_collision function is defined as above

    def generate_random_rectangle():
        center_x = random.uniform(-3, 3)
        center_y = random.uniform(-3, 3)
        yaw = np.radians(random.uniform(0, 360))
        width = random.uniform(3, 5)
        height = random.uniform(1, 3)
        return (center_x, center_y, yaw, width, height)

    rectangle_pairs = [(generate_random_rectangle(), generate_random_rectangle()) for _ in range(9)]

    fig, axs = plt.subplots(3, 3, figsize=(15, 15))

    for i, ((rect1, rect2), ax) in enumerate(zip(rectangle_pairs, axs.flatten())):
        vertices1 = get_rectangle_vertices(*rect1)
        vertices2 = get_rectangle_vertices(*rect2)
        
        polygon1 = patches.Polygon(vertices1, closed=True, color='blue', alpha=0.5)
        polygon2 = patches.Polygon(vertices2, closed=True, color='red', alpha=0.5)
        
        ax.add_patch(polygon1)
        ax.add_patch(polygon2)
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
        ax.set_aspect('equal')
        
        overlap_result = rectangles_overlap(rect1, rect2)
        ax.set_title(f'Case {i+1}: {"Overlap" if overlap_result else "No Overlap"}')

    plt.tight_layout()
    plt.show()



if __name__ == '__main__':
    test_rectangle_2()


def test_circle_rectangle():
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


