import pygame

def draw_rectangle(screen, rect, color=(255,0,0)):
    rect_color = color  # Red color
    
    # rect is a 5-element tuple featuring (x,y,yaw,width,height)
    rect_width, rect_height = rect[3], rect[4]  # Size
    rect_x, rect_y = rect[0], rect[1]  # Center position
    rect_yaw = rect[2]  # Rotation angle in degrees


    # Create a new surface for the rectangle (with per-pixel alpha)
    rect_surface = pygame.Surface((rect_width, rect_height), pygame.SRCALPHA)
    pygame.draw.rect(rect_surface, rect_color, (0, 0, rect_width, rect_height))

    # Rotate the rectangle surface
    rotated_surface = pygame.transform.rotate(rect_surface, -rect_yaw)  # Pygame rotates counterclockwise
    rotated_rect = rotated_surface.get_rect(center=(rect_x, rect_y))

    # Blit the rotated surface onto the screen
    screen.blit(rotated_surface, rotated_rect.topleft)

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

def draw_rectangle_matplotlib(x, y, yaw, length, width):
    # Calculate corner points in local coordinate system before rotation
    corner_offsets = np.array([
        [-length / 2, -width / 2],
        [-length / 2, width / 2],
        [length / 2, width / 2],
        [length / 2, -width / 2],
    ])
    
    # Rotation matrix based on yaw angle
    rotation_matrix = np.array([
        [np.cos(yaw), -np.sin(yaw)],
        [np.sin(yaw), np.cos(yaw)]
    ])
    
    # Rotate corners and translate to global position
    corners_rotated_and_translated = np.dot(rotation_matrix, corner_offsets.T).T + np.array([x, y])
    
    # Create a Polygon patch
    rectangle = Polygon(corners_rotated_and_translated, closed=True, edgecolor='r', facecolor='none')
    return rectangle
    # Add patch to current axes
    # plt.gca().add_patch(rectangle)
    #plt.axis('equal')  # Ensure x and y have the same scale for proper aspect ratio
