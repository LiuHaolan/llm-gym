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

