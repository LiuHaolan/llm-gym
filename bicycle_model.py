import math

class BicycleModel:
    def __init__(self, x=0.0, y=0.0, theta=0.0, v=0.0, L=1.0):
        self.x = x  # X position
        self.y = y  # Y position
        self.theta = theta  # Heading angle in radians
        self.v = v  # Velocity
        self.L = L  # Wheelbase

    def step(self, delta, a, dt):
        """
        Update the state of the bicycle.
        
        Parameters:
        - delta: Steering angle in radians.
        - a: Acceleration.
        - dt: Time step duration.
        """
        # Update velocity
        self.v += a * dt
        
        # Update heading angle
        self.theta += (self.v / self.L) * math.tan(delta) * dt
        self.theta = self.theta % (2 * math.pi)  # Normalize angle to [0, 2*pi]
        
        # Update position
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt

    def get_state(self):
        """
        Return the current state.
        """
        return self.x, self.y, self.v, self.theta

# Example usage

if __name__ == "__main__":
    bicycle = BicycleModel(x=0, y=300, theta=0, v=10, L=1.0)
    #bicycle.step(delta=0.1, a=1, dt=1)  # steer right and accelerate
    #print(bicycle.get_state())
    import pygame
    import random
    import sys
    import time
    import numpy as np
    # Initialize Pygame
    pygame.init()

    # Screen dimensions
    SCREEN_WIDTH, SCREEN_HEIGHT = 800, 600
    LANE_WIDTH = 100
    LANE_Y = SCREEN_HEIGHT // 2 - LANE_WIDTH // 2

    # Colors
    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)
    RED = (255, 0, 0)

    # Pedestrian settings
    PED_RADIUS = 15
    PED_SPEED = 2

    # Setup screen
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Bicycle Model Simulation")

    # Main game loop
    clock = pygame.time.Clock()


    def draw_lane():
        pygame.draw.rect(screen, WHITE, (0, LANE_Y, SCREEN_WIDTH, LANE_WIDTH))

    def draw_bicycle():
        OBJ_RADIUS = 15
        x, y , _, _ = bicycle.get_state()
        pygame.draw.circle(screen, RED, (int(x), int(y)), OBJ_RADIUS)

    max_throttle=0.75
    max_brake=0.3
    max_steering=0.8

    from pid import PIDController
    v_f = 0
    pid = PIDController(Kp=1.0, Ki=0.0, Kd=0.0, set_point=v_f)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill(BLACK)
        draw_lane()

        acc = pid.get_control(measurement=bicycle.v, dt=0.1)
        #print(acc)
        #max_brake = 5
        acc = min(max_brake, abs(acc))*np.sign(acc)
        #print(acc)
        
        bicycle.step(delta=0, a=acc, dt=0.1)  # steer right and accelerate

        draw_bicycle()

        print(bicycle.v)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()
