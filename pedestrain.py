import pygame
import random
import sys

from utils.graphics import draw_rectangle

# Initialize Pygame
pygame.init()

# Screen dimensions
SCREEN_WIDTH, SCREEN_HEIGHT = 900, 600
LANE_WIDTH = 100
LANE_Y = SCREEN_HEIGHT // 2 - LANE_WIDTH // 2

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)

GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

YELLOW = (255, 255, 0)
CYAN = (0, 255, 255)
MAGENTA = (255, 0, 255)

COLOR_ALUMINIUM_0 = pygame.Color(238, 238, 236)
COLOR_ALUMINIUM_1 = pygame.Color(211, 215, 207)
COLOR_ALUMINIUM_2 = pygame.Color(186, 189, 182)
COLOR_ALUMINIUM_3 = pygame.Color(136, 138, 133)
COLOR_ALUMINIUM_4 = pygame.Color(85, 87, 83)
COLOR_ALUMINIUM_4_5 = pygame.Color(66, 62, 64)
COLOR_ALUMINIUM_5 = pygame.Color(46, 52, 54)

# Pedestrian settings
PED_RADIUS = 15
PED_SPEED = 2
SPAWN_RATE = 50  # Lower is more frequent

# Setup screen
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Pedestrian Simulation")

# Main game loop
clock = pygame.time.Clock()
pedestrians = []

# Imagine you are 
from pid import PIDController
from bicycle_model import BicycleModel
import numpy as np

class Vehicle:

    def __init__(self, x, y , yaw, length, width, v):
        #self.position = [x,y]
        #self.yaw = yaw
        self.length = length
        self.width = width
        self.kinematics = BicycleModel(x=x, y=y, theta=yaw, v=v, L=length*0.8)
       
        self.max_throttle=0.75
        self.max_brake=0.3
        self.max_steering=0.8
    
    def x(self):
        return self.kinematics.x   
    
    def y(self):
        return self.kinematics.y   
    
    def yaw(self):
        return self.kinematics.theta  
    
    def control(self, dt, v_f = 0):
        
        v_f = v_f
        pid = PIDController(Kp=1.0, Ki=0.0, Kd=0.0, set_point=v_f)

        acc = pid.get_control(measurement=self.kinematics.v, dt=0.1)
        #print(acc)
        #max_brake = 5
        
        if acc < 0:
            acc = -min(self.max_brake, abs(acc))
        else:
            acc = max(self.max_throttle, acc)
        #print(acc)
        
        self.kinematics.step(delta=0, a=acc, dt=dt)  # steer right and accelerate
        
       
    def render(self,screen):
        x, y = self.kinematics.x, self.kinematics.y
        yaw = self.kinematics.theta
        draw_rectangle(screen, rect=[x*SCALE,y*SCALE,yaw,self.length*SCALE,self.width*SCALE])

Y_SCALE = 30
X_SCALE = 30

SCALE=30

class StraightSidewalk:
    def __init__(self, y, width):
        self.y = y
        self.lane_width = width
        
    def render(self, screen):
        pygame.draw.rect(screen, COLOR_ALUMINIUM_2, (0, (self.y-0.5*self.lane_width)*Y_SCALE, SCREEN_WIDTH, self.lane_width*Y_SCALE))


class StraightLane:
    def __init__(self, y, width):
        self.y = y
        self.lane_width = width
        
    def render(self, screen):
        pygame.draw.rect(screen, COLOR_ALUMINIUM_3, (0, (self.y-0.5*self.lane_width)*Y_SCALE, SCREEN_WIDTH, self.lane_width*Y_SCALE))


class Pedestrian:
    def __init__(self, x, y, radius, v=1.5):
        self.x, self.y = x, y
        self.radius = radius
        self.v = v
        
    def render(self, screen):
        pygame.draw.circle(screen, RED, (int(self.x*X_SCALE), int(self.y*Y_SCALE)), X_SCALE*self.radius)

def spawn_pedestrian():
    x = 15 #random.randint(10, 26)
    y = sidewalk1_y#random.choice([LANE_Y - PED_RADIUS, LANE_Y + LANE_WIDTH + PED_RADIUS])
    #speed = random.random()*PED_SPEED if y > LANE_Y else -PED_SPEED
    pedestrians.append(Pedestrian(x,y,radius=0.5))

def move_pedestrians(dt):
    for ped in pedestrians:
        ped.y += ped.v*dt
        # Remove pedestrian if it goes off screen
        if ped.y < 5 or ped.y > 15:
            pedestrians.remove(ped)

#def draw_lane():
#    pygame.draw.rect(screen, WHITE, (0, LANE_Y, SCREEN_WIDTH, LANE_WIDTH))

def draw_pedestrians(screen):
    for ped in pedestrians:
        #print(ped.x, ped.y)
        ped.render(screen)


def collision_checking():
    pass


#ped_num = 1
#for _ in range(ped_num):
#    spawn_pedestrian()

lane_y = 10
lane_width = 3.3
sidewalk1_width = 1.8
sidewalk1_y = lane_y - lane_width/2 - sidewalk1_width/2


sidewalk2_width = 1.8
sidewalk2_y = lane_y + lane_width/2 + sidewalk2_width/2

lane0 = StraightLane(y=lane_y, width=lane_width)
sidewalk1 = StraightSidewalk(y=sidewalk1_y, width = sidewalk1_width)
sidewalk2 = StraightSidewalk(y=sidewalk2_y, width = sidewalk2_width)

vehicle = Vehicle(x=4,y=10,yaw=0,length=3,width=1.2,v=0)

spawn_pedestrian()

from utils.collision import check_collision
def collision_checking(vehicle):
    for ped in pedestrians: 
        res = check_collision(vehicle.x(), vehicle.y(), vehicle.yaw(), vehicle.length, vehicle.width, ped.x, ped.y, ped.radius)
        if res == True:
            return True


import vidmaker

FPS = 60

recording = False

if recording:
    video = vidmaker.Video("vidmaker.mp4", late_export=True)


import time

running = True

commute_time = 0.0

SIM_STEP = 30

dt = 0.1
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill(BLACK)
    #draw_lane()

    lane0.render(screen)
    sidewalk1.render(screen)
    sidewalk2.render(screen)
    vehicle.render(screen)

    vehicle.control(dt, v_f = 20)
    
    move_pedestrians(dt)
    draw_pedestrians(screen)
    
    if collision_checking(vehicle):
        print("collision")
    
    time.sleep(0.1)
    
    commute_time = commute_time + 1
    if commute_time> SIM_STEP:
        break
    
    # video recording
    if recording:
        video.update(pygame.surfarray.pixels3d(screen).swapaxes(0, 1), inverted=False) # THIS LINE
        
    
    pygame.display.flip()
    clock.tick(60)

if recording:
    video.export(verbose=True)


pygame.quit()
sys.exit()

