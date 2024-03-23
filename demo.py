import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.interpolate import CubicSpline

from abc import ABCMeta, abstractmethod

from utils.cubic_spline_planner import CubicSpline2D

# an abstract base class that can be converted to a LLM readable format
class LLMEntry(ABCMeta):
    @abstractmethod
    def describe(self):
        raise NotImplementedError

from collections import deque

from langchain.tools import (
    BaseTool,
    StructuredTool,
    tool,
)

class Vehicle:

    DECELERATION_DELTA = 2
    ACELERATION_DELTA = 2

    def __init__(self, center_x, center_y, vehicle_length, vehicle_width, vehicle_yaw, target_speed = 30.0 / 3.6 ):
        #self.center_x = center_x
        #self.center_y = center_y
        #self.yaw = vehicle_yaw
        self.state = State(x=center_x, y=center_y, yaw=vehicle_yaw, v=0.0)
        self.vehicle_length = vehicle_length
        self.vehicle_width = vehicle_width
        self.target_speed =  target_speed
        # a buffered reference line.
        self.buffered_traj = deque(maxlen=5)


    # generate a course to follow, given (ax, ay)
    def plan(self, ax, ay):
            # generating a reference line to track
        self.cx, self.cy, self.cyaw, ck, s = cubic_spline_planner.calc_spline_course(
            ax, ay, ds=0.1)
        self.target_idx, _ = calc_target_index(self.state, self.cx, self.cy)
        self.last_idx = len(self.cx) - 1

    def plannable(self):
        return self.last_idx > self.target_idx

    def act(self, dt):
        ai = pid_control(self.target_speed, self.state.v)
        di, self.target_idx = stanley_control(self.state, self.cx, self.cy, self.cyaw, self.target_idx)
        
        # buffered traj
        self.buffered_traj.append((self.state.x, self.state.y))
        self.state.update(ai, di)


    def describe(self):
        return """

        """







def generate_lane(num_points=3, lane_width=5):
    # Generate a random sequence of 2D coordinates for the lane's centerline
    x = np.linspace(0, 50, num_points)
    y = np.random.uniform(-1, 1, num_points)

    # Create a spline curve for the centerline
    sp = CubicSpline2D(x, y)

    # Return the spline object and lane width
    return sp, lane_width

def generate_two_lanes(num_points=5, lane_width=5, lane_length=100, lane_separation=0):
    # Generate a random sequence of 2D coordinates for the first lane's centerline
    x = np.linspace(0, lane_length, num_points)
    y = np.random.uniform(-1, -0.5, num_points)
    
    # Create a spline curve for the first lane's centerline
    spline1 = CubicSpline2D(x, y)
    # Create a parallel lane by offsetting the y-coordinates
    spline2 = CubicSpline2D(x, y + lane_width + lane_separation)  # Offset by lane width and separation

    return spline1, spline2, lane_width

def generate_vehicle_on_lane(spline, vehicle_length, vehicle_width, s):
    #spline = splines[0]  # Choose the spline based on the lane number

    #print(lane_number)
    #s = 2
    yaw = spline.calc_yaw(s)#+np.pi/2

    center_x, center_y = spline.calc_position(s)
    return center_x, center_y, vehicle_length, vehicle_width, yaw

from utils.graphics import draw_rectangle_matplotlib
   
def visualize_two_lane_vehicle(splines, lane_width, center_x, center_y, vehicle_yaw, vehicle_length, vehicle_width):
    fig, _  = plt.subplots()
    ax = plt.gca()

    # same
    #ax.set_aspect('equal', adjustable='box')

    for spline in splines:
        # Generate points along the spline for visualization
        s = np.linspace(0, 50, 500)
        x = np.array([0.0]*500)
        y = np.array([0.0]*500)
        for id, s_i in enumerate(s):
            x[id], y[id] = spline.calc_position(s_i)

        # Plot lane centerline and borders
        plt.plot(x, y, 'k--')  # Lane centerline
        plt.plot(x, y - lane_width / 2, 'b-')  # Lane border
        plt.plot(x, y + lane_width / 2, 'b-')  # Lane border
    
    plt.axis('equal')
    # Create and add the rectangle for the vehicle
    rect = draw_rectangle_matplotlib(center_x , center_y, yaw=vehicle_yaw,length=vehicle_length, width=vehicle_width)
    ax.add_patch(rect)

    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.show()

def render_plt(splines, vehicle_list, lane_width):
    plt.cla()

    plt.gcf().canvas.mpl_connect('key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

    #visualize_two_lane_vehicle([spline1],lane_width, state.x,state.y, state.yaw, vehicle_length,vehicle_width,,figure=plt)
    # Plot lane centerline and borders
    for spline in splines:
        interval = 30
        s = np.linspace(0, 50, interval)
        ax = np.array([0.0]*interval)
        ay = np.array([0.0]*interval)
        for id, s_i in enumerate(s):
            ax[id], ay[id] = spline.calc_position(s_i)

        plt.plot(ax, ay, 'k--')  # Lane centerline
        plt.plot(ax, ay - lane_width / 2, 'b-')  # Lane border
        plt.plot(ax, ay + lane_width / 2, 'b-')  # Lane border
    
    #plt.plot(cx[:target_idx], cy[:target_idx], ".r", label="course")
    #plt.plot(x, y, "-b", label="trajectory")
    #plt.plot(cx[target_idx], cy[target_idx], "xg", label="target")

    #ax.equal()
    # Create and add the rectangle for the vehicle
    #rect = draw_rectangle_matplotlib(state.x, state.y, yaw=state.yaw, length=vehicle_length, width=vehicle_width)
    for v in vehicle_list:
        rect = draw_rectangle_matplotlib(v.state.x, v.state.y, yaw=v.state.yaw, length=v.vehicle_length, width=v.vehicle_width)
        
        plt.gca().add_patch(rect)
    # for stopping simulation with the esc key.
    
    plt.ylim((-5, 15))

    """
    plt.plot(cx, cy, ".r", label="course")
    plt.plot(x, y, "-b", label="trajectory")
    plt.plot(cx[target_idx], cy[target_idx], "xg", label="target")
    plt.axis("equal")
    plt.grid(True)
    plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
    
    """
    
from utils.collision import rectangles_overlap
from utils.stanley_nav import *
def lane_changing():

    recording = False
    if recording:
        list_of_im_paths = []

    # Generate two lanes
    #spline1, lane_width = generate_lane(num_points=6, lane_width=8)
    #splines = [spline1]
    lane_length = 50
    spline1, spline2, lane_width = generate_two_lanes(num_points=6, lane_width=5, lane_length=200)
    splines = [spline1, spline2]

    #print(len(spline1.x))

    # Generate a vehicle on a randomly chosen lane
    #lane_number = np.random.choice([0])  # Randomly choose lane 1 or 2
    lane_number = 1
    center_x, center_y, vehicle_length, vehicle_width, vehicle_yaw = generate_vehicle_on_lane(spline1, vehicle_length=4, vehicle_width=2, s=2)#(splines, lane_width, 4, 2, lane_number)
    

    #print(center_y)
    #visualize_two_lane_vehicle(splines, lane_width, center_x, center_y, vehicle_yaw, vehicle_length, vehicle_width)

    show_animation = True
    """Plot an example of Stanley steering control on a cubic spline."""
    #  target course
    interval = 30
    s = np.linspace(0, 50, interval)
    ax = np.array([0.0]*interval)
    ay = np.array([0.0]*interval)
    for id, s_i in enumerate(s):
        ax[id], ay[id] = spline1.calc_position(s_i)
    
    #ax = spline1
    #ay = [0.0, -8.0, -25.0, -15.0, -10.0]
    
    vehicle_list = []

    v0 = Vehicle(center_x, center_y, vehicle_length, vehicle_width, vehicle_yaw, target_speed=20/3.6)



    v0.plan(ax, ay)
    # ego vehicle is always in the first element
    vehicle_list.append(v0)


    center_x, center_y, vehicle_length, vehicle_width, vehicle_yaw = generate_vehicle_on_lane(spline2, vehicle_length=4, vehicle_width=2, s=0)#(splines, lane_width, 4, 2, lane_number)
    v1 = Vehicle(center_x, center_y, vehicle_length, vehicle_width, vehicle_yaw)
    interval = 30
    s = np.linspace(0, 50, interval)
    ax = np.array([0.0]*interval)
    ay = np.array([0.0]*interval)
    for id, s_i in enumerate(s):
        ax[id], ay[id] = spline2.calc_position(s_i)
    v1.plan(ax, ay)
    vehicle_list.append(v1)

    # generating a reference line to track
    # cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
    #    ax, ay, ds=0.1)

    target_speed = 30.0 / 3.6  # [m/s] #

    #max_simulation_time = 6.0

    time = 0.0
    # Initial state
    """
    x,y,yaw = center_x, center_y, vehicle_yaw
    state = State(x=x, y=y, yaw=yaw, v=0.0)

    last_idx = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_idx, _ = calc_target_index(state, cx, cy)
    """

    iter_n = 0
    max_iter = 60
    while iter_n <= max_iter and v0.plannable():
        """
        ai = pid_control(target_speed, state.v)
        di, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)
        state.update(ai, di)
        """
        # 
        iter_n = iter_n + 1
        if iter_n == 14:
            interval = 30
            s = np.linspace(0, 50, interval)
            ax = np.array([0.0]*interval)
            ay = np.array([0.0]*interval)
            for id, s_i in enumerate(s):
                ax[id], ay[id] = spline2.calc_position(s_i)
            v0.plan(ax, ay)

        # control for every agents
        for v in vehicle_list:
            v.act(dt)


        # collision checking
         # collision checking
        for v in vehicle_list[1:]:
            rect_ego = (v0.state.x, v0.state.y, v0.state.yaw, v0.vehicle_length, v0.vehicle_width)
            rect_v = (v.state.x, v.state.y, v.state.yaw, v.vehicle_length, v.vehicle_width)

            if rectangles_overlap(rect_ego, rect_v):
                print("Collision detected")
                return False

        time += dt

        """
        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        """
        # Set aspect ratio and labels
        plt.gca().set_aspect('equal', adjustable='box')
        
        if show_animation:  # pragma: no cover
            render_plt(splines, vehicle_list, lane_width)
            if recording:
                filename = "png/{}.png".format(iter_n)
                plt.savefig(filename)
                list_of_im_paths.append(filename)
                plt.pause(0.0001)   
            else:
                plt.pause(0.01)    

    if recording:
        # concat to gif
        import imageio
        #list_of_im_paths = 
        ims = [imageio.imread(f) for f in list_of_im_paths]
        imageio.mimwrite("./demo.gif", ims)

    print("task done\n")
    return True
    """
    if not show_animation:
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(cx, cy, "-b", label="course")
        plt.plot(x, y, ".r", label="trajectory")
        #plt.plot(cx[target_idx], cy[target_idx], "xg", label="target")

        plt.scatter(ax[0],ay[0], s=160, c='forestgreen', marker=(5, 1))  #creates plot with star marker
        plt.scatter(ax[-1],ay[-1],s=160, c='mediumseagreen', marker=(5, 1))  #creates plot with star marker


        plt.axis("equal")
        plt.grid(True)
        plt.show()
        #plt.pause(0.001)

    # Test
    assert last_idx >= target_idx, "Cannot reach goal"

    if show_animation:  #trajdata pragma: no cover
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        plt.subplots(1)
        plt.plot(t, [iv * 3.6 for iv in v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()

        """

def lane_following(show_animation = False):

    recording = False
    if recording:
        list_of_im_paths = []

    lane_length = 50
    spline1, spline2, lane_width = generate_two_lanes(num_points=6, lane_width=5, lane_length=200)
    splines = [spline1, spline2]

    center_x, center_y, vehicle_length, vehicle_width, vehicle_yaw = generate_vehicle_on_lane(spline1, vehicle_length=4, vehicle_width=2, s=2)#(splines, lane_width, 4, 2, lane_number)
    

    #print(center_y)
    #visualize_two_lane_vehicle(splines, lane_width, center_x, center_y, vehicle_yaw, vehicle_length, vehicle_width)

    
    """Plot an example of Stanley steering control on a cubic spline."""
    #  target course
    interval = 30
    s = np.linspace(0, 50, interval)
    ax = np.array([0.0]*interval)
    ay = np.array([0.0]*interval)
    for id, s_i in enumerate(s):
        ax[id], ay[id] = spline1.calc_position(s_i)
    
    vehicle_list = []

    v0 = Vehicle(center_x, center_y, vehicle_length, vehicle_width, vehicle_yaw, target_speed=20/3.6)


    v0.plan(ax, ay)
    vehicle_list.append(v0)


    center_x, center_y, vehicle_length, vehicle_width, vehicle_yaw = generate_vehicle_on_lane(spline1, vehicle_length=4, vehicle_width=2, s=10)#(splines, lane_width, 4, 2, lane_number)
    v1 = Vehicle(center_x, center_y, vehicle_length, vehicle_width, vehicle_yaw , target_speed=20/3.6)
    interval = 30
    s = np.linspace(0, 50, interval)
    ax = np.array([0.0]*interval)
    ay = np.array([0.0]*interval)
    for id, s_i in enumerate(s):
        ax[id], ay[id] = spline1.calc_position(s_i)
    v1.plan(ax, ay)
    vehicle_list.append(v1)


    target_speed = 30.0 / 3.6  # [m/s] #

    #max_simulation_time = 6.0

    time = 0.0

    iter_n = 0
    max_iter = 75
    while iter_n <= max_iter and v0.plannable():
        """
        ai = pid_control(target_speed, state.v)
        di, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)
        state.update(ai, di)
        """
        # 
        iter_n = iter_n + 1
        if False:#iter_n == 14:
            
            interval = 30
            s = np.linspace(0, 50, interval)
            ax = np.array([0.0]*interval)
            ay = np.array([0.0]*interval)
            for id, s_i in enumerate(s):
                ax[id], ay[id] = spline2.calc_position(s_i)
            v0.plan(ax, ay)

        # control for every agents
        for v in vehicle_list:
            v.act(dt)

        # collision checking
        for v in vehicle_list[1:]:
            rect_ego = (v0.state.x, v0.state.y, v0.state.yaw, v0.vehicle_length, v0.vehicle_width)
            rect_v = (v.state.x, v.state.y, v.state.yaw, v.vehicle_length, v.vehicle_width)

            if rectangles_overlap(rect_ego, rect_v):
                print("Collision detected")
                return False

        time += dt

        # Set aspect ratio and labels
        plt.gca().set_aspect('equal', adjustable='box')
        
        if show_animation:  # pragma: no cover
            render_plt(splines, vehicle_list, lane_width)
            if recording:
                filename = "png/{}.png".format(iter_n)
                plt.savefig(filename)
                list_of_im_paths.append(filename)
                plt.pause(0.0001)   
            else:
                plt.pause(0.01)    

    if recording:
        # concat to gif
        import imageio
        #list_of_im_paths = 
        ims = [imageio.imread(f) for f in list_of_im_paths]
        imageio.mimwrite("./demo.gif", ims)

    print("task done\n")
    return True

if __name__ == '__main__':
    lane_following(show_animation = True)
    #lane_changing()#show_animation=True)

    #from llm_gym.map import 