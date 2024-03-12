import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.interpolate import CubicSpline

from abc import ABCMeta, abstractmethod

# an abstract base class that can be converted to a LLM readable format
class LLMEntry(ABCMeta):
    @abstractmethod
    def describe(self):
        raise NotImplementedError

class Lane(LLMEntry):
    
    def __init__(self):
        self.spline = None

    def describe(self):
        pass
    


class Map:
    pass
    # a map object that contains several lanes

def frenet2cartesian(spline: CubicSpline, s: float, l: float):
    x, y = spline(s)

from utils.cubic_spline_planner import CubicSpline2D

def generate_lane(num_points=3, lane_width=5):
    # Generate a random sequence of 2D coordinates for the lane's centerline
    x = np.linspace(0, 50, num_points)
    y = np.random.uniform(-1, 4, num_points)

    # Create a spline curve for the centerline
    sp = CubicSpline2D(x, y)

    # Return the spline object and lane width
    return sp, lane_width
"""
def generate_two_lanes(num_points=5, lane_width=5, lane_length=80, lane_separation=0):
    # Generate a random sequence of 2D coordinates for the first lane's centerline
    x = np.linspace(0, lane_length, num_points)
    y = np.random.uniform(-2, 2, num_points)
    
    # Create a spline curve for the first lane's centerline
    spline1 = CubicSpline(x, y)
    
    # Create a parallel lane by offsetting the y-coordinates
    spline2 = CubicSpline(x, y + lane_width + lane_separation)  # Offset by lane width and separation

    return spline1, spline2, lane_width
"""
def generate_vehicle_on_lane(splines, lane_width, vehicle_length, vehicle_width, lane_number=1):
    spline = splines[lane_number - 1]  # Choose the spline based on the lane number

    s = 2
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

from utils.stanley_nav import *
def main():


    # Generate two lanes
    spline1, lane_width = generate_lane(num_points=6, lane_width=8)
    splines = [spline1]
    #spline1, lane_width = generate_lane(lane_width=8)
    #splines = [spline1]

    #print(len(spline1.x))

    # Generate a vehicle on a randomly chosen lane
    lane_number = np.random.choice([0])  # Randomly choose lane 1 or 2
    center_x, center_y, vehicle_length, vehicle_width, vehicle_yaw = generate_vehicle_on_lane(splines, lane_width, 4, 2, lane_number)
    
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

    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=0.1)

    target_speed = 30.0 / 3.6  # [m/s]

    max_simulation_time = 100.0

    
    # Initial state
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

    show_animation = True

    while max_simulation_time >= time and last_idx > target_idx:
        ai = pid_control(target_speed, state.v)
        di, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)
        state.update(ai, di)

        time += dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        # Set aspect ratio and labels
        plt.gca().set_aspect('equal', adjustable='box')
        
        if show_animation:  # pragma: no cover
            plt.cla()

            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])

            #visualize_two_lane_vehicle([spline1],lane_width, state.x,state.y, state.yaw, vehicle_length,vehicle_width,,figure=plt)
            # Plot lane centerline and borders
            plt.plot(ax, ay, 'k--')  # Lane centerline
            plt.plot(ax, ay - lane_width / 2, 'b-')  # Lane border
            plt.plot(ax, ay + lane_width / 2, 'b-')  # Lane border
        
            #plt.plot(cx[:target_idx], cy[:target_idx], ".r", label="course")
            #plt.plot(x, y, "-b", label="trajectory")
            #plt.plot(cx[target_idx], cy[target_idx], "xg", label="target")

            #ax.equal()
            # Create and add the rectangle for the vehicle
            rect = draw_rectangle_matplotlib(state.x, state.y, yaw=state.yaw, length=vehicle_length, width=vehicle_width)
            plt.gca().add_patch(rect)
            # for stopping simulation with the esc key.
            
            """
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(cx[target_idx], cy[target_idx], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            
            """
            plt.pause(0.1)

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

    if show_animation:  # pragma: no cover
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


if __name__ == '__main__':
    main()