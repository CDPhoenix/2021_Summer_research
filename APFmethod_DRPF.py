"""
Potential Field based path planner
author: Atsushi Sakai (@Atsushi_twi)
Ref:
Potential Field based path planner
author: Atsushi Sakai (@Atsushi_twi)

https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/PotentialFieldPlanning/potential_field_planning.py

Modified by: WANG Phoenix
Contact:shadowdouble76@gmail.com

"""

from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import random
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
# Parameters
KP = 5.0 # attractive potential gain
#KP_1 = 2.0
ETA = 500# repulsive potential gain
AREA_WIDTH = 40.0  # potential area width [m]
# the number of previous positions used to check oscillations
OSCILLATIONS_DETECTION_LENGTH = 3
global j
j = 2
rr = 5.0
o_x = []
o_y = []
random.seed(20)
fig, ax = plt.subplots(1,1)
for i in range(20):
    #random.seed(20)
    ox = random.randint(0,30)  # obstacle x position list [m]
    oy = random.randint(0,30)
    o_x.append(ox)
    o_y.append(oy)


show_animation = True


def calc_potential_field(gx, gy,o_x, o_y, reso, rr, sx, sy,j):
    minx = min(min(o_x), sx, gx) - AREA_WIDTH / 2.0
    miny = min(min(o_y), sy, gy) - AREA_WIDTH / 2.0
    maxx = max(max(o_x), sx, gx) + AREA_WIDTH / 2.0
    maxy = max(max(o_y), sy, gy) + AREA_WIDTH / 2.0
    global xw,yw
    xw = int(round((maxx - minx) / reso))
    yw = int(round((maxy - miny) / reso))
    print(xw)
    print(yw)

    # calc each potential
    global pmap
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]

    for ix in range(xw):
        x = ix * reso + minx

        for iy in range(yw):
            y = iy * reso + miny
            ug = calc_attractive_potential(x, y, gx, gy)
            uo = calc_repulsive_potential(x, y, o_x, o_y, rr,j)
            #ug_1 = calc_attractive_potential_1(x,y,gx_1,gy_1)
            uf = ug + uo#+ug_1
            pmap[ix][iy] = uf
        

    return pmap, minx, miny


def calc_attractive_potential(x, y, gx, gy):
    return 0.5 * KP * np.hypot(x - gx, y - gy)
#def calc_attractive_potential_1(x, y, gx_1, gy_1):
#    return 0.5 * KP_1 * np.hypot(x - gx_1, y - gy_1)

def calc_repulsive_potential(x, y, o_x, o_y, rr,j):
    # search nearest obstacle
    minid = -1
    dmin = float("inf")
    for i, _ in enumerate(o_x):
        d = np.hypot(x - o_x[i], y - o_y[i])
        if dmin >= d:
            dmin = d
            minid = i

    # calc repulsive potential
    dq = np.hypot(x - o_x[minid], y - o_y[minid])

    if dq <= rr:
        if dq <= 0.1:
            dq = 0.1

        return 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** j
    else:
        return 0.0


def get_motion_model():
    # dx, dy
    motion = [[1, 0],
              [0, 1],
              [-1, 0],
              [0, -1],
              [-1, -1],
              [-1, 1],
              [1, -1],
              [1, 1]]

    return motion


def oscillations_detection(previous_ids, ix, iy):
    previous_ids.append((ix, iy))

    if (len(previous_ids) > OSCILLATIONS_DETECTION_LENGTH):
        previous_ids.popleft()

    # check if contains any duplicates by copying into a set
    previous_ids_set = set()
    for index in previous_ids:
        if index in previous_ids_set:
            return True
        else:
            previous_ids_set.add(index)
    return False



def potential_field_planning(sx, sy, gx, gy,o_x, o_y, reso, rr,j):

    # calc potential field
    pmap, minx, miny = calc_potential_field(gx, gy,o_x, o_y, reso, rr, sx, sy,j)
    global a
    a = 0
    b = 0
    # search path
    d = np.hypot(sx - gx, sy - gy)
    ix = round((sx - minx) / reso)
    iy = round((sy - miny) / reso)
    gix = round((gx - minx) / reso)
    giy = round((gy - miny) / reso)

    if show_animation:
        draw_heatmap(pmap)
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(ix, iy, "*k")
        if b == 0:
            plt.plot(96, 100, "*m")
    global rx
    rx, ry = [sx], [sy]
    motion = get_motion_model()
    previous_ids = deque()

    while d >= reso:
        a += 1
        minp = float("inf")
        minix, miniy = -1, -1
        for i, _ in enumerate(motion):
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])
            if inx >= len(pmap) or iny >= len(pmap[0]) or inx < 0 or iny < 0:
                p = float("inf")  # outside area
                print("outside potential!")
            else:
                p = pmap[inx][iny]
            if minp > p:
                minp = p
                minix = inx
                miniy = iny
        ix = minix
        iy = miniy
        xp = ix * reso + minx
        yp = iy * reso + miny
        d = np.hypot(gx - xp, gy - yp)
        rx.append(xp)
        ry.append(yp)

        if (oscillations_detection(previous_ids, ix, iy)):
            print("Oscillation detected at ({},{})!".format(ix, iy))
            j += 1
            #rr -= 0.5
            potential_field_planning(xp, yp, gx, gy,o_x, o_y, reso, rr,j)
            break
        if show_animation:
            plt.plot(ix, iy, ".r")
            plt.pause(0.01)
            
    print("Goal!!")

    return rx, ry


def draw_heatmap(data):
    data = np.array(data).T
    plt.pcolor(data,vmax=200.0,cmap=plt.cm.Blues)
    plt.xlabel("x direction (m)")
    plt.ylabel('y direction (m)')
    #fig, ax = plt.subplots(1,1)
    #axins = inset_axes(ax,width="40%",height="30%",loc='lower left',bbox_to_anchor=(0.5,0.1,1,1),
    #                       bbox_transform = ax.transAxes)
    #return ax
#vmax=100.0

def main():
    print("potential_field_planning start")

    sx = 0.0  # start x position [m]
    sy = 10.0  # start y positon [m]
    gx = 30.0  # goal x position [m]
    gy = 30.0  # goal y position [m]
    #gx_1 = 30.0
    #gy_1 = 10.0
    grid_size = 0.5  # potential grid size [m]
      # robot radius [m]

    #ox = [15.0, 5.0, 20.0, 25.0]  # obstacle x position list [m]
    #oy = [25.0, 15.0, 26.0, 25.0]  # obstacle y position list [m]

    if show_animation:
        plt.grid(True)
        plt.axis("equal")

    # path generation
    _, _ = potential_field_planning(
        sx, sy, gx, gy,o_x, o_y, grid_size, rr,j)
    axins = inset_axes(ax,width="50%",height="40%",loc='lower left',bbox_to_anchor=(0.5,0.1,1,1),
                           bbox_transform = ax.transAxes)
    potential_field_planning(0, 10, 30, 30,o_x, o_y, grid_size, rr,j)
    zone_left= 60
    zone_right= 100
    x_ratio = 0.0125
    y_ratio = 0.0125
    xlim0 = zone_left-(zone_right-zone_left)*x_ratio
    xlim1 = zone_right-(zone_right-zone_left)*y_ratio
    y = []
    for i in range(60,120):
        for z in range(0,140):
            y.append(pmap[i][z])
    ylim0 = 75
    ylim1 = 110
    axins.set_xlim(xlim0,xlim1)
    axins.set_ylim(ylim0,ylim1)
    mark_inset(ax,axins,loc1=3,loc2=1,fc="none",ec='k',lw=1)
    plt.xlabel(" ")
    plt.ylabel(' ')
    plt.show()
    if show_animation:
        plt.show()


if __name__ == '__main__':
    print(__file__ + " start!!")
    #fig, ax = plt.subplots(1,1)
    main()
    #axins = inset_axes(ax,width="40%",height="30%",loc='lower left',bbox_to_anchor=(0.5,0.1,1,1),
    #                       bbox_transform = ax.transAxes)
    print(a)
    print(__file__ + " Done!!")
    print("The length of orginal solution path is " + str(len(rx))+'pixels')
    
    