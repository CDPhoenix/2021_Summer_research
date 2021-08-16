# -*- coding: utf-8 -*-
"""
Created on Sun Aug 15 10:58:42 2021

@author: Administrator
"""

from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import random
import time
# Parameters
KP = 5.0 # attractive potential gain
#KP_1 = 2.0
ETA = 500# repulsive potential gain
AREA_WIDTH = 40.0  # potential area width [m]
# the number of previous positions used to check oscillations
OSCILLATIONS_DETECTION_LENGTH = 3
#global j
#j = 2
rr = 5.0
o_x = []
o_y = []
random.seed(20)
for i in range(20):
    #random.seed(20)
    ox = random.randint(0,30)  # obstacle x position list [m]
    oy = random.randint(0,30)
    o_x.append(ox)
    o_y.append(oy)
print(o_x)
print(o_y)


show_animation = True


def calc_potential_field(gx, gy,o_x, o_y, reso, rr, sx, sy):#,j):
    minx = min(min(o_x), sx, gx) - AREA_WIDTH / 2.0
    miny = min(min(o_y), sy, gy) - AREA_WIDTH / 2.0
    maxx = max(max(o_x), sx, gx) + AREA_WIDTH / 2.0
    maxy = max(max(o_y), sy, gy) + AREA_WIDTH / 2.0
    xw = int(round((maxx - minx) / reso))
    yw = int(round((maxy - miny) / reso))

    # calc each potential
    global pmap
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]

    for ix in range(xw):
        x = ix * reso + minx

        for iy in range(yw):
            y = iy * reso + miny
            ug = calc_attractive_potential(x, y, gx, gy)
            uo = calc_repulsive_potential(x, y, o_x, o_y, rr)#,j)
            #ug_1 = calc_attractive_potential_1(x,y,gx_1,gy_1)
            uf = ug + uo#+ug_1
            pmap[ix][iy] = uf
        

    return pmap, minx, miny


def calc_attractive_potential(x, y, gx, gy):
    return 0.5 * KP * np.hypot(x - gx, y - gy)
#def calc_attractive_potential_1(x, y, gx_1, gy_1):
#    return 0.5 * KP_1 * np.hypot(x - gx_1, y - gy_1)

def calc_repulsive_potential(x, y, o_x, o_y, rr):#,j):
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

        return 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** 2
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


def potential_field_planning(sx, sy, gx, gy,o_x, o_y, reso, rr):#,j):

    # calc potential field
    pmap, minx, miny = calc_potential_field(gx, gy,o_x, o_y, reso, rr, sx, sy)#,j)

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
        plt.plot(gix, giy, "*m")
    global rx
    rx, ry = [sx], [sy]
    motion = get_motion_model()
    previous_ids = deque()

    while d >= reso:
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
            #j += 1
            #rr -= 0.5
            #potential_field_planning(xp, yp, gx, gy,o_x, o_y, reso, rr,j)
            
            
            #从这里开始执行RRT随机逃逸路径搜索
            a = 5
            while True:#搜索开始
                for i in range(a):
                    for j in range(a):#初始的搜索范围，半径为5单位长度
                        plt.plot(ix+i,iy+j,".b")
                        plt.plot(ix+i,iy-j,".b")
                        plt.plot(ix,iy+j,".b")
                        plt.plot(ix,iy-j,".b")
                        plt.plot(ix+i,iy,".b")
                        plt.plot(ix-i,iy,".b")
                        plt.plot(ix-i,iy+j,".b")
                        plt.plot(ix-i,iy-j,".b")                    
                        plt.pause(0.01)#绘制出搜索范围
                        if pmap[ix+i][iy+j] < pmap[ix][iy]:#判断8个方向的搜索结果是否有比该局部最小值更小的势能点
                            xp = (ix+i) * reso + minx
                            yp = (iy+j) * reso + miny
                            potential_field_planning(xp, yp, gx, gy,o_x, o_y, reso, rr)#搜索到结果，重新执行APF路径规划
                            break                     
                        elif pmap[ix-i][iy-j]<pmap[ix][iy]:
                            xp = (ix-i) * reso + minx
                            yp = (iy-j) * reso + miny
                            potential_field_planning(xp, yp, gx, gy,o_x, o_y, reso, rr)
                            break
                        elif pmap[ix+i][iy-j]<pmap[ix][iy]:
                            xp = (ix+i) * reso + minx
                            yp = (iy-j) * reso + miny
                            potential_field_planning(xp, yp, gx, gy,o_x, o_y, reso, rr)
                            break
                            
                        elif pmap[ix-i][iy+j]<pmap[ix][iy]:
                            xp = (ix-i) * reso + minx
                            yp = (iy+j) * reso + miny
                            potential_field_planning(xp, yp, gx, gy,o_x, o_y, reso, rr)
                            break
                            
                        elif pmap[ix][iy+j]<pmap[ix][iy]:
                            xp = (ix) * reso + minx
                            yp = (iy+j) * reso + miny
                            potential_field_planning(xp, yp, gx, gy,o_x, o_y, reso, rr)
                            break
                        elif pmap[ix][iy-j]<pmap[ix][iy]:
                            xp = (ix+i) * reso + minx
                            yp = (iy-j) * reso + miny
                            potential_field_planning(xp, yp, gx, gy,o_x, o_y, reso, rr)
                            break
                            
                        elif pmap[ix+i][iy]<pmap[ix][iy]:
                            xp = (ix+i) * reso + minx
                            yp = (iy) * reso + miny
                            potential_field_planning(xp, yp, gx, gy,o_x, o_y, reso, rr)
                            break
                            
                        elif pmap[ix-i][iy]<pmap[ix][iy]:
                            xp = (ix-i) * reso + minx
                            yp = (iy) * reso + miny
                            potential_field_planning(xp, yp, gx, gy,o_x, o_y, reso, rr)
                            break
                            
                else:
                    a += 1#若没有搜索到，加大检索范围的半径，继续进行随即搜索
            break
        if show_animation:
            plt.plot(ix, iy, ".r")
            plt.pause(0.01)

    print("Goal!!")

    return rx, ry


def draw_heatmap(data):
    data = np.array(data).T
    plt.pcolor(data,vmax=200.0,cmap=plt.cm.Blues)
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
        sx, sy, gx, gy,o_x, o_y, grid_size, rr)#,j)
    if show_animation:
        plt.show()



if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")
    print("The length of orginal solution path is " + str(len(rx))+'pixels')