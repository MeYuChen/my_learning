"""
Hybrid A star
@author: Yu Chen
"""
import os
import sys
import math
import heapq
import time
import numpy as np
import matplotlib.pyplot as plt
import scipy.spatial.kdtree as kd
from heapdict import heapdict
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../MotionPlanning/")

import HybridAstarPlanner.astar as astar
import HybridAstarPlanner.draw as draw
import CurvesGenerator.reeds_shepp as rs

def design_obstacles(x,y):
    ox, oy = [], []

    for i in range(x):
        ox.append(i)
        oy.append(0)
    for i in range(x):
        ox.append(i)
        oy.append(y - 1)
    for i in range(y):
        ox.append(0)
        oy.append(i)
    for i in range(y):
        ox.append(x - 1)
        oy.append(i)
    for i in range(10, 21):
        ox.append(i)
        oy.append(15)
    for i in range(15):
        ox.append(20)
        oy.append(i)
    for i in range(15, 30):
        ox.append(30)
        oy.append(i)
    for i in range(16):
        ox.append(40)
        oy.append(i)

    return ox, oy


class QueuePrior:
    def __init__(self):
        self.queue = heapdict()

    def empty(self):
        return len(self.queue) == 0  # if Q is empty

    def put(self, item, priority):
        self.queue[item] = priority  # push 

    def get(self):
        return self.queue.popitem()[0]  # pop out element with smallest priority

class Para:
    def __init__(self, minx, miny, minyaw, maxx, maxy, maxyaw,
                 xw, yw, yaww, xyreso, yawreso, ox, oy, kdtree):
        self.minx = minx
        self.miny = miny
        self.minyaw = minyaw
        self.maxx = maxx
        self.maxy = maxy
        self.maxyaw = maxyaw
        self.xw = xw
        self.yw = yw
        self.yaww = yaww
        self.xyreso = xyreso
        self.yawreso = yawreso
        self.ox = ox
        self.oy = oy
        self.kdtree = kdtree

class C:
    PI = math.pi

    XY_RESO = 2.0 #[M]
    YAW_RESO = np.deg2rad(15.0) # [rad]
    MOVE_STEP = 0.4
    N_STEER = 20.0
    COLLISION_CHECK_STEP = 5
    EXTEND_BOUND = 1


    GEAR_COST = 100.0
    BACKWARD_COST = 5.0
    STEER_CHANGE_COST = 5.0
    STEER_ANGLE_COST = 1.0
    H_COST = 15.0

    RF = 4.5
    RB = 1.0
    W = 3.0
    WD = 0.7
    WB = 3.5
    TR = 0.5
    TW = 1
    MAX_STEER = 0.6

class Path:
    def __init__(self, x, y, yaw, direction, cost):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.direction = direction
        self.cost = cost

class Node:
    def __init__(self,xindx,yindex,yawindx,direction,x,y,yaw,dirs,steer,cost,pind):
        self.xindx = xindx 
        self.yindex = yindex 
        self.yawindx = yawindx 
        self.direction = direction 
        self.x = x 
        self.y = y 
        self.yaw = yaw 
        self.dirs = dirs 
        self.steer = steer 
        self.cost = cost 
        self.pind = pind 
# utils
def pi_2_pi(theta):
    PI = C.PI
    while theta >  PI:
        theta -= 2.0 * PI

    while theta < -PI:
        theta += 2.0 * PI

    return theta

def is_same_grid(node1, node2):
    if node1.xind != node2.xind or \
            node1.yind != node2.yind or \
            node1.yawind != node2.yawind:
        return False

    return True

def calc_parameters(ox,oy,xyreso,yawreso,kdtree):
    minx = round(min(ox) / xyreso)
    miny = round(min(ox) / xyreso)
    maxx = round(max(ox) / xyreso)
    maxy = round(max(ox) / xyreso)

    xw,yw = maxx = minx,maxy - miny
    minyaw = round( -C.PI /yawreso) -1
    maxyaw = round(C.PI /yawreso)
    yaww = maxyaw - minyaw

    return  Para(minx, miny, minyaw, maxx, maxy, maxyaw,
                xw, yw, yaww, xyreso, yawreso, ox, oy, kdtree)


def calc_motion_set():
    s = np.arange(C.MAX_STEER/C.N_STEER,C.MAX_STEER,C.MAX_STEER/C.N_STEER)
    steer = list(s) + [0.0] + list(-s)
    direc = [1.0 for _ in range(len(steer))]
    + [-1.0 for _ in range(len(steer))]
    steer  = steer + steer
    return steer ,direc

# 计算index 
def calc_index(node,p):
    ind = (node.yawindx - p.minyaw)*p.xw * p.yw+(node.yindex - p.miny)*p.xw + (node.xindx - p.minx)
    return ind

def calc_hybrid_cost(node, hmap, P):
     cost = node.cost + \
           C.H_COST * hmap[node.xind - P.minx][node.yind - P.miny]
     return cost

def calc_rs_path_cost(path):
    cost = 0.0
    for lr in path.lengths:
        if lr >= 0 :
            cost+=1
        else:
            cost+=abs(lr)*C.BACKWARD_COST
    
    for i in range(len(path.lengths)- 1):
        if path.lengths[i]*path.lengths[i+1] < 0.0:
            cost+=C.GEAR_COST
    for ctype in path.ctypes:
        if ctype != "s":
            cost+=C.STEER_ANGLE_COST * abs(C.MAX_STEER)

    nctypes = len(path.ctypes)
    ulist = [0.0 for _ in range(nctypes)]
    
    for i in range(nctypes - 1):
        cost+=C.STEER_CHANGE_COST * abs(ulist[i+1] - ulist[i])
    
    return cost

def is_collision(x, y, yaw, P):
    for ix,iy,iyaw in zip(x,y,yaw):
        d = 1
        d1 = (C.RF - C.RB) /2.0
        r = (C.RF + C.RB) /2.0 + d
        cx = ix + d1*math.cos(iyaw)
        cy = iy + d1*math.sin(iyaw)

        ids = P.kdtree.query_ball_point([cx,cy],r)
        if not ids:
            continue

        for i in ids:
            xo = P.ox[i] - cx
            yo = P.oy[i] - cy
            dx = xo * math.cos(iyaw)+yo*math.sin(iyaw)
            dy = -xo * math.cos(iyaw)+yo*math.sin(iyaw)

            if abs(dx) < r and abs (dy) < C.W / 2 + d:
                return True
    return False

def analystic_expection(node,ngoal,P):
    sx,sy,syaw = node.x[-1],node.y[-1],node.yaw[-1]
    gx,gy,gyaw = ngoal.x[-1],ngoal.y[-1],ngoal.yaw[-1]
    maxc = math.tan(C.MAX_STEER) / C.WB
    paths = rs.calc_all_paths(sx,sy,syaw,gx,gy,gyaw,maxc,step_size=C.MOVE_STEP)

    if not paths:
        return None
    pq = QueuePrior()
    for path in paths :
        # {path,cost}
        pq.put(path,calc_rs_path_cost(path))

    while not pq.empty():
        path = pq.get()
        index = range(0,len(path.x),C.COLLISION_CHECK_STEP)

        pathx = [path.x[k] for k in index]
        pathy = [path.y[k] for k in index]
        pathyaw = [path.yaw[k] for k in index]

        if not is_collision(pathx, pathy, pathyaw, P):
            return path

    return None



def  update_node_with_analystic_expantion(n_curr, ngoal, P):
    path = analystic_expection(n_curr,ngoal,P)

    if not path:
        return False,None
    fx = path.x[1:-1]
    fy = path.y[1:-1]
    fyaw = path.yaw[1:-1]
    fd = path.directions[1:-1]

    fcost = n_curr.cost + calc_rs_path_cost(path)
    fpind = calc_index(n_curr,P)
    fsteer = 0.0
    fpath = Node(n_curr.xind, n_curr.yind, n_curr.yawind, n_curr.direction,
                 fx, fy, fyaw, fd, fsteer, fcost, fpind)
    return True,fpath

def calc_next_node(n_curr, c_id, u, d, P):
    step = C.XY_RESO * 2
    nlist = math.ceil(step / C.MOVE_STEP)
    xlist = [n_curr.x[-1]] + d* C.MOVE_STEP * math.cos(n_curr.yaw[-1])
    ylist = [n_curr.y[-1]] + d* C.MOVE_STEP * math.sin(n_curr.yaw[-1])
    yawlist = [pi_2_pi(n_curr.yaw[-1] + d * C.MOVE_STEP /C.WB* math.tan(u))]
    for i in range(nlist -1 ):
        xlist.append(xlist[i] + d * C.MOVE_STEP * math .cos(yawlist[i]))
        ylist.append(ylist[i] + d * C.MOVE_STEP * math .sin(yawlist[i]))
        yawlist.append(pi_2_pi( yawlist[i]) + d * C.MOVE_STEP /C.WB* math .tan(u))
    xind = round(xlist[-1]/P.xyreso)
    yind = round(ylist[-1]/P.xyreso)
    yawind = round(yawlist[-1]/P.yawreso)
  
    if not is_index_ok(xind, yind, xlist, ylist, yawlist, P):
        return None
    
    cost = 0.0

    if d > 0:

        dirsction = 1 
        cost+=abs(step)
    else:
        direction = -1
        cost+=abs(step) * C.BACKWARD_COST
    
    if direction != n_curr.direction : 
        cost+=C.GEAR_COST
    cost += C.STEER_ANGLE_COST * abs(u) 
    cost+= C.STEER_CHANGE_COST * abs(n_curr.steer - u)
    cost = n_curr.cost + cost
    directions = [direction for _ in range(len(xlist))]

    node = Node(xind, yind, yawind, direction, xlist, ylist,
                yawlist, directions, u, cost, c_id)
    
    return node
def is_index_ok(xind, yind, xlist, ylist, yawlist, P):
    if xind <= P.minx or \
        xind >= P.maxx or \
        yind <= P.miny or \
        yind >= P.maxy:
        return False
    ind = range(0, len(xlist), C.COLLISION_CHECK_STEP)

    nodex = [xlist[k] for k in ind]
    nodey = [ylist[k] for k in ind]
    nodeyaw = [yawlist[k] for k in ind]

    if is_collision(nodex, nodey, nodeyaw, P):
        return False

    return True


def hybrid_astar_planning(sx,sy,syaw0,gx,gy,gyaw0,ox,oy,xyreso,yawreso):
    sxr,syr = round(sx/xyreso),round(sy/xyreso)
    gxr,gyr = round(gx/xyreso),round(gy/xyreso)
    syawr = round(pi_2_pi(syaw0)/yawreso)
    gyawr = round(pi_2_pi(gyaw0)/yawreso)

    nstart = Node(sxr,syr,syawr,1,[sx],[sy],[syaw0],[1],0.0,0.0,-1)
    ngoal = Node(gxr,gyr,gyawr,1,[gx],[gy],[gyaw0],[1],0.0,0.0,-1)
    
    kdtree = kd.KDTree([[x,y]for x,y in zip(ox,oy)])
    P = calc_parameters(ox,oy,xyreso,yawreso,kdtree)

    hmap = astar.calc_holonomic_heuristic_with_obstacle(ngoal,P.ox,P.oy,P.xyreso,1.0)

    steer_set,direc_set = calc_motion_set()
    # {key,value}->(index,node)
    open_set, close_set = {calc_index(nstart, P): nstart}, {}

    qp = QueuePrior()
    # {index,cost}
    qp.put(calc_index(nstart,P),calc_hybrid_cost(nstart,hmap,P))
    
    while True:
        if not open_set:
            return None
        index = qp.get()
        curr_node = open_set[index]
        close_set[index] = curr_node
        open_set.pop(index)

        update,fpath =  update_node_with_analystic_expantion(curr_node, ngoal, P)

        if update:
            fnode = fpath
            break

        for i in range(len(steer_set)):
            node =  calc_next_node(curr_node, index, steer_set[i], direc_set[i], P)

            if not node:
                continue
            node_index = calc_index(node,P)
            if node_index in close_set:
                continue
            if node_index not in open_set:
                open_set[node_index]  = node
                qp.put(node_index,calc_hybrid_cost(node,hmap,P))

    return extract_path(close_set,fnode,nstart)
        

def extract_path(closed, ngoal, nstart):
    rx, ry, ryaw, direc = [], [], [], []
    cost = 0.0
    node = ngoal

    while True:
        rx += node.x[::-1]
        ry += node.y[::-1]
        ryaw += node.yaw[::-1]
        direc += node.directions[::-1]
        cost += node.cost

        if is_same_grid(node, nstart):
            break

        node = closed[node.pind]

    rx = rx[::-1]
    ry = ry[::-1]
    ryaw = ryaw[::-1]
    direc = direc[::-1]

    direc[0] = direc[1]
    path = Path(rx, ry, ryaw, direc, cost)

    return path



def draw_car(x, y, yaw, steer, color='black'):
    car = np.array([[-C.RB, -C.RB, C.RF, C.RF, -C.RB],
                    [C.W / 2, -C.W / 2, -C.W / 2, C.W / 2, C.W / 2]])

    wheel = np.array([[-C.TR, -C.TR, C.TR, C.TR, -C.TR],
                      [C.TW / 4, -C.TW / 4, -C.TW / 4, C.TW / 4, C.TW / 4]])

    rlWheel = wheel.copy()
    rrWheel = wheel.copy()
    frWheel = wheel.copy()
    flWheel = wheel.copy()

    Rot1 = np.array([[math.cos(yaw), -math.sin(yaw)],
                     [math.sin(yaw), math.cos(yaw)]])

    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    frWheel = np.dot(Rot2, frWheel)
    flWheel = np.dot(Rot2, flWheel)

    frWheel += np.array([[C.WB], [-C.WD / 2]])
    flWheel += np.array([[C.WB], [C.WD / 2]])
    rrWheel[1, :] -= C.WD / 2
    rlWheel[1, :] += C.WD / 2

    frWheel = np.dot(Rot1, frWheel)
    flWheel = np.dot(Rot1, flWheel)

    rrWheel = np.dot(Rot1, rrWheel)
    rlWheel = np.dot(Rot1, rlWheel)
    car = np.dot(Rot1, car)

    frWheel += np.array([[x], [y]])
    flWheel += np.array([[x], [y]])
    rrWheel += np.array([[x], [y]])
    rlWheel += np.array([[x], [y]])
    car += np.array([[x], [y]])

    plt.plot(car[0, :], car[1, :], color)
    plt.plot(frWheel[0, :], frWheel[1, :], color)
    plt.plot(rrWheel[0, :], rrWheel[1, :], color)
    plt.plot(flWheel[0, :], flWheel[1, :], color)
    plt.plot(rlWheel[0, :], rlWheel[1, :], color)
    draw.Arrow(x, y, yaw, C.WB * 0.8, color)




def main():
    print("start ")
    x,y = 51,31
    sx,sy,syaw0 =10.0, 7.0 ,np.deg2rad(120.0)
    gx,gy,gyaw0 = 45.0,20.0,np.deg2rad(90.0)

    ox,oy = design_obstacles(x,y)

    t0 = time.time()
    path = hybrid_astar_planning(sx,sy,syaw0,gx,gy,gyaw0,ox,oy,C.XY_RESO,C.YAW_RESO)
    t1 = time.time()
    print("running T: ", t1 - t0)

    if not path:
        print("Searching failed!")
        return

    x = path.x
    y = path.y
    yaw = path.yaw
    direction = path.direction

    for k in range(len(x)):
        plt.cla()
        plt.plot(ox, oy, "sk")
        plt.plot(x, y, linewidth=1.5, color='r')

        if k < len(x) - 2:
            dy = (yaw[k + 1] - yaw[k]) / C.MOVE_STEP
            steer = rs.pi_2_pi(math.atan(-C.WB * dy / direction[k]))
        else:
            steer = 0.0

        draw_car(gx, gy, gyaw0, 0.0, 'dimgray')
        draw_car(x[k], y[k], yaw[k], steer)
        plt.title("Hybrid A*")
        plt.axis("equal")
        plt.pause(0.0001)

    plt.show()
    print("Done!")
        

if __name__=='__main__':
    main()