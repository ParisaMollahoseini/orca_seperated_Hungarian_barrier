#!/usr/bin/env python
# coding: utf-8

# In[ ]:


from functools import reduce
from timeit import repeat
from tkinter import EXCEPTION
from matplotlib import pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation
from ast import literal_eval as make_tuple
from math import sqrt
import argparse
import numpy as np
from random import choice
from mayavi import mlab

color_list = [
            (205/255, 7/255, 30/255),
            (255/255, 166/255, 0),
            (0, 72/255, 186/255),
            (135/255, 50/255, 96/255),
            (102/255, 255/255, 0),
            (153/255, 51/255, 0),
            (253/255, 238/255, 0)
            ]
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def read_log(file_name:str):
    f = open(file_name , 'r')
    f_lines = f.read().split('\n')
    f_lines.pop()
    f.close()
    return f_lines


def convert_str2tup(inp:str):
    list_of_tuple = [make_tuple(i) for i in inp.split()[1:]]
    return list_of_tuple

def distance_between(first, second):
      return sqrt((first[0]-second[0]) ** 2 + (first[1]-second[1]) ** 2 + (first[2]-second[2]) ** 2)

def check_collision(all_positions, radius):
    collisions = []
    for i in range(len(all_positions)):
        for j in range(i, len(all_positions)):
            if i == j:
                continue
            distance = distance_between(all_positions[i], all_positions[j])
            if distance <= 2*radius:
                collisions.append((all_positions[i], all_positions[j]))
    return collisions

def create_surface(radius, x, y, z, c_index, agent_num,isObstacle):

    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = radius * np.outer(np.cos(u), np.sin(v)) + x
    y = radius * np.outer(np.sin(u), np.sin(v)) + y
    z = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + z

    # Plot the surface
    if isObstacle:
        shape = mlab.mesh(x, y, z, color=(0,0,0))
    else:
        shape = mlab.mesh(x, y, z, color=color_list[c_index % min(len(color_list), agent_num)])
    return shape 



def main():

    parser = argparse.ArgumentParser(description="Help for run simulator",
                                    formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-i", "--inputfile", help="get input file")
    parser.add_argument("-s", "--interval_speed", help="interval speed", default=1)
    parser.add_argument("-r", "--agent_radius", help="set agent radius", default=5)

    args = parser.parse_args()
    config = vars(args)
    print(config)


    #initialize global vars
    interval_speed = float(config['interval_speed'])
    LOG_FILE = config['inputfile']
    MAIN_LIST = read_log(LOG_FILE)
    RADIUS = float(config['agent_radius'])
    Surface = []


    AGENT_NUMBER = int(MAIN_LIST[0])
    X_MAX = 100
    Y_MAX = 100
    Z_MAX = 100
    X_MIN = -100
    Y_MIN = -100
    Z_MIN = -100




    if AGENT_NUMBER == 0:
        raise EXCEPTION("YOUR NUMBER OF AGENTS IS ZERO !!")
    

    #start plot
    
    fig = mlab.figure(size=(1900,1000),bgcolor=(0.4824, 0.4824, 0.4902),fgcolor=(0,0,0))
    
    first_points = convert_str2tup(MAIN_LIST[1])
    i = 1
    print(i)
    single_log = MAIN_LIST[i]
    print(single_log)
    time = single_log.split()[0]
    states = convert_str2tup(single_log)
    
    

    all_positions = []

    # update properties
    color_ind = 1
    count = 0
    for coordinate in states:
        new_x = coordinate[0]
        new_y = coordinate[1]
        new_z = coordinate[2]
        obst = True if coordinate[3]==2 else False
        
        Surface.append(create_surface(RADIUS, new_x, new_y, new_z, color_ind, AGENT_NUMBER,obst))
        color_ind += 1        
    
    txt = mlab.title('travel time', color=(1,1,1),size=0.5,figure=fig)
    #mlab.axes(figure = fig,xlabel='X', ylabel='Y', zlabel='Z',extent=(X_MIN,X_MAX,Y_MIN,Y_MAX, Z_MIN,Z_MAX),color=(0,0,0)
            # ,line_width=5)
    
    txt.text = 'travel time = {}'.format(time)
    print(txt.text)
    
    @mlab.animate(delay=40)
    def anim(main_list):

        for i in range(2,len(MAIN_LIST)):
            print(i)
            single_log = MAIN_LIST[i]
            print(single_log)
            time = single_log.split()[0]
            states = convert_str2tup(single_log)
            txt.text = 'travel time = {}'.format(time)
            print(txt.text)

            all_positions = []

            # update properties
            color_ind = 1
            count = 0
            for coordinate in states:
                new_x = coordinate[0]
                new_y = coordinate[1]
                new_z = coordinate[2]

                all_positions.append((new_x, new_y, new_z))
                
                
                u = np.linspace(0, 2 * np.pi, 100)
                v = np.linspace(0, np.pi, 100)
                x = RADIUS * np.outer(np.cos(u), np.sin(v)) + new_x
                y = RADIUS * np.outer(np.sin(u), np.sin(v)) + new_y
                z = RADIUS * np.outer(np.ones(np.size(u)), np.cos(v)) + new_z

                Surface[count].mlab_source.reset(x = x,y = y,z = z)
                count += 1
            
            collisions_points = check_collision(all_positions, RADIUS)

            if not len(collisions_points) == 0:
                print(bcolors.FAIL + "--COLLISION NUMBER : {} \nCOLLISION BETWEEN {}--".format(len(collisions_points), collisions_points)+bcolors.ENDC)
                txt.text = ("COLLISTION")   
                
            yield

    # #pause
    # anim.event_source.stop()

    # #unpause
    # anim.event_source.start()

    #save gif
    # writergif = animation.PillowWriter(fps=30) 
    # ani.save('result.gif', writer=writergif)
    anim(MAIN_LIST)
    mlab.show()
    print("THE END...")



if __name__ == "__main__":
    main()


# In[ ]:





# In[ ]:





# In[ ]:




