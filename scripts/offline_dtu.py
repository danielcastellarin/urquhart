import os
import sys
import argparse
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import ConnectionPatch

def keypress(event):
    global frameID, numObs
    # print('press', event.key)
    if event.key == "escape":
        plt.close()
    else:
        # plt.clf()   # clear
        for a in ax.flatten(): a.cla()
        if event.key == 'left' and 1 <= frameID-1:  # base-1 this time
            frameID -= 1
        elif event.key == 'right' and frameID+1 < len(globalErrors)+1:
            frameID += 1
        draw_vis()


def get_obs(dirname, frameID):
    polygons = []
    for line in open(f'{dirname}/global/p/{frameID}.txt'):
        polygons.append(zip(*[tuple(map(float, c.split(' '))) for c in line.split('|')[:-1]]))

    triangles = []
    for line in open(f'{dirname}/global/t/{frameID}.txt'):
        triangles.append(zip(*[tuple(map(float, c.split(' '))) for c in line.split('|')[:-1]]))

    trees = [tuple(map(float, [n for n in line.split(' ')[1:] if n != ""])) for line in open(f'{dirname}/global/graph_nodes/{frameID}.txt') if line.split(' ')[0][0] == 'L']

    return polygons, triangles, trees


# def display_triangulation(plt_axis, dirname, frameID):
#     _, triangles, trees = get_obs(dirname, frameID)
#     plt_axis.set_title(f"Frame {frameID} Triangulation")
#     for i,x,y in triangles:
#         plt_axis.fill(x, y, facecolor='none', edgecolor='black', linewidth=2)
#         plt_axis.text(sum(x)/3, sum(y)/3, int(i[0]))
#     for x, y in trees: plt_axis.plot(x, y, 'ro')


# def display_polygons(plt_axis, dirname, frameID):
#     polygons, _, _ = get_obs(dirname, frameID)
#     plt_axis.set_title(f"Frame {frameID} Polygons")
#     for i,x,y in polygons:
#         plt_axis.fill(x, y, facecolor=np.random.rand(3,))
#         plt_axis.text(sum(x)/len(x), sum(y)/len(y), int(i[0]))

def parse_edges(x:list, y:list):
    n = len(x)
    # v = [(x[i], y[i]) for i in range(n)]
    return [(x[i]+x[(i+1)%n], y[i]+y[(i+1)%n]) for i in range(n)]
    # return [(v[i][0]+v[i+1%n][0], ) for i in range(n)]

def dist(x1, y1, x2, y2):
    return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)

def dist2(x1, x2, y1, y2):
    return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)

def find_longest_edge(x:list, y:list):
    n = len(x)
    longest = -1
    edge = None
    for i in range(n):
        x1, y1, x2, y2 = x[i], y[i], x[(i+1)%n], y[(i+1)%n]
        d = dist(x1, y1, x2, y2)
        if d > longest:
            longest = d
            edge = ([x1, x2], [y1, y2])
    return edge

def get_edges(x:list, y:list):
    n = len(x)
    # return [([x[i], x[(i+1)%n]], [y[i], y[(i+1)%n]]) for i in range(n)]
    return [((x[i], x[(i+1)%n]), (y[i], y[(i+1)%n])) for i in range(n)]

def get_longest_edge(e:list):
    longest = -1
    edge = None
    for ex, ey in e:
        d = dist2(*ex, *ey)
        if d > longest:
            longest = d
            edge = (ex, ey)
    return edge


def get_not_longest_edges(x:list, y:list):
    n = len(x)
    e = [([x[i], x[(i+1)%n]], [y[i], y[(i+1)%n]]) for i in range(n)]
    longest = -1
    edge = None
    # for i, (ex, ey) in enumerate(e):
    #     d = dist2(*ex, *ey)
    #     if d > longest:
    #         longest = d
    #         edge = i
    # return [goode for i, goode in enumerate(e) if i != edge]
    for ex, ey in e:
        d = dist2(*ex, *ey)
        if d > longest:
            longest = d
            edge = (ex, ey)
    # return [goode for goode in e if goode != edge]
    revedge = (edge[0][::-1], edge[1][::-1])
    return set(e), set(edge, revedge)
        

def draw_vis():
    ax[0].set_aspect('equal', adjustable='box')
    ax[0].tick_params(left = False, right = False, labelleft = False, labelbottom = False, bottom = False)
    ax[0].set_title("Delaunay Triangulation", fontsize=16, fontweight='bold')
    ax[1].set_aspect('equal', adjustable='box')
    ax[1].tick_params(left = False, right = False, labelleft = False, labelbottom = False, bottom = False)
    ax[1].set_title("Urquhart Graph", fontsize=16, fontweight='bold')
    ax[2].set_aspect('equal', adjustable='box')
    ax[2].tick_params(left = False, right = False, labelleft = False, labelbottom = False, bottom = False)
    ax[2].set_title("Geometric Hierarchy Polygons", fontsize=16, fontweight='bold')
    # fig.tight_layout()

    polygons, triangles, trees = get_obs(directory, frameID)
    
    for i,x,y in polygons:
        ax[2].fill(x, y, facecolor=np.random.rand(3,), edgecolor='black', linewidth=2)

    edges = []
    all_long = set()
    for i,x,y in triangles:
        ax[0].fill(x, y, facecolor='none', edgecolor='black', linewidth=2)

        e = get_edges(x, y)
        edges.extend(e)
        long = get_longest_edge(e)
        revlong = (long[0][::-1], long[1][::-1])
        if long not in all_long and revlong not in all_long:
            all_long.add(long)
        
        # ax[1].fill(x, y, facecolor='none', edgecolor='black', linewidth=2)
        # lx, ly = find_longest_edge(x, y)
        # ax[1].plot(lx, ly, color='white', linewidth=2)

        # for ex, ey in get_not_longest_edges(x,y):
        #     ax[1].plot(ex, ey, color='black', linewidth=2)
    
    # for x,y in all_e:
    #     ax[1].plot(x, y, color='black', linewidth=2)
    
    all_e = set()
    for (x,y) in edges:
        if (x,y) not in all_long and (x[::-1], y[::-1]) not in all_long:
            if (x,y) not in all_e and (x[::-1], y[::-1]) not in all_e:
                ax[1].plot(x, y, color='black', linewidth=2)
                all_e.add((x,y))
    
    for (x,y) in all_long:
        ax[1].plot(x, y, color='red', linewidth=1, linestyle='--')

    
    for x, y in trees: 
        ax[0].plot(x, y, 'bo')
        ax[1].plot(x, y, 'bo')
        ax[2].plot(x, y, 'bo')

    
    fig.canvas.draw()

#######################
# Initialization Script
#######################

parser = argparse.ArgumentParser()
parser.add_argument("dirname", help="The name of the directory with observations")
parser.add_argument("-f", "--frameID", help="Which frame to display", type=int, default=1) # base-1 this time
args = parser.parse_args()

if not os.path.isdir(args.dirname):
    print(f'The given directory "{args.dirname}" does not exist.')
    sys.exit(0)


directory = args.dirname
frameID = args.frameID

# Get all global errors beforehand
globalErrors = [float(val) for val in open(f'{directory}/global/!gError.txt')]

# fig=plt.figure()
fig, ax = plt.subplots(nrows=1, ncols=3)
# fig.set_figheight(4)
# fig.set_figwidth(9)
fig.canvas.mpl_connect('key_press_event', keypress)
draw_vis()
print('Press left and right to view different frames. Press "escape" to exit')
plt.show()
