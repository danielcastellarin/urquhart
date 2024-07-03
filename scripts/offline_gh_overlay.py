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
        plt.clf()   # clear
        # for a in ax.flatten(): a.cla()
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
    # trees = []
    # for line in open(f'{dirname}/graph_nodes/{frameID}.txt'):
    #     if line.split(' ')[0][0] == 'L':
    #         print(line.split(' ')[1:])
    #         trees.append(tuple(map(float, [n for n in line.split(' ')[1:] if n != ""])))
    return polygons, triangles, trees


def display_triangulation(plt_axis, dirname, frameID):
    _, triangles, trees = get_obs(dirname, frameID)
    plt_axis.set_title(f"Frame {frameID} Triangulation")
    for i,x,y in triangles:
        plt_axis.fill(x, y, facecolor='none', edgecolor='black', linewidth=2)
        plt_axis.text(sum(x)/3, sum(y)/3, int(i[0]))
    for x, y in trees: plt_axis.plot(x, y, 'ro')


def display_polygons(plt_axis, dirname, frameID):
    polygons, _, _ = get_obs(dirname, frameID)
    plt_axis.set_title(f"Frame {frameID} Polygons")
    for i,x,y in polygons:
        plt_axis.fill(x, y, facecolor=np.random.rand(3,))
        plt_axis.text(sum(x)/len(x), sum(y)/len(y), int(i[0]))

def draw_vis():
    polygons, triangles, trees = get_obs(directory, frameID)

    for i,x,y in polygons:
        fig.gca().fill(x, y, facecolor=np.random.rand(3,))

    for i,x,y in triangles:
        fig.gca().fill(x, y, facecolor='none', edgecolor='black', linewidth=2)
    
    # for x, y in trees: fig.gca().plot(x, y, 'ro')
    
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

fig=plt.figure()
fig.set_tight_layout(True)
# fig, ax = plt.subplots(nrows=2, ncols=1)
# fig.set_figheight(9)
# fig.set_figwidth(4)
fig.canvas.mpl_connect('key_press_event', keypress)
draw_vis()
print('Press left and right to view different frames. Press "escape" to exit')
plt.show()
