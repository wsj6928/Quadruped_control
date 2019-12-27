import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

######################### you can change these values ########################
link1 = 0.3  # link length setting
link2 = 0.3
th1 = 45.0
th2 = 45.0
##############################################################################

# value initialize

fr = []
fl = []
br = []
bl = []
frth = []
flth = []
brth = []
blth = []


def linkshow(bodypoint, feetpoint):
    bx = bodypoint[0]
    by = bodypoint[1]
    fx = feetpoint[0]
    fy = feetpoint[1]
    x = bx - fx
    y = by - fy

    th2 = np.arccos(((x**2)+(y**2)-(link1**2)-(link2**2))/(2*link1*link2))
    th2_i = -th2
    th1 = np.arctan2(y, x)-np.arccos(((x**2)+(y**2)+(link1**2) -
                                      (link2**2)) / (2*link1*math.sqrt(x**2+y**2)))

    joint1 = bodypoint
    joint2 = [joint1[0]-link1*np.cos(th1), joint1[1]-link1*np.sin(th1)]
    joint3 = feetpoint

    leg = []
    th1n = th1 * 180.0 / math.pi 
    th2n = th2 * 180.0 / math.pi 
    leg.append(th1n)
    leg.append(th2n)

    return [joint1,joint2,joint3]

def inclination(frontpoint, backpoint, angle):
    fx = frontpoint[0]
    fy = frontpoint[1]
    bx = backpoint[0]
    by = backpoint[1]
    cx = (fx + bx)/2.0
    cy = (fy + by)/2.0
    r = math.sqrt(((fx - bx)**2.0 + (fy - by)**2.0))/2.0

    tfx = cx + r*math.cos((angle/180.0)*math.pi)
    tfy = cy + r*math.sin((angle/180.0)*math.pi)
    tbx = cx - r*math.cos((angle/180.0)*math.pi)
    tby = cy - r*math.sin((angle/180.0)*math.pi)

    return [tfx,tfy],[tbx,tby] 

def inclination_from_front(frontpoint, angle):
    fx = frontpoint[0]
    fy = frontpoint[1]
    r = 0.75

    tfx = fx
    tfy = fy
    tbx = fx - r*math.cos((angle/180.0)*math.pi)
    tby = fy - r*math.sin((angle/180.0)*math.pi)

    return [tfx,tfy],[tbx,tby]     

def inclination_from_back(backpoint, angle):
    bx = backpoint[0]
    by = backpoint[1]
    r = 0.75

    tfx = bx + r*math.cos((angle/180.0)*math.pi)
    tfy = by + r*math.sin((angle/180.0)*math.pi)
    tbx = bx
    tby = by

    return [tfx,tfy],[tbx,tby]   

def circletraj(startpoint, endpoint, step, i):
    sx = startpoint[0]
    sy = startpoint[1]
    ex = endpoint[0]
    ey = endpoint[1]
    cx = (ex + sx)/2.0
    cy = (ey + sy)/2.0
    r = math.sqrt(((ex - sx)**2.0 + (ey - sy)**2.0))/2.0

    theta = math.atan((ey-sy)/(ex-sx))
    astep = math.pi/step
    stepangle = math.pi + theta - astep * i

    tx = cx + r*math.cos(stepangle)
    ty = cy + r*math.sin(stepangle)

    forprint = (stepangle/math.pi)*180
    forth = (theta/math.pi)*180

    return [tx, ty]


step = 100

rfjoint1,rbjoint1 = [0.1,0.5],[0.1-0.75,0.5]

# for i in range(step+1):
#     frlink,frth = linkshow(rfjoint1,circletraj([-0.05,0],[0.25,0.17],step,i))
#     fllink,flth = linkshow(rfjoint1,[-0.05,0])
#     brlink,brth = linkshow(rbjoint1,[-0.05-0.75,0])
#     bllink,blth = linkshow(rbjoint1,[-0.05-0.75,0])

#     fr.append(frlink)
#     fl.append(fllink)
#     br.append(brlink)
#     bl.append(bllink)


for i in range(step+1):
    fr.append(linkshow(rfjoint1,[0.25,0.17]))
    fl.append(linkshow(rfjoint1,circletraj([-0.05,0],[0.25,0.17],step,i)))
    br.append(linkshow(rbjoint1,[-0.05-0.75,0]))
    bl.append(linkshow(rbjoint1,[-0.05-0.75,0]))

rfjoint1,rbjoint1 = inclination_from_back(rbjoint1, 15)

for i in range(step+1):
    fr.append(linkshow(rfjoint1,[0.25,0.17]))
    fl.append(linkshow(rfjoint1,[0.25,0.17]))
    br.append(linkshow(rbjoint1,circletraj([-0.05-0.75,0],[0.25-0.75,0],step,i)))
    bl.append(linkshow(rbjoint1,[-0.05-0.75,0]))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[0.25,0.17]))
#     fl.append(linkshow(rfjoint1,[0.25,0.17]))
#     br.append(linkshow(rbjoint1,[0.25-0.75,0]))
#     bl.append(linkshow(rbjoint1,circletraj([-0.05-0.75,0],[0.25-0.75,0],step,i)))

# rfjoint1,rbjoint1 = inclination_from_front([0.4,0.67],15)

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,circletraj([0.25,0.17],[0.55,0.34],step,i)))
#     fl.append(linkshow(rfjoint1,[0.25,0.17]))
#     br.append(linkshow(rbjoint1,[0.25-0.75,0]))
#     bl.append(linkshow(rbjoint1,[0.25-0.75,0]))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[0.55,0.34]))
#     fl.append(linkshow(rfjoint1,circletraj([0.25,0.17],[0.55,0.34],step,i)))
#     br.append(linkshow(rbjoint1,[0.25-0.75,0]))
#     bl.append(linkshow(rbjoint1,[0.25-0.75,0]))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[0.55,0.34]))
#     fl.append(linkshow(rfjoint1,[0.55,0.34]))
#     br.append(linkshow(rbjoint1,circletraj([0.25-0.75,0],[0.45-0.75,0],step,i)))
#     bl.append(linkshow(rbjoint1,[0.25-0.75,0]))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[0.55,0.34]))
#     fl.append(linkshow(rfjoint1,[0.55,0.34]))
#     br.append(linkshow(rbjoint1,[0.45-0.75,0]))
#     bl.append(linkshow(rbjoint1,circletraj([0.25-0.75,0],[0.45-0.75,0],step,i)))

# rfjoint1,rbjoint1 = inclination_from_front([0.6,0.84], 30)

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[0.55,0.34]))
#     fl.append(linkshow(rfjoint1,[0.55,0.34]))
#     br.append(linkshow(rbjoint1,circletraj([0.45-0.75,0],[0.7-0.75,0],step,i)))
#     bl.append(linkshow(rbjoint1,[0.45-0.75,0]))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[0.55,0.34]))
#     fl.append(linkshow(rfjoint1,[0.55,0.34]))
#     br.append(linkshow(rbjoint1,[0.7-0.75,0]))
#     bl.append(linkshow(rbjoint1,circletraj([0.45-0.75,0],[0.7-0.75,0],step,i)))

# rfjoint1,rbjoint1 = inclination_from_front([0.7,0.84], 30)

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,circletraj([0.55,0.34],[0.85,0.51],step,i)))
#     fl.append(linkshow(rfjoint1,[0.55,0.34]))
#     br.append(linkshow(rbjoint1,[0.7-0.75,0]))
#     bl.append(linkshow(rbjoint1,[0.7-0.75,0]))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[0.85,0.51]))
#     fl.append(linkshow(rfjoint1,circletraj([0.55,0.34],[0.85,0.51],step,i)))
#     br.append(linkshow(rbjoint1,[0.7-0.75,0]))
#     bl.append(linkshow(rbjoint1,[0.7-0.75,0]))

# rfjoint1,rbjoint1 = inclination_from_back([0.1,0.5], 30)

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[0.85,0.51]))
#     fl.append(linkshow(rfjoint1,[0.85,0.51]))
#     br.append(linkshow(rbjoint1,circletraj([0.7-0.75,0],[0.25,0.17],step,i)))
#     bl.append(linkshow(rbjoint1,[0.7-0.75,0]))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[0.85,0.51]))
#     fl.append(linkshow(rfjoint1,[0.85,0.51]))
#     br.append(linkshow(rbjoint1,[0.25,0.17]))
#     bl.append(linkshow(rbjoint1,circletraj([0.7-0.75,0],[0.25,0.17],step,i)))

# rfjoint1,rbjoint1 = inclination_from_back([0.4,0.67], 15)

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,circletraj([0.85,0.51],[1.05,0.51],step,i)))
#     fl.append(linkshow(rfjoint1,[0.85,0.51]))
#     br.append(linkshow(rbjoint1,[0.25,0.17]))
#     bl.append(linkshow(rbjoint1,[0.25,0.17]))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[1.05,0.51]))
#     fl.append(linkshow(rfjoint1,circletraj([0.85,0.51],[1.05,0.51],step,i)))
#     br.append(linkshow(rbjoint1,[0.25,0.17]))
#     bl.append(linkshow(rbjoint1,[0.25,0.17]))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[1.05,0.51]))
#     fl.append(linkshow(rfjoint1,[1.05,0.51]))
#     br.append(linkshow(rbjoint1,circletraj([0.25,0.17],[0.55,0.34],step,i)))
#     bl.append(linkshow(rbjoint1,[0.25,0.17]))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[1.05,0.51]))
#     fl.append(linkshow(rfjoint1,[1.05,0.51]))
#     br.append(linkshow(rbjoint1,[0.55,0.34]))
#     bl.append(linkshow(rbjoint1,circletraj([0.25,0.17],[0.55,0.34],step,i)))

# rfjoint1,rbjoint1 = inclination_from_back([0.6,0.84], 0)

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,circletraj([1.05,0.51],[1.2,0.51],step,i)))
#     fl.append(linkshow(rfjoint1,[1.05,0.51]))
#     br.append(linkshow(rbjoint1,[0.55,0.34]))
#     bl.append(linkshow(rbjoint1,[0.55,0.34]))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[1.2,0.51]))
#     fl.append(linkshow(rfjoint1,circletraj([1.05,0.51],[1.2,0.51],step,i)))
#     br.append(linkshow(rbjoint1,[0.55,0.34]))
#     bl.append(linkshow(rbjoint1,[0.55,0.34]))

# rfjoint1,rbjoint1 = inclination_from_back([0.7,0.84], 0)

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[1.2,0.51]))
#     fl.append(linkshow(rfjoint1,[1.2,0.51]))
#     br.append(linkshow(rbjoint1,circletraj([0.55,0.34],[0.8,0.51],step,i)))
#     bl.append(linkshow(rbjoint1,[0.55,0.34]))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[1.2,0.51]))
#     fl.append(linkshow(rfjoint1,[1.2,0.51]))
#     br.append(linkshow(rbjoint1,[0.8,0.51]))
#     bl.append(linkshow(rbjoint1,circletraj([0.55,0.34],[0.8,0.51],step,i)))

# rfjoint1,rbjoint1 = inclination_from_back([0.8,0.81], 0)

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,circletraj([1.2,0.51],[1.5,0.34],step,i)))
#     fl.append(linkshow(rfjoint1,[1.2,0.51]))
#     br.append(linkshow(rbjoint1,[0.8,0.51]))
#     bl.append(linkshow(rbjoint1,[0.8,0.51]))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[1.5,0.34]))
#     fl.append(linkshow(rfjoint1,circletraj([1.2,0.51],[1.5,0.34],step,i)))
#     br.append(linkshow(rbjoint1,[0.8,0.51]))
#     bl.append(linkshow(rbjoint1,[0.8,0.51]))

# rfjoint1,rbjoint1 = inclination_from_front([1.7,0.81], 0)

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[1.5,0.34]))
#     fl.append(linkshow(rfjoint1,[1.5,0.34]))
#     br.append(linkshow(rbjoint1,circletraj([0.8,0.51],[0.9,0.51],step,i)))
#     bl.append(linkshow(rbjoint1,[0.8,0.51]))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[1.5,0.34]))
#     fl.append(linkshow(rfjoint1,[1.5,0.34]))
#     br.append(linkshow(rbjoint1,[0.9,0.51]))
#     bl.append(linkshow(rbjoint1,circletraj([0.8,0.51],[0.9,0.51],step,i)))

# rfjoint1,rbjoint1 = inclination_from_front([1.85,0.64], -15)

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,circletraj([1.5,0.34],[1.8,0.17],step,i)))
#     fl.append(linkshow(rfjoint1,[1.5,0.34]))
#     br.append(linkshow(rbjoint1,[0.9,0.51]))
#     bl.append(linkshow(rbjoint1,[0.9,0.51]))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[1.8,0.17]))
#     fl.append(linkshow(rfjoint1,circletraj([1.5,0.34],[1.8,0.17],step,i)))
#     br.append(linkshow(rbjoint1,[0.9,0.51]))
#     bl.append(linkshow(rbjoint1,[0.9,0.51]))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[1.8,0.17]))
#     fl.append(linkshow(rfjoint1,[1.8,0.17]))
#     br.append(linkshow(rbjoint1,circletraj([0.9,0.51],[1.1,0.51],step,i)))
#     bl.append(linkshow(rbjoint1,[0.9,0.51]))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[1.8,0.17]))
#     fl.append(linkshow(rfjoint1,[1.8,0.17]))
#     br.append(linkshow(rbjoint1,[1.1,0.51]))
#     bl.append(linkshow(rbjoint1,circletraj([0.9,0.51],[1.1,0.51],step,i)))

# rfjoint1,rbjoint1 = inclination_from_front([2.15,0.47], -30)

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,circletraj([1.8,0.17],[2.1,0],step,i)))
#     fl.append(linkshow(rfjoint1,[1.8,0.17]))
#     br.append(linkshow(rbjoint1,[1.1,0.51]))
#     bl.append(linkshow(rbjoint1,[1.1,0.51]))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[2.1,0]))
#     fl.append(linkshow(rfjoint1,circletraj([1.8,0.17],[2.1,0],step,i)))
#     br.append(linkshow(rbjoint1,[1.1,0.51]))
#     bl.append(linkshow(rbjoint1,[1.1,0.51]))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[2.1,0]))
#     fl.append(linkshow(rfjoint1,[2.1,0]))
#     br.append(linkshow(rbjoint1,circletraj([1.1,0.51],[1.5,0.34],step,i)))
#     bl.append(linkshow(rbjoint1,[1.1,0.51]))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[2.1,0]))
#     fl.append(linkshow(rfjoint1,[2.1,0]))
#     br.append(linkshow(rbjoint1,[1.5,0.34]))
#     bl.append(linkshow(rbjoint1,circletraj([1.1,0.51],[1.5,0.34],step,i)))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,circletraj([2.1,0],[2.3,0],step,i)))
#     fl.append(linkshow(rfjoint1,[2.1,0]))
#     br.append(linkshow(rbjoint1,[1.5,0.34]))
#     bl.append(linkshow(rbjoint1,[1.5,0.34]))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[2.3,0]))
#     fl.append(linkshow(rfjoint1,circletraj([2.1,0],[2.3,0],step,i)))
#     br.append(linkshow(rbjoint1,[1.5,0.34]))
#     bl.append(linkshow(rbjoint1,[1.5,0.34]))

# rfjoint1,rbjoint1 = inclination_from_back([1.85,0.64], -15)

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[2.3,0]))
#     fl.append(linkshow(rfjoint1,[2.3,0]))
#     br.append(linkshow(rbjoint1,circletraj([1.5,0.34],[1.8,0.17],step,i)))
#     bl.append(linkshow(rbjoint1,[1.5,0.34]))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[2.3,0]))
#     fl.append(linkshow(rfjoint1,[2.3,0]))
#     br.append(linkshow(rbjoint1,[1.8,0.17]))
#     bl.append(linkshow(rbjoint1,circletraj([1.5,0.34],[1.8,0.17],step,i)))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,circletraj([2.3,0],[2.5,0],step,i)))
#     fl.append(linkshow(rfjoint1,[2.3,0]))
#     br.append(linkshow(rbjoint1,[1.8,0.17]))
#     bl.append(linkshow(rbjoint1,[1.8,0.17]))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[2.5,0]))
#     fl.append(linkshow(rfjoint1,circletraj([2.3,0],[2.5,0],step,i)))
#     br.append(linkshow(rbjoint1,[1.8,0.17]))
#     bl.append(linkshow(rbjoint1,[1.8,0.17]))

# rfjoint1,rbjoint1 = inclination_from_back([2.0,0.64], -15)

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,circletraj([2.5,0],[2.8,0],step,i)))
#     fl.append(linkshow(rfjoint1,[2.5,0]))
#     br.append(linkshow(rbjoint1,[1.8,0.17]))
#     bl.append(linkshow(rbjoint1,[1.8,0.17]))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[2.8,0]))
#     fl.append(linkshow(rfjoint1,circletraj([2.5,0],[2.8,0],step,i)))
#     br.append(linkshow(rbjoint1,[1.8,0.17]))
#     bl.append(linkshow(rbjoint1,[1.8,0.17]))

# rfjoint1,rbjoint1 = inclination_from_back([2.15,0.5], 0)

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[2.8,0]))
#     fl.append(linkshow(rfjoint1,[2.8,0]))
#     br.append(linkshow(rbjoint1,circletraj([1.8,0.17],[2.1,0],step,i)))
#     bl.append(linkshow(rbjoint1,[1.8,0.17]))

# for i in range(step+1):
#     fr.append(linkshow(rfjoint1,[2.8,0]))
#     fl.append(linkshow(rfjoint1,[2.8,0]))
#     br.append(linkshow(rbjoint1,[2.1,0]))
#     bl.append(linkshow(rbjoint1,circletraj([1.8,0.17],[2.1,0],step,i)))

########################################################################

# # create a time array from 0..100 sampled at 0.05 second steps
dt = 0.05

fig = plt.figure(figsize=(16, 8))
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-1, 3), ylim=(-0.5, 1.5))
ax.grid()

line1, = ax.plot([], [], 'o-', lw=2)
line2, = ax.plot([], [], 'o-', lw=2)
line3, = ax.plot([], [], 'o-', lw=2)
line4, = ax.plot([], [], 'o-', lw=2)
line5, = ax.plot([], [], 'o-', lw=2)
line6, = ax.plot([], [], 'o-', lw=2)

def init():
    line1.set_data([], [])
    line2.set_data([], [])
    line3.set_data([], [])
    line4.set_data([], [])
    line5.set_data([], [])
    line6.set_data([], [])

    return line1,line2,line3,line4,line5,line6


def animate(i):
    line1x = [fr[i][0][0], fr[i][1][0], fr[i][2][0]]
    line1y = [fr[i][0][1], fr[i][1][1], fr[i][2][1]]
    line2x = [fl[i][0][0], fl[i][1][0], fl[i][2][0]]
    line2y = [fl[i][0][1], fl[i][1][1], fl[i][2][1]]
    line3x = [bl[i][0][0], bl[i][1][0], bl[i][2][0]]
    line3y = [bl[i][0][1], bl[i][1][1], bl[i][2][1]]
    line4x = [br[i][0][0], br[i][1][0], br[i][2][0]]
    line4y = [br[i][0][1], br[i][1][1], br[i][2][1]]
    line5x = [fr[i][0][0], br[i][0][0]]
    line5y = [fr[i][0][1], br[i][0][1]]
    line6x = [0.1,0.1,0.4,0.4,0.7,0.7,1.3,1.3,1.6,1.6,1.9,1.9]
    line6y = [0,0.17,0.17,0.34,0.34,0.51,0.51,0.34,0.34,0.17,0.17,0]

    # line6x = []
    # line6y = []
    # for j in range(i):
    #     line6x.append(x5[j])
    #     line6y.append(y5[j])

    line1.set_data(line1x, line1y)
    line2.set_data(line2x, line2y)
    line3.set_data(line3x, line3y)
    line4.set_data(line4x, line4y)
    line5.set_data(line5x, line5y)
    line6.set_data(line6x, line6y)

    return line1,line2,line3,line4,line5,line6


ani = animation.FuncAnimation(fig, animate, np.arange(1, len(fl)),
                              interval=10, blit=True, init_func=init, repeat=False)

# ani.save('double_pendulum.mp4', fps=15)
plt.show()
