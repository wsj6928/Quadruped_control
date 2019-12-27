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

# control loop
step = 1000
rfjoint1=[-0.013,0.597]#15
rbjoint1=[-0.737,0.403]#15

# f= open("/home/wsj/python_coding/quadruped/file/angle.txt",'w')

def beginhold(trajectory, walkwidth,front,i):

    theta = math.pi + i*math.pi/(step-1)
    x = 0
    y = 0.5

    th2 = np.arccos(((x**2)+(y**2)-(link1**2)-(link2**2))/(2*link1*link2))
    th2_i = -th2
    th1 = np.arctan2(y, x)-np.arccos(((x**2)+(y**2)+(link1**2) -
                                      (link2**2)) / (2*link1*math.sqrt(x**2+y**2)))

    # f.write("%f ," %th1)
    # f.write("%f \n" %th2)
    
    if front==True:
        joint1 = rfjoint1
        joint2 = [joint1[0]-link1*np.cos(th1), joint1[1]-link1*np.sin(th1)]
        joint3 = [joint2[0]-link2*np.cos(th1+th2), joint2[1]-link2*np.sin(th1+th2)]

        trajectory.append([joint1, joint2, joint3])
    else:
        joint1 = rbjoint1
        joint2 = [joint1[0]-link1*np.cos(th1), joint1[1]-link1*np.sin(th1)]
        joint3 = [joint2[0]-link2*np.cos(th1+th2), joint2[1]-link2*np.sin(th1+th2)]

        trajectory.append([joint1, joint2, joint3])
    return joint1

def beginstepforward(trajectory, walkwidth,front,i):

    theta = math.pi + i*math.pi/(step-1)
    x = -walkwidth/2.0- (walkwidth/2.0 * np.cos(theta))
    y = 0.5 + walkwidth/2.0 * np.sin(theta)

    th2 = np.arccos(((x**2)+(y**2)-(link1**2)-(link2**2))/(2*link1*link2))
    th2_i = -th2
    th1 = np.arctan2(y, x)-np.arccos(((x**2)+(y**2)+(link1**2) -
                                      (link2**2)) / (2*link1*math.sqrt(x**2+y**2)))

    # f.write("%f ," %th1)
    # f.write("%f \n" %th2)
    if front==True:
        joint1 = fjoint1
        joint2 = [joint1[0]-link1*np.cos(th1), joint1[1]-link1*np.sin(th1)]
        joint3 = [joint2[0]-link2*np.cos(th1+th2), joint2[1]-link2*np.sin(th1+th2)]

        trajectory.append([joint1, joint2, joint3])
    else:
        joint1 = bjoint1
        joint2 = [joint1[0]-link1*np.cos(th1), joint1[1]-link1*np.sin(th1)]
        joint3 = [joint2[0]-link2*np.cos(th1+th2), joint2[1]-link2*np.sin(th1+th2)]

        trajectory.append([joint1, joint2, joint3])

def pullback(trajectory, walkwidth,front,i):

    x = -walkwidth+(i*(walkwidth*2/(step-1)))
    y = 0.5
    xi = -walkwidth
    yi = 0.5

    th2 = np.arccos(((x**2)+(y**2)-(link1**2)-(link2**2))/(2*link1*link2))
    th2_i = -th2
    th1 = np.arctan2(y, x)-np.arccos(((x**2)+(y**2)+(link1**2) -
                                      (link2**2)) / (2*link1*math.sqrt(x**2+y**2)))

    # f.write("%f ," %th1)
    # f.write("%f \n" %th2)

    th2i = np.arccos(((xi**2)+(yi**2)-(link1**2)-(link2**2))/(2*link1*link2))
    th1i = np.arctan2(yi, xi)-np.arccos(((xi**2)+(yi**2)+(link1**2) -
                                      (link2**2)) / (2*link1*math.sqrt(xi**2+yi**2)))
    if front==True:
        joint1 = rfjoint1
        joint2 = [joint1[0]-link1*np.cos(th1i), joint1[1]-link1*np.sin(th1i)]
        joint3 = [joint2[0]-link2*np.cos(th1i+th2i), joint2[1]-link2*np.sin(th1i+th2i)]

        joint2 = [joint3[0]+link2*np.cos(th1+th2), joint3[1]+link2*np.sin(th1+th2)]
        joint1 = [joint2[0]+link1*np.cos(th1), joint2[1]+link1*np.sin(th1)]

        trajectory.append([joint1, joint2, joint3])
    else:
        joint1 = rbjoint1
        joint2 = [joint1[0]-link1*np.cos(th1i), joint1[1]-link1*np.sin(th1i)]
        joint3 = [joint2[0]-link2*np.cos(th1i+th2i), joint2[1]-link2*np.sin(th1i+th2i)]

        joint2 = [joint3[0]+link2*np.cos(th1+th2), joint3[1]+link2*np.sin(th1+th2)]
        joint1 = [joint2[0]+link1*np.cos(th1), joint2[1]+link1*np.sin(th1)]

        trajectory.append([joint1, joint2, joint3])
    return joint1

def stepforward(trajectory, walkwidth,front,i):

    theta = math.pi + i*math.pi/(step-1)
    x = - walkwidth * np.cos(theta)
    y = 0.5 + walkwidth * np.sin(theta)

    th2 = np.arccos(((x**2)+(y**2)-(link1**2)-(link2**2))/(2*link1*link2))
    th2_i = -th2
    th1 = np.arctan2(y, x)-np.arccos(((x**2)+(y**2)+(link1**2) -
                                      (link2**2)) / (2*link1*math.sqrt(x**2+y**2)))

    # f.write("%f ," %th1)
    # f.write("%f \n" %th2)

    if front==True:
        joint1 = fjoint1
        joint2 = [joint1[0]-link1*np.cos(th1), joint1[1]-link1*np.sin(th1)]
        joint3 = [joint2[0]-link2*np.cos(th1+th2), joint2[1]-link2*np.sin(th1+th2)]

        trajectory.append([joint1, joint2, joint3])
    else:
        joint1 = bjoint1
        joint2 = [joint1[0]-link1*np.cos(th1), joint1[1]-link1*np.sin(th1)]
        joint3 = [joint2[0]-link2*np.cos(th1+th2), joint2[1]-link2*np.sin(th1+th2)]

        trajectory.append([joint1, joint2, joint3])

for i in range(0, step):
    fjoint1 = beginhold(fl,0.15,True,i)
    beginstepforward(fr,0.15,True,i)
    bjoint1 = beginhold(br,0.15,False,i)
    beginhold(bl,0.15,False,i)
rfjoint1=fjoint1
rbjoint1=bjoint1
# for i in range(0, step):
#     fjoint1 = pullback(fr, 0.15,True,i)
#     beginstepforward(fl,0.15,True,i)   
#     bjoint1 = pullback(bl, 0.15,False,i)
#     beginstepforward(br,0.15,False,i)   
# rfjoint1=fjoint1
# rbjoint1=bjoint1
# for i in range(0, step):
#     fjoint1 = pullback(fl, 0.15,True,i)
#     stepforward(fr, 0.15,True,i)
#     bjoint1 = pullback(br, 0.15,False,i)
#     stepforward(bl, 0.15,False,i)
# rfjoint1=fjoint1
# rbjoint1=bjoint1
# for i in range(0, step):
#     fjoint1 = pullback(fr, 0.15,True,i)
#     stepforward(fl, 0.15,True,i)
#     bjoint1 = pullback(bl, 0.15,False,i)
#     stepforward(br, 0.15,False,i)

# f.close()

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


ani = animation.FuncAnimation(fig, animate, np.arange(1, len(fr)),
                              interval=1, blit=True, init_func=init, repeat=False)

# ani.save('double_pendulum.mp4', fps=15)
plt.show()
