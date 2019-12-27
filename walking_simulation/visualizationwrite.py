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
centerset = []
frth = []
flth = []
brth = []
blth = []

f= open("/home/seungjun/walking_simulation/stairclimbing_53_2/24.txt",'w')

def linkshow(bodypoint, feetpoint, inclinedangle):
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
    th1n = (th1 * 180.0 / math.pi) -inclinedangle
    th2n = th2 * 180.0 / math.pi 
    leg.append(th1n)
    leg.append(th2n)

    return [joint1,joint2,joint3],leg

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
    r = 0.53

    tfx = fx
    tfy = fy
    tbx = fx - r*math.cos((angle/180.0)*math.pi)
    tby = fy - r*math.sin((angle/180.0)*math.pi)

    return [tfx,tfy],[tbx,tby]     

def inclination_from_back(backpoint, angle):
    bx = backpoint[0]
    by = backpoint[1]
    r = 0.53

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


step = 2

rfjoint1,rbjoint1 = [0.1,0.5],[0.1-0.53,0.5]

for i in range(step+1):#1
    frlink,frth = linkshow(rfjoint1,circletraj([-0.05,0],[0.25,0.17],step,i),0)
    fllink,flth = linkshow(rfjoint1,[-0.05,0],0)
    brlink,brth = linkshow(rbjoint1,[-0.05-0.53,0],0)
    bllink,blth = linkshow(rbjoint1,[-0.05-0.53,0],0)

    m1 = frlink[0][0]*0.3 + frlink[1][0]*0.3 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
    m2 = fllink[0][0]*0.3 + fllink[1][0]*0.3 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
    m3 = brlink[0][0]*0.3 + brlink[1][0]*0.3 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
    m4 = bllink[0][0]*0.3 + bllink[1][0]*0.3 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
    m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 0.7
    center = (m1 + m2 + m3 + m4 + m5)/4.7

    centerset.append(center)
    fr.append(frlink)
    fl.append(fllink)
    br.append(brlink)
    bl.append(bllink)
    
    f.write("%f," %flth[0])
    f.write("%f," %flth[1])
    f.write("%f," %frth[0])
    f.write("%f," %frth[1])
    f.write("%f," %blth[0])
    f.write("%f," %blth[1])
    f.write("%f," %brth[0])
    f.write("%f\n" %brth[1])

# for i in range(step+1):#2
#     frlink,frth = linkshow(rfjoint1,[0.25,0.17],0)
#     fllink,flth = linkshow(rfjoint1,circletraj([-0.05,0],[0.25,0.17],step,i),0)
#     brlink,brth = linkshow(rbjoint1,[-0.05-0.53,0],0)
#     bllink,blth = linkshow(rbjoint1,[-0.05-0.53,0],0)

#     m1 = frlink[0][0]*0.3 + frlink[1][0]*0.3 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.3 + fllink[1][0]*0.3 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.3 + brlink[1][0]*0.3 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.3 + bllink[1][0]*0.3 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 0.7
#     center = (m1 + m2 + m3 + m4 + m5)/4.7

#     centerset.append(center)

#     fr.append(frlink)
#     fl.append(fllink)
#     br.append(brlink)
#     bl.append(bllink)

#     f.write("%f," %flth[0])
#     f.write("%f," %flth[1])
#     f.write("%f," %frth[0])
#     f.write("%f," %frth[1])
#     f.write("%f," %blth[0])
#     f.write("%f," %blth[1])
#     f.write("%f," %brth[0])
#     f.write("%f\n" %brth[1])

rfjoint1,rbjoint1 = inclination_from_front([0.2,0.57],15)

# for i in range(step+1):#3
#     frlink,frth = linkshow(rfjoint1,[0.25,0.17],15)
#     fllink,flth = linkshow(rfjoint1,[0.25,0.17],15)
#     brlink,brth = linkshow(rbjoint1,circletraj([-0.05-0.53,0],[0.15-0.53,0],step,i),15)
#     bllink,blth = linkshow(rbjoint1,[-0.05-0.53,0],15)

#     m1 = frlink[0][0]*0.3 + frlink[1][0]*0.3 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.3 + fllink[1][0]*0.3 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.3 + brlink[1][0]*0.3 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.3 + bllink[1][0]*0.3 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 0.7
#     center = (m1 + m2 + m3 + m4 + m5)/4.7

#     centerset.append(center)

#     fr.append(frlink)
#     fl.append(fllink)
#     br.append(brlink)
#     bl.append(bllink)

#     f.write("%f," %flth[0])
#     f.write("%f," %flth[1])
#     f.write("%f," %frth[0])
#     f.write("%f," %frth[1])
#     f.write("%f," %blth[0])
#     f.write("%f," %blth[1])
#     f.write("%f," %brth[0])
#     f.write("%f\n" %brth[1])

# for i in range(step+1):#4
#     frlink,frth = linkshow(rfjoint1,[0.25,0.17],15)
#     fllink,flth = linkshow(rfjoint1,[0.25,0.17],15)
#     brlink,brth = linkshow(rbjoint1,[0.15-0.53,0],15)
#     bllink,blth = linkshow(rbjoint1,circletraj([-0.05-0.53,0],[0.15-0.53,0],step,i),15)

#     m1 = frlink[0][0]*0.3 + frlink[1][0]*0.3 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.3 + fllink[1][0]*0.3 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.3 + brlink[1][0]*0.3 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.3 + bllink[1][0]*0.3 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 0.7
#     center = (m1 + m2 + m3 + m4 + m5)/4.7

#     centerset.append(center)

#     fr.append(frlink)
#     fl.append(fllink)
#     br.append(brlink)
#     bl.append(bllink)

#     f.write("%f," %flth[0])
#     f.write("%f," %flth[1])
#     f.write("%f," %frth[0])
#     f.write("%f," %frth[1])
#     f.write("%f," %blth[0])
#     f.write("%f," %blth[1])
#     f.write("%f," %brth[0])
#     f.write("%f\n" %brth[1])

rfjoint1,rbjoint1 = inclination_from_front([0.35,0.60],15)

# for i in range(step+1):#5
#     frlink,frth = linkshow(rfjoint1,[0.25,0.17],15)
#     fllink,flth = linkshow(rfjoint1,[0.25,0.17],15)
#     brlink,brth = linkshow(rbjoint1,circletraj([0.15-0.53,0],[0.35-0.53,0],step,i),15)
#     bllink,blth = linkshow(rbjoint1,[0.15-0.53,0],15)
    
#     m1 = frlink[0][0]*0.3 + frlink[1][0]*0.3 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.3 + fllink[1][0]*0.3 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.3 + brlink[1][0]*0.3 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.3 + bllink[1][0]*0.3 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 0.7
#     center = (m1 + m2 + m3 + m4 + m5)/4.7

#     centerset.append(center)
#     fr.append(frlink)
#     fl.append(fllink)
#     br.append(brlink)
#     bl.append(bllink)

#     f.write("%f," %flth[0])
#     f.write("%f," %flth[1])
#     f.write("%f," %frth[0])
#     f.write("%f," %frth[1])
#     f.write("%f," %blth[0])
#     f.write("%f," %blth[1])
#     f.write("%f," %brth[0])
#     f.write("%f\n" %brth[1])

# for i in range(step+1):#6
#     frlink,frth = linkshow(rfjoint1,[0.25,0.17],15)
#     fllink,flth = linkshow(rfjoint1,[0.25,0.17],15)
#     brlink,brth = linkshow(rbjoint1,[0.35-0.53,0],15)
#     bllink,blth = linkshow(rbjoint1,circletraj([0.15-0.53,0],[0.35-0.53,0],step,i),15)
    
#     m1 = frlink[0][0]*0.3 + frlink[1][0]*0.3 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.3 + fllink[1][0]*0.3 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.3 + brlink[1][0]*0.3 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.3 + bllink[1][0]*0.3 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 0.7
#     center = (m1 + m2 + m3 + m4 + m5)/4.7

#     centerset.append(center)

#     fr.append(frlink)
#     fl.append(fllink)
#     br.append(brlink)
#     bl.append(bllink)

#     f.write("%f," %flth[0])
#     f.write("%f," %flth[1])
#     f.write("%f," %frth[0])
#     f.write("%f," %frth[1])
#     f.write("%f," %blth[0])
#     f.write("%f," %blth[1])
#     f.write("%f," %brth[0])
#     f.write("%f\n" %brth[1])

rfjoint1,rbjoint1 = inclination_from_front([0.4,0.67],15)

# for i in range(step+1):#7
#     frlink,frth = linkshow(rfjoint1,circletraj([0.25,0.17],[0.55,0.34],step,i),15)
#     fllink,flth = linkshow(rfjoint1,[0.25,0.17],15)
#     brlink,brth = linkshow(rbjoint1,[0.35-0.53,0],15)
#     bllink,blth = linkshow(rbjoint1,[0.35-0.53,0],15)
    
#     m1 = frlink[0][0]*0.3 + frlink[1][0]*0.3 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.3 + fllink[1][0]*0.3 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.3 + brlink[1][0]*0.3 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.3 + bllink[1][0]*0.3 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 0.7
#     center = (m1 + m2 + m3 + m4 + m5)/4.7

#     centerset.append(center)

#     fr.append(frlink)
#     fl.append(fllink)
#     br.append(brlink)
#     bl.append(bllink)

#     f.write("%f," %flth[0])
#     f.write("%f," %flth[1])
#     f.write("%f," %frth[0])
#     f.write("%f," %frth[1])
#     f.write("%f," %blth[0])
#     f.write("%f," %blth[1])
#     f.write("%f," %brth[0])
#     f.write("%f\n" %brth[1])

# for i in range(step+1):#8
#     frlink,frth = linkshow(rfjoint1,[0.55,0.34],15)
#     fllink,flth = linkshow(rfjoint1,circletraj([0.25,0.17],[0.55,0.34],step,i),15)
#     brlink,brth = linkshow(rbjoint1,[0.35-0.53,0],15)
#     bllink,blth = linkshow(rbjoint1,[0.35-0.53,0],15)
    
#     m1 = frlink[0][0]*0.3 + frlink[1][0]*0.3 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.3 + fllink[1][0]*0.3 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.3 + brlink[1][0]*0.3 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.3 + bllink[1][0]*0.3 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 0.7
#     center = (m1 + m2 + m3 + m4 + m5)/4.7

#     centerset.append(center)
#     fr.append(frlink)
#     fl.append(fllink)
#     br.append(brlink)
#     bl.append(bllink)

#     f.write("%f," %flth[0])
#     f.write("%f," %flth[1])
#     f.write("%f," %frth[0])
#     f.write("%f," %frth[1])
#     f.write("%f," %blth[0])
#     f.write("%f," %blth[1])
#     f.write("%f," %brth[0])
#     f.write("%f\n" %brth[1])


rfjoint1,rbjoint1 = inclination_from_back([0.0,0.5], 30)

# for i in range(step+1):#9
#     frlink,frth = linkshow(rfjoint1,[0.55,0.34],30)
#     fllink,flth = linkshow(rfjoint1,[0.55,0.34],30)
#     brlink,brth = linkshow(rbjoint1,circletraj([0.35-0.53,0],[0.0,0],step,i),30)
#     bllink,blth = linkshow(rbjoint1,[0.35-0.53,0],30)
    
#     m1 = frlink[0][0]*0.3 + frlink[1][0]*0.3 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.3 + fllink[1][0]*0.3 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.3 + brlink[1][0]*0.3 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.3 + bllink[1][0]*0.3 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 0.7
#     center = (m1 + m2 + m3 + m4 + m5)/4.7

#     centerset.append(center)
#     fr.append(frlink)
#     fl.append(fllink)
#     br.append(brlink)
#     bl.append(bllink)

#     f.write("%f," %flth[0])
#     f.write("%f," %flth[1])
#     f.write("%f," %frth[0])
#     f.write("%f," %frth[1])
#     f.write("%f," %blth[0])
#     f.write("%f," %blth[1])
#     f.write("%f," %brth[0])
#     f.write("%f\n" %brth[1])
    

# for i in range(step+1):#10
#     frlink,frth = linkshow(rfjoint1,[0.55,0.34],30)
#     fllink,flth = linkshow(rfjoint1,[0.55,0.34],30)
#     brlink,brth = linkshow(rbjoint1,[0.0,0],30)
#     bllink,blth = linkshow(rbjoint1,circletraj([0.35-0.53,0],[0.0,0],step,i),30)
    
#     m1 = frlink[0][0]*0.3 + frlink[1][0]*0.3 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.3 + fllink[1][0]*0.3 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.3 + brlink[1][0]*0.3 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.3 + bllink[1][0]*0.3 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 0.7
#     center = (m1 + m2 + m3 + m4 + m5)/4.7

#     centerset.append(center)
#     fr.append(frlink)
#     fl.append(fllink)
#     br.append(brlink)
#     bl.append(bllink)

#     f.write("%f," %flth[0])
#     f.write("%f," %flth[1])
#     f.write("%f," %frth[0])
#     f.write("%f," %frth[1])
#     f.write("%f," %blth[0])
#     f.write("%f," %blth[1])
#     f.write("%f," %brth[0])
#     f.write("%f\n" %brth[1])

rfjoint1,rbjoint1 = inclination_from_back([0.2,0.5], 30)


# for i in range(step+1):#11
#     frlink,frth = linkshow(rfjoint1,[0.55,0.34],30)
#     fllink,flth = linkshow(rfjoint1,[0.55,0.34],30)
#     brlink,brth = linkshow(rbjoint1,circletraj([0,0],[0.25,0.17],step,i),30)
#     bllink,blth = linkshow(rbjoint1,[0,0],30)

#     m1 = frlink[0][0]*0.3 + frlink[1][0]*0.3 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.3 + fllink[1][0]*0.3 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.3 + brlink[1][0]*0.3 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.3 + bllink[1][0]*0.3 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 0.7
#     center = (m1 + m2 + m3 + m4 + m5)/4.7

#     centerset.append(center)
#     fr.append(frlink)
#     fl.append(fllink)
#     br.append(brlink)
#     bl.append(bllink)

#     f.write("%f," %flth[0])
#     f.write("%f," %flth[1])
#     f.write("%f," %frth[0])
#     f.write("%f," %frth[1])
#     f.write("%f," %blth[0])
#     f.write("%f," %blth[1])
#     f.write("%f," %brth[0])
#     f.write("%f\n" %brth[1])

# for i in range(step+1):#12
#     frlink,frth = linkshow(rfjoint1,[0.55,0.34],30)
#     fllink,flth = linkshow(rfjoint1,[0.55,0.34],30)
#     brlink,brth = linkshow(rbjoint1,[0.25,0.17],30)
#     bllink,blth = linkshow(rbjoint1,circletraj([0,0],[0.25,0.17],step,i),30)

#     m1 = frlink[0][0]*0.3 + frlink[1][0]*0.3 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.3 + fllink[1][0]*0.3 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.3 + brlink[1][0]*0.3 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.3 + bllink[1][0]*0.3 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 0.7
#     center = (m1 + m2 + m3 + m4 + m5)/4.7

#     centerset.append(center)
#     fr.append(frlink)
#     fl.append(fllink)
#     br.append(brlink)
#     bl.append(bllink)

#     f.write("%f," %flth[0])
#     f.write("%f," %flth[1])
#     f.write("%f," %frth[0])
#     f.write("%f," %frth[1])
#     f.write("%f," %blth[0])
#     f.write("%f," %blth[1])
#     f.write("%f," %brth[0])
#     f.write("%f\n" %brth[1])

rfjoint1,rbjoint1 = inclination_from_front([0.7,0.84], 30)

# for i in range(step+1):#13
#     frlink,frth = linkshow(rfjoint1,circletraj([0.55,0.34],[0.85,0.51],step,i),30)
#     fllink,flth = linkshow(rfjoint1,[0.55,0.34],30)
#     brlink,brth = linkshow(rbjoint1,[0.25,0.17],30)
#     bllink,blth = linkshow(rbjoint1,[0.25,0.17],30)

#     m1 = frlink[0][0]*0.3 + frlink[1][0]*0.3 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.3 + fllink[1][0]*0.3 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.3 + brlink[1][0]*0.3 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.3 + bllink[1][0]*0.3 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 0.7
#     center = (m1 + m2 + m3 + m4 + m5)/4.7

#     centerset.append(center)

#     fr.append(frlink)
#     fl.append(fllink)
#     br.append(brlink)
#     bl.append(bllink)

#     f.write("%f," %flth[0])
#     f.write("%f," %flth[1])
#     f.write("%f," %frth[0])
#     f.write("%f," %frth[1])
#     f.write("%f," %blth[0])
#     f.write("%f," %blth[1])
#     f.write("%f," %brth[0])
#     f.write("%f\n" %brth[1])

# for i in range(step+1):#14
#     frlink,frth = linkshow(rfjoint1,[0.85,0.51],30)
#     fllink,flth = linkshow(rfjoint1,circletraj([0.55,0.34],[0.85,0.51],step,i),30)
#     brlink,brth = linkshow(rbjoint1,[0.25,0.17],30)
#     bllink,blth = linkshow(rbjoint1,[0.25,0.17],30)

#     m1 = frlink[0][0]*0.3 + frlink[1][0]*0.3 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.3 + fllink[1][0]*0.3 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.3 + brlink[1][0]*0.3 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.3 + bllink[1][0]*0.3 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 0.7
#     center = (m1 + m2 + m3 + m4 + m5)/4.7

#     centerset.append(center)

#     fr.append(frlink)
#     fl.append(fllink)
#     br.append(brlink)
#     bl.append(bllink)

#     f.write("%f," %flth[0])
#     f.write("%f," %flth[1])
#     f.write("%f," %frth[0])
#     f.write("%f," %frth[1])
#     f.write("%f," %blth[0])
#     f.write("%f," %blth[1])
#     f.write("%f," %brth[0])
#     f.write("%f\n" %brth[1])

rfjoint1,rbjoint1 = inclination_from_back([0.45,0.60], 30)

# for i in range(step+1):#15
#     frlink,frth = linkshow(rfjoint1,[0.85,0.51],30)
#     fllink,flth = linkshow(rfjoint1,[0.85,0.51],30)
#     brlink,brth = linkshow(rbjoint1,circletraj([0.25,0.17],[0.3,0.17],step,i),30)
#     bllink,blth = linkshow(rbjoint1,[0.25,0.17],30)
#     m1 = frlink[0][0]*0.3 + frlink[1][0]*0.3 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.3 + fllink[1][0]*0.3 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.3 + brlink[1][0]*0.3 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.3 + bllink[1][0]*0.3 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 0.7
#     center = (m1 + m2 + m3 + m4 + m5)/4.7

#     centerset.append(center)

#     fr.append(frlink)
#     fl.append(fllink)
#     br.append(brlink)
#     bl.append(bllink)

#     f.write("%f," %flth[0])
#     f.write("%f," %flth[1])
#     f.write("%f," %frth[0])
#     f.write("%f," %frth[1])
#     f.write("%f," %blth[0])
#     f.write("%f," %blth[1])
#     f.write("%f," %brth[0])
#     f.write("%f\n" %brth[1])

# for i in range(step+1):#16
#     frlink,frth = linkshow(rfjoint1,[0.85,0.51],30)
#     fllink,flth = linkshow(rfjoint1,[0.85,0.51],30)
#     brlink,brth = linkshow(rbjoint1,[0.3,0.17],30)
#     bllink,blth = linkshow(rbjoint1,circletraj([0.25,0.17],[0.3,0.17],step,i),30)

#     m1 = frlink[0][0]*0.3 + frlink[1][0]*0.3 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.3 + fllink[1][0]*0.3 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.3 + brlink[1][0]*0.3 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.3 + bllink[1][0]*0.3 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 0.7
#     center = (m1 + m2 + m3 + m4 + m5)/4.7

#     centerset.append(center)
#     fr.append(frlink)
#     fl.append(fllink)
#     br.append(brlink)
#     bl.append(bllink)

#     f.write("%f," %flth[0])
#     f.write("%f," %flth[1])
#     f.write("%f," %frth[0])
#     f.write("%f," %frth[1])
#     f.write("%f," %blth[0])
#     f.write("%f," %blth[1])
#     f.write("%f," %brth[0])
#     f.write("%f\n" %brth[1])

rfjoint1,rbjoint1 = inclination_from_back([0.5,0.67], 30)

# for i in range(step+1):#17
#     frlink,frth = linkshow(rfjoint1,[0.85,0.51],30)
#     fllink,flth = linkshow(rfjoint1,[0.85,0.51],30)
#     brlink,brth = linkshow(rbjoint1,circletraj([0.3,0.17],[0.55,0.34],step,i),30)
#     bllink,blth = linkshow(rbjoint1,[0.3,0.17],30)
#     m1 = frlink[0][0]*0.3 + frlink[1][0]*0.3 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.3 + fllink[1][0]*0.3 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.3 + brlink[1][0]*0.3 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.3 + bllink[1][0]*0.3 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 0.7
#     center = (m1 + m2 + m3 + m4 + m5)/4.7

#     centerset.append(center)

#     fr.append(frlink)
#     fl.append(fllink)
#     br.append(brlink)
#     bl.append(bllink)

#     f.write("%f," %flth[0])
#     f.write("%f," %flth[1])
#     f.write("%f," %frth[0])
#     f.write("%f," %frth[1])
#     f.write("%f," %blth[0])
#     f.write("%f," %blth[1])
#     f.write("%f," %brth[0])
#     f.write("%f\n" %brth[1])

# for i in range(step+1):#18
#     frlink,frth = linkshow(rfjoint1,[0.85,0.51],30)
#     fllink,flth = linkshow(rfjoint1,[0.85,0.51],30)
#     brlink,brth = linkshow(rbjoint1,[0.55,0.34],30)
#     bllink,blth = linkshow(rbjoint1,circletraj([0.3,0.17],[0.55,0.34],step,i),30)
#     m1 = frlink[0][0]*0.3 + frlink[1][0]*0.3 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.3 + fllink[1][0]*0.3 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.3 + brlink[1][0]*0.3 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.3 + bllink[1][0]*0.3 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 0.7
#     center = (m1 + m2 + m3 + m4 + m5)/4.7

#     centerset.append(center)

#     fr.append(frlink)
#     fl.append(fllink)
#     br.append(brlink)
#     bl.append(bllink)

#     f.write("%f," %flth[0])
#     f.write("%f," %flth[1])
#     f.write("%f," %frth[0])
#     f.write("%f," %frth[1])
#     f.write("%f," %blth[0])
#     f.write("%f," %blth[1])
#     f.write("%f," %brth[0])
#     f.write("%f\n" %brth[1])


# for i in range(step+1):#19
#     frlink,frth = linkshow(rfjoint1,circletraj([0.85,0.51],[1.05,0.51],step,i),30)
#     fllink,flth = linkshow(rfjoint1,[0.85,0.51],30)
#     brlink,brth = linkshow(rbjoint1,[0.55,0.34],30)
#     bllink,blth = linkshow(rbjoint1,[0.55,0.34],30)
    
#     m1 = frlink[0][0]*0.3 + frlink[1][0]*0.3 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.3 + fllink[1][0]*0.3 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.3 + brlink[1][0]*0.3 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.3 + bllink[1][0]*0.3 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 0.7
#     center = (m1 + m2 + m3 + m4 + m5)/4.7

#     centerset.append(center)

#     fr.append(frlink)
#     fl.append(fllink)
#     br.append(brlink)
#     bl.append(bllink)

#     f.write("%f," %flth[0])
#     f.write("%f," %flth[1])
#     f.write("%f," %frth[0])
#     f.write("%f," %frth[1])
#     f.write("%f," %blth[0])
#     f.write("%f," %blth[1])
#     f.write("%f," %brth[0])
#     f.write("%f\n" %brth[1])

# for i in range(step+1):#20
#     frlink,frth = linkshow(rfjoint1,[1.05,0.51],30)
#     fllink,flth = linkshow(rfjoint1,circletraj([0.85,0.51],[1.05,0.51],step,i),30)
#     brlink,brth = linkshow(rbjoint1,[0.55,0.34],30)
#     bllink,blth = linkshow(rbjoint1,[0.55,0.34],30)
    
#     m1 = frlink[0][0]*0.3 + frlink[1][0]*0.3 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.3 + fllink[1][0]*0.3 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.3 + brlink[1][0]*0.3 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.3 + bllink[1][0]*0.3 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 0.7
#     center = (m1 + m2 + m3 + m4 + m5)/4.7

#     centerset.append(center)

#     fr.append(frlink)
#     fl.append(fllink)
#     br.append(brlink)
#     bl.append(bllink)

#     f.write("%f," %flth[0])
#     f.write("%f," %flth[1])
#     f.write("%f," %frth[0])
#     f.write("%f," %frth[1])
#     f.write("%f," %blth[0])
#     f.write("%f," %blth[1])
#     f.write("%f," %brth[0])
#     f.write("%f\n" %brth[1])

rfjoint1,rbjoint1 = inclination_from_back([0.7,0.84], 15)

# for i in range(step+1):#21
#     frlink,frth = linkshow(rfjoint1,circletraj([1.05,0.51],[1.20,0.51],step,i),15)
#     fllink,flth = linkshow(rfjoint1,[1.05,0.51],15)
#     brlink,brth = linkshow(rbjoint1,[0.55,0.34],15)
#     bllink,blth = linkshow(rbjoint1,[0.55,0.34],15)
    
#     m1 = frlink[0][0]*0.3 + frlink[1][0]*0.3 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.3 + fllink[1][0]*0.3 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.3 + brlink[1][0]*0.3 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.3 + bllink[1][0]*0.3 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 0.7
#     center = (m1 + m2 + m3 + m4 + m5)/4.7

#     centerset.append(center)

#     fr.append(frlink)
#     fl.append(fllink)
#     br.append(brlink)
#     bl.append(bllink)

#     f.write("%f," %flth[0])
#     f.write("%f," %flth[1])
#     f.write("%f," %frth[0])
#     f.write("%f," %frth[1])
#     f.write("%f," %blth[0])
#     f.write("%f," %blth[1])
#     f.write("%f," %brth[0])
#     f.write("%f\n" %brth[1])

# for i in range(step+1):#22
#     frlink,frth = linkshow(rfjoint1,[1.20,0.51],15)
#     fllink,flth = linkshow(rfjoint1,circletraj([1.05,0.51],[1.20,0.51],step,i),15)
#     brlink,brth = linkshow(rbjoint1,[0.55,0.34],15)
#     bllink,blth = linkshow(rbjoint1,[0.55,0.34],15)
    
#     m1 = frlink[0][0]*0.3 + frlink[1][0]*0.3 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.3 + fllink[1][0]*0.3 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.3 + brlink[1][0]*0.3 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.3 + bllink[1][0]*0.3 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 0.7
#     center = (m1 + m2 + m3 + m4 + m5)/4.7

#     centerset.append(center)

#     fr.append(frlink)
#     fl.append(fllink)
#     br.append(brlink)
#     bl.append(bllink)

#     f.write("%f," %flth[0])
#     f.write("%f," %flth[1])
#     f.write("%f," %frth[0])
#     f.write("%f," %frth[1])
#     f.write("%f," %blth[0])
#     f.write("%f," %blth[1])
#     f.write("%f," %brth[0])
#     f.write("%f\n" %brth[1])

# for i in range(step+1):#23
#     frlink,frth = linkshow(rfjoint1,[1.20,0.51],15)
#     fllink,flth = linkshow(rfjoint1,[1.20,0.51],15)
#     brlink,brth = linkshow(rbjoint1,circletraj([0.55,0.34],[0.80,0.51],step,i),15)
#     bllink,blth = linkshow(rbjoint1,[0.55,0.34],15)
#     m1 = frlink[0][0]*0.3 + frlink[1][0]*0.3 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.3 + fllink[1][0]*0.3 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.3 + brlink[1][0]*0.3 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.3 + bllink[1][0]*0.3 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 0.7
#     center = (m1 + m2 + m3 + m4 + m5)/4.7

#     centerset.append(center)

#     fr.append(frlink)
#     fl.append(fllink)
#     br.append(brlink)
#     bl.append(bllink)

#     f.write("%f," %flth[0])
#     f.write("%f," %flth[1])
#     f.write("%f," %frth[0])
#     f.write("%f," %frth[1])
#     f.write("%f," %blth[0])
#     f.write("%f," %blth[1])
#     f.write("%f," %brth[0])
#     f.write("%f\n" %brth[1])

for i in range(step+1):#24
    frlink,frth = linkshow(rfjoint1,[1.20,0.51],15)
    fllink,flth = linkshow(rfjoint1,[1.20,0.51],15)
    brlink,brth = linkshow(rbjoint1,[0.80,0.51],15)
    bllink,blth = linkshow(rbjoint1,circletraj([0.55,0.34],[0.80,0.51],step,i),15)
    m1 = frlink[0][0]*0.3 + frlink[1][0]*0.3 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
    m2 = fllink[0][0]*0.3 + fllink[1][0]*0.3 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
    m3 = brlink[0][0]*0.3 + brlink[1][0]*0.3 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
    m4 = bllink[0][0]*0.3 + bllink[1][0]*0.3 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
    m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 0.7
    center = (m1 + m2 + m3 + m4 + m5)/4.7

    centerset.append(center)

    fr.append(frlink)
    fl.append(fllink)
    br.append(brlink)
    bl.append(bllink)

    f.write("%f," %flth[0])
    f.write("%f," %flth[1])
    f.write("%f," %frth[0])
    f.write("%f," %frth[1])
    f.write("%f," %blth[0])
    f.write("%f," %blth[1])
    f.write("%f," %brth[0])
    f.write("%f\n" %brth[1])

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
line7, = ax.plot([], [], 'o-', lw=2)

def init():
    line1.set_data([], [])
    line2.set_data([], [])
    line3.set_data([], [])
    line4.set_data([], [])
    line5.set_data([], [])
    line6.set_data([], [])
    line7.set_data([], [])

    return line1,line2,line3,line4,line5,line6,line7


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
    line7x = [centerset[i],centerset[i]]
    line7y = [0,1]

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
    line7.set_data(line7x, line7y)

    return line1,line2,line3,line4,line5,line6,line7


ani = animation.FuncAnimation(fig, animate, np.arange(1, len(fl)),
                              interval=10, blit=True, init_func=init, repeat=False)

# ani.save('double_pendulum.mp4', fps=15)
plt.show()
