import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

######################### you can change these values ########################
link1 = 0.40  # link length setting
link2 = 0.40
th1 = 45.0
th2 = 45.0

step_h=0.175
step_d=0.285
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

f= open("/home/seungjun/walking_simulation/mark8_12stairs/test.txt",'w')

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

def linkshow2(bodypoint, feetpoint, inclinedangle):
    bx = bodypoint[0]
    by = bodypoint[1]
    fx = feetpoint[0]
    fy = feetpoint[1]
    x = bx - fx
    y = by - fy

    th2 = np.arccos(((x**2)+(y**2)-(link1**2)-(link2**2))/(2*link1*link2))
    th2_i = -th2
    th1_v =abs(th2)+( np.arctan2(y, x)-np.arccos(((x**2)+(y**2)+(link1**2) -
                                      (link2**2)) / (2*link1*math.sqrt(x**2+y**2))))
    th1 = math.pi - th1_v
    joint1 = bodypoint
    joint2 = [joint1[0]-link1*np.cos(th1_v), joint1[1]-link1*np.sin(th1_v)]
    joint3 = feetpoint

    leg = []
    th1n = (th1 * 180.0 / math.pi) +inclinedangle
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
    r = 0.22

    tfx = fx
    tfy = fy
    tbx = fx - r*math.cos((angle/180.0)*math.pi)
    tby = fy - r*math.sin((angle/180.0)*math.pi)

    return [tfx,tfy],[tbx,tby]     

def inclination_from_back(backpoint, angle):
    bx = backpoint[0]
    by = backpoint[1]
    r = 0.22

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

def linetraj(startpoint, endpoint, step, i):
    sx = startpoint[0]
    sy = startpoint[1]
    ex = endpoint[0]
    ey = endpoint[1]
    tx = sx+((ex-sx)/step)*i
    ty = sy+((ey-sy)/step)*i

    return [tx, ty]

def lineangletraj(startangle, endangle, step, i):
    angle = startangle + ((endangle-startangle)/step)*i

    return angle

def rectraj(startpoint, endpoint, i):
    sx = startpoint[0]
    sy = startpoint[1]
    ex = endpoint[0]
    ey = endpoint[1]

    if ey>=sy:
        if i == 0:
            tx = sx
            ty = sy
        elif i ==1:
            tx = sx
            ty = sy + ((ey+0.1-sy)/4.0)
        elif i ==2:
            tx = sx
            ty = sy + 2.0*((ey+0.1-sy)/4.0)
        elif i ==3:
            tx = sx
            ty = sy + 3.0*((ey+0.1-sy)/4.0)
        elif i ==4:
            tx = sx
            ty = ey + 0.1
        elif i == 5:
            tx = sx + ((ex-sx)/4.0)
            ty = ey + 0.1
        elif i == 6:
            tx = sx + 2.0*((ex-sx)/4.0)
            ty = ey + 0.1
        elif i == 7:
            tx = sx + 3.0*((ex-sx)/4.0)
            ty = ey + 0.1
        elif i == 8:
            tx = ex
            ty = ey + 0.1
        elif i == 9:
            tx = ex
            ty = ey + 0.03
        elif i == 10:
            tx = ex 
            ty = ey + 0.01
        elif i == 11:
            tx = ex
            ty = ey 

    elif ey<sy:
        if i == 0:
            tx = sx
            ty = sy
        elif i ==1:
            tx = sx
            ty = sy + 0.1
        elif i ==2:
            tx = sx + ((ex-sx)/4.0)
            ty = sy + 0.1
        elif i ==3:
            tx = sx + 2.0*((ex-sx)/4.0)
            ty = sy + 0.1
        elif i ==4:
            tx = sx + 3.0*((ex-sx)/4.0)
            ty = sy + 0.1
        elif i == 5:
            tx = ex
            ty = sy + 0.1
        elif i == 6:
            tx = ex
            ty = ey + 3.0*((sy+0.1-ey)/4.0)
        elif i == 7:
            tx = ex
            ty = ey + 2.0*((sy+0.1-ey)/4.0)
        elif i == 8:
            tx = ex
            ty = ey + 1.0*((sy+0.1-ey)/4.0)
        elif i == 9:
            tx = ex
            ty = ey + 0.03
        elif i == 10:
            tx = ex 
            ty = ey + 0.01
        elif i == 11:
            tx = ex
            ty = ey 

    return [tx, ty]

def up_max_foward(startpoint,endpoint,firstlinkangle,linklength):
    sx = startpoint[0]
    sy = startpoint[1]
    ex = endpoint[0]
    ey = endpoint[1]
    thp = math.atan((ey-sy)/(ex-sx))
    h = math.sqrt(pow((ex - sx),2.0) + pow((ey - sy),2.0))/2.0
    thl = math.acos(h/linklength)
    a = sx + linklength*math.cos(thl + thp)
    b = sy + linklength*math.sin(thl + thp)
    fx = a + linklength*math.cos(firstlinkangle*3.14/180.0)
    fy = b + linklength*math.sin(firstlinkangle*3.14/180.0)

    return [fx,fy]

def frontshoulder_frontfeet(frontfeet,angle):
    xf = frontfeet[0]
    yf = frontfeet[1]

    fx = xf - link1*math.cos(angle*3.14/180.0)
    fy = yf + link2 + link1*math.sin(angle*3.14/180.0)

    return [fx,fy]

def backfeet_backshoulder(backshoulder,yb):
    bx = backshoulder[0]
    by = backshoulder[1]

    th = math.asin((by-yb-link1)/link1)
    xb = bx - link1* math.cos(th)

    return [xb,yb]

    


step = 15
stephalf = 25
step10 = 10
step3 = 3
step5 = 11

rfjoint1,rbjoint1 = inclination_from_front(frontshoulder_frontfeet([-0.05,0],30),0)
prfjoint1,prbjoint1 = rfjoint1,rbjoint1
rfjoint1,rbjoint1 = inclination_from_front(frontshoulder_frontfeet([-0.05,0],30),30)
rfjoint1,rbjoint1 = [rfjoint1[0]+0.1,rfjoint1[1]],[rbjoint1[0]+0.1,rbjoint1[1]]
backfeet = backfeet_backshoulder(rbjoint1,0)

# for i in range(step+1):#0
#     frlink,frth = linkshow2(linetraj(prfjoint1,rfjoint1,step,i),[-0.05,0],lineangletraj(0.0,30.0,step,i))
#     fllink,flth = linkshow2(linetraj(prfjoint1,rfjoint1,step,i),[-0.05,0],lineangletraj(0.0,30.0,step,i))
#     brlink,brth = linkshow(linetraj(prbjoint1,rbjoint1,step,i),backfeet,lineangletraj(0.0,30.0,step,i))
#     bllink,blth = linkshow(linetraj(prbjoint1,rbjoint1,step,i),backfeet,lineangletraj(0.0,30.0,step,i))

#     m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
#     center = (m1 + m2 + m3 + m4 + m5)/6.8

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

# for i in range(step5+1):#1
#     frlink,frth = linkshow2(rfjoint1,[-0.05,0],30)
#     fllink,flth = linkshow2(rfjoint1,[-0.05,0],30)
#     brlink,brth = linkshow(rbjoint1,rectraj(backfeet,[backfeet[0]+0.2,backfeet[1]],i),30)
#     bllink,blth = linkshow(rbjoint1,backfeet,30)

#     m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
#     center = (m1 + m2 + m3 + m4 + m5)/6.8

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

# for i in range(step5+1):#1-2
#     frlink,frth = linkshow2(rfjoint1,rectraj([-0.05,0],[0.1+(step_d*0.0)+0.1,step_h*1.0],i),30)
#     fllink,flth = linkshow2(rfjoint1,[-0.05,0],30)
#     brlink,brth = linkshow(rbjoint1,[backfeet[0]+0.2,backfeet[1]],30)
#     bllink,blth = linkshow(rbjoint1,backfeet,30)

#     m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
#     center = (m1 + m2 + m3 + m4 + m5)/6.8

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

# for i in range(step5+1):#2
#     frlink,frth = linkshow2(rfjoint1,[0.1+(step_d*0.0)+0.1,step_h*1.0],30)
#     fllink,flth = linkshow2(rfjoint1,[-0.05,0],30)
#     brlink,brth = linkshow(rbjoint1,[backfeet[0]+0.2,backfeet[1]],30)
#     bllink,blth = linkshow(rbjoint1,rectraj(backfeet,[backfeet[0]+0.2,backfeet[1]],i),30)

#     m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
#     center = (m1 + m2 + m3 + m4 + m5)/6.8

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

# for i in range(step5+1):#2-2
#     frlink,frth = linkshow2(rfjoint1,[0.1+(step_d*0.0)+0.1,step_h*1.0],30)
#     fllink,flth = linkshow2(rfjoint1,rectraj([-0.05,0],[0.1+(step_d*0.0)+0.1,step_h*1.0],i),30)
#     brlink,brth = linkshow(rbjoint1,[backfeet[0]+0.2,backfeet[1]],30)
#     bllink,blth = linkshow(rbjoint1,[backfeet[0]+0.2,backfeet[1]],30)

#     m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
#     center = (m1 + m2 + m3 + m4 + m5)/6.8

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

prfjoint1,prbjoint1 = rfjoint1,rbjoint1
rfjoint1,rbjoint1 = inclination_from_front(frontshoulder_frontfeet([0.1+(step_d*0.0)+0.1,step_h*1.0],10),30)
rfjoint1,rbjoint1 = [rfjoint1[0]+0.1,rfjoint1[1]],[rbjoint1[0]+0.1,rbjoint1[1]]

# for i in range(step+1):#3
#     frlink,frth = linkshow2(linetraj(prfjoint1,rfjoint1,step,i),[0.1+(step_d*0.0)+0.1,step_h*1.0],30)
#     fllink,flth = linkshow2(linetraj(prfjoint1,rfjoint1,step,i),[0.1+(step_d*0.0)+0.1,step_h*1.0],30)
#     brlink,brth = linkshow(linetraj(prbjoint1,rbjoint1,step,i),[backfeet[0]+0.2,backfeet[1]],30)
#     bllink,blth = linkshow(linetraj(prbjoint1,rbjoint1,step,i),[backfeet[0]+0.2,backfeet[1]],30)

#     m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
#     center = (m1 + m2 + m3 + m4 + m5)/6.8

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

# for i in range(step5+1):#4
#     frlink,frth = linkshow2(rfjoint1,[0.1+(step_d*0.0)+0.1,step_h*1.0],30)
#     fllink,flth = linkshow2(rfjoint1,[0.1+(step_d*0.0)+0.1,step_h*1.0],30)
#     brlink,brth = linkshow(rbjoint1,rectraj([backfeet[0]+0.2,backfeet[1]],[backfeet[0]+0.4,backfeet[1]],i),30)
#     bllink,blth = linkshow(rbjoint1,[backfeet[0]+0.2,backfeet[1]],30)

#     m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
#     center = (m1 + m2 + m3 + m4 + m5)/6.8

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

# for i in range(step5+1):#4-2
#     frlink,frth = linkshow2(rfjoint1,rectraj([0.1+(step_d*0.0)+0.1,step_h*1.0],[0.1+(step_d*1.0)+0.1,step_h*2.0],i),30)
#     fllink,flth = linkshow2(rfjoint1,[0.1+(step_d*0.0)+0.1,step_h*1.0],30)
#     brlink,brth = linkshow(rbjoint1,[backfeet[0]+0.4,backfeet[1]],30)
#     bllink,blth = linkshow(rbjoint1,[backfeet[0]+0.2,backfeet[1]],30)

#     m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
#     center = (m1 + m2 + m3 + m4 + m5)/6.8

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

# for i in range(step5+1):#5
#     frlink,frth = linkshow2(rfjoint1,[0.1+(step_d*1.0)+0.1,step_h*2.0],30)
#     fllink,flth = linkshow2(rfjoint1,[0.1+(step_d*0.0)+0.1,step_h*1.0],30)
#     brlink,brth = linkshow(rbjoint1,[backfeet[0]+0.4,backfeet[1]],30)
#     bllink,blth = linkshow(rbjoint1,rectraj([backfeet[0]+0.2,backfeet[1]],[backfeet[0]+0.4,backfeet[1]],i),30)

#     m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
#     center = (m1 + m2 + m3 + m4 + m5)/6.8

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

# for i in range(step5+1):#5-2
#     frlink,frth = linkshow2(rfjoint1,[0.1+(step_d*1.0)+0.1,step_h*2.0],30)
#     fllink,flth = linkshow2(rfjoint1,rectraj([0.1+(step_d*0.0)+0.1,step_h*1.0],[0.1+(step_d*1.0)+0.1,step_h*2.0],i),30)
#     brlink,brth = linkshow(rbjoint1,[backfeet[0]+0.4,backfeet[1]],30)
#     bllink,blth = linkshow(rbjoint1,[backfeet[0]+0.4,backfeet[1]],30)

#     m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
#     center = (m1 + m2 + m3 + m4 + m5)/6.8

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

prfjoint1,prbjoint1 = rfjoint1,rbjoint1
rfjoint1,rbjoint1 = inclination_from_front(frontshoulder_frontfeet([0.1+(step_d*1.0)+0.1,step_h*2.0],0),30)
rfjoint1,rbjoint1 = [rfjoint1[0],rfjoint1[1]],[rbjoint1[0],rbjoint1[1]]

# for i in range(step+1):#6
#     frlink,frth = linkshow2(linetraj(prfjoint1,rfjoint1,step,i),[0.1+(step_d*1.0)+0.1,step_h*2.0],30)
#     fllink,flth = linkshow2(linetraj(prfjoint1,rfjoint1,step,i),[0.1+(step_d*1.0)+0.1,step_h*2.0],30)
#     brlink,brth = linkshow(linetraj(prbjoint1,rbjoint1,step,i),[backfeet[0]+0.4,backfeet[1]],30)
#     bllink,blth = linkshow(linetraj(prbjoint1,rbjoint1,step,i),[backfeet[0]+0.4,backfeet[1]],30)

#     m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
#     center = (m1 + m2 + m3 + m4 + m5)/6.8

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

# for i in range(step5+1):#7
#     frlink,frth = linkshow2(rfjoint1,[0.1+(step_d*1.0)+0.1,step_h*2.0],30)
#     fllink,flth = linkshow2(rfjoint1,[0.1+(step_d*1.0)+0.1,step_h*2.0],30)
#     brlink,brth = linkshow(rbjoint1,rectraj([backfeet[0]+0.4,backfeet[1]],[backfeet[0]+0.6,backfeet[1]],i),30)
#     bllink,blth = linkshow(rbjoint1,[backfeet[0]+0.4,backfeet[1]],30)

#     m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
#     center = (m1 + m2 + m3 + m4 + m5)/6.8

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

# for i in range(step5+1):#8
#     frlink,frth = linkshow2(rfjoint1,[0.1+(step_d*1.0)+0.1,step_h*2.0],30)
#     fllink,flth = linkshow2(rfjoint1,[0.1+(step_d*1.0)+0.1,step_h*2.0],30)
#     brlink,brth = linkshow(rbjoint1,[backfeet[0]+0.6,backfeet[1]],30)
#     bllink,blth = linkshow(rbjoint1,rectraj([backfeet[0]+0.4,backfeet[1]],[backfeet[0]+0.6,backfeet[1]],i),30)

#     m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
#     center = (m1 + m2 + m3 + m4 + m5)/6.8

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

prfjoint1,prbjoint1 = rfjoint1,rbjoint1
rfjoint1,rbjoint1 = inclination_from_front(frontshoulder_frontfeet([0.1+(step_d*1.0)+0.1,step_h*2.0],0),30)
rfjoint1,rbjoint1 = [rfjoint1[0]+0.1,rfjoint1[1]],[rbjoint1[0]+0.1,rbjoint1[1]]

# for i in range(step+1):#9
#     frlink,frth = linkshow2(linetraj(prfjoint1,rfjoint1,step,i),[0.1+(step_d*1.0)+0.1,step_h*2.0],30)
#     fllink,flth = linkshow2(linetraj(prfjoint1,rfjoint1,step,i),[0.1+(step_d*1.0)+0.1,step_h*2.0],30)
#     brlink,brth = linkshow(linetraj(prbjoint1,rbjoint1,step,i),[backfeet[0]+0.6,backfeet[1]],30)
#     bllink,blth = linkshow(linetraj(prbjoint1,rbjoint1,step,i),[backfeet[0]+0.6,backfeet[1]],30)

#     m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
#     center = (m1 + m2 + m3 + m4 + m5)/6.8

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

# for i in range(step5+1):#10
#     frlink,frth = linkshow2(rfjoint1,rectraj([0.1+(step_d*1.0)+0.1,step_h*2.0],[0.1+(step_d*2.0)+0.1,step_h*3.0],i),30)
#     fllink,flth = linkshow2(rfjoint1,[0.1+(step_d*1.0)+0.1,step_h*2.0],30)
#     brlink,brth = linkshow(rbjoint1,[backfeet[0]+0.6,backfeet[1]],30)
#     bllink,blth = linkshow(rbjoint1,[backfeet[0]+0.6,backfeet[1]],30)

#     m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
#     center = (m1 + m2 + m3 + m4 + m5)/6.8

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

# for i in range(step5+1):#11
#     frlink,frth = linkshow2(rfjoint1,[0.1+(step_d*2.0)+0.1,step_h*3.0],30)
#     fllink,flth = linkshow2(rfjoint1,rectraj([0.1+(step_d*1.0)+0.1,step_h*2.0],[0.1+(step_d*2.0)+0.1,step_h*3.0],i),30)
#     brlink,brth = linkshow(rbjoint1,[backfeet[0]+0.6,backfeet[1]],30)
#     bllink,blth = linkshow(rbjoint1,[backfeet[0]+0.6,backfeet[1]],30)

#     m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
#     center = (m1 + m2 + m3 + m4 + m5)/6.8

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

prfjoint1,prbjoint1 = rfjoint1,rbjoint1
rfjoint1,rbjoint1 = [rfjoint1[0]+0.15,rfjoint1[1]],[rbjoint1[0]+0.15,rbjoint1[1]]

# for i in range(step+1):#12
#     frlink,frth = linkshow2(linetraj(prfjoint1,rfjoint1,step,i),[0.1+(step_d*2.0)+0.1,step_h*3.0],30)
#     fllink,flth = linkshow2(linetraj(prfjoint1,rfjoint1,step,i),[0.1+(step_d*2.0)+0.1,step_h*3.0],30)
#     brlink,brth = linkshow(linetraj(prbjoint1,rbjoint1,step,i),[backfeet[0]+0.6,backfeet[1]],30)
#     bllink,blth = linkshow(linetraj(prbjoint1,rbjoint1,step,i),[backfeet[0]+0.6,backfeet[1]],30)

#     m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
#     center = (m1 + m2 + m3 + m4 + m5)/6.8

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

# for i in range(step5+1):#13
#     frlink,frth = linkshow2(rfjoint1,[0.1+(step_d*2.0)+0.1,step_h*3.0],30)
#     fllink,flth = linkshow2(rfjoint1,[0.1+(step_d*2.0)+0.1,step_h*3.0],30)
#     brlink,brth = linkshow(rbjoint1,[backfeet[0]+0.6,backfeet[1]],30)
#     bllink,blth = linkshow(rbjoint1,rectraj([backfeet[0]+0.6,backfeet[1]],[0.2-step_d,0.0],i),30)

#     m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
#     center = (m1 + m2 + m3 + m4 + m5)/6.8

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

# for i in range(step5+1):#14
#     frlink,frth = linkshow2(rfjoint1,[0.1+(step_d*2.0)+0.1,step_h*3.0],30)
#     fllink,flth = linkshow2(rfjoint1,[0.1+(step_d*2.0)+0.1,step_h*3.0],30)
#     brlink,brth = linkshow(rbjoint1,rectraj([backfeet[0]+0.6,backfeet[1]],[0.2-step_d,0.0],i),30)
#     bllink,blth = linkshow(rbjoint1,[0.2-step_d,0.0],30)

#     m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
#     center = (m1 + m2 + m3 + m4 + m5)/6.8

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

prfjoint1,prbjoint1 = rfjoint1,rbjoint1
rfjoint1,rbjoint1 = inclination_from_front(frontshoulder_frontfeet([0.1+(step_d*2.0)+0.1,step_h*3.0],-30),30)
rfjoint1,rbjoint1 = [rfjoint1[0]+0.1,rfjoint1[1]],[rbjoint1[0]+0.1,rbjoint1[1]]

# for i in range(step+1):#15
#     frlink,frth = linkshow2(linetraj(prfjoint1,rfjoint1,step,i),[0.1+(step_d*2.0)+0.1,step_h*3.0],30)
#     fllink,flth = linkshow2(linetraj(prfjoint1,rfjoint1,step,i),[0.1+(step_d*2.0)+0.1,step_h*3.0],30)
#     brlink,brth = linkshow(linetraj(prbjoint1,rbjoint1,step,i),[0.2-step_d,0.0],30)
#     bllink,blth = linkshow(linetraj(prbjoint1,rbjoint1,step,i),[0.2-step_d,0.0],30)

#     m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
#     center = (m1 + m2 + m3 + m4 + m5)/6.8

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

for i in range(step5+1):#set 1
    frlink,frth = linkshow2(rfjoint1,[0.1+(step_d*2.0)+0.1,step_h*3.0],30)
    fllink,flth = linkshow2(rfjoint1,[0.1+(step_d*2.0)+0.1,step_h*3.0],30)
    brlink,brth = linkshow(rbjoint1,[0.2-step_d,0.0],30)
    bllink,blth = linkshow(rbjoint1,rectraj([0.2-step_d,0.0],[0.1+(step_d*0.0)+0.1-0.02,step_h*1.0],i),30)

    m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
    m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
    m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
    m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
    m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
    center = (m1 + m2 + m3 + m4 + m5)/6.8

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

for i in range(step5+1):#set 2
    frlink,frth = linkshow2(rfjoint1,[0.1+(step_d*2.0)+0.1,step_h*3.0],30)
    fllink,flth = linkshow2(rfjoint1,[0.1+(step_d*2.0)+0.1,step_h*3.0],30)
    brlink,brth = linkshow(rbjoint1,rectraj([0.2-step_d,0.0],[0.1+(step_d*0.0)+0.1-0.02,step_h*1.0],i),30)
    bllink,blth = linkshow(rbjoint1,[0.1+(step_d*0.0)+0.1-0.02,step_h*1.0],30)

    m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
    m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
    m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
    m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
    m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
    center = (m1 + m2 + m3 + m4 + m5)/6.8

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

prfjoint1,prbjoint1 = rfjoint1,rbjoint1
rfjoint1,rbjoint1 = [rfjoint1[0],rfjoint1[1]+0.2],[rbjoint1[0],rbjoint1[1]+0.2]

for i in range(step+1):#set 3
    frlink,frth = linkshow2(linetraj(prfjoint1,rfjoint1,step,i),[0.1+(step_d*2.0)+0.1,step_h*3.0],30)
    fllink,flth = linkshow2(linetraj(prfjoint1,rfjoint1,step,i),[0.1+(step_d*2.0)+0.1,step_h*3.0],30)
    brlink,brth = linkshow(linetraj(prbjoint1,rbjoint1,step,i),[0.1+(step_d*0.0)+0.1-0.02,step_h*1.0],30)
    bllink,blth = linkshow(linetraj(prbjoint1,rbjoint1,step,i),[0.1+(step_d*0.0)+0.1-0.02,step_h*1.0],30)

    m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
    m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
    m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
    m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
    m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
    center = (m1 + m2 + m3 + m4 + m5)/6.8

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

# for i in range(step5+1):#set 4
#     frlink,frth = linkshow2(rfjoint1,rectraj([0.1+(step_d*2.0)+0.1,step_h*3.0],[0.1+(step_d*3.0)+0.1-0.02,step_h*4.0],i),30)
#     fllink,flth = linkshow2(rfjoint1,[0.1+(step_d*2.0)+0.1,step_h*3.0],30)
#     brlink,brth = linkshow(rbjoint1,[0.1+(step_d*0.0)+0.1-0.02,step_h*1.0],30)
#     bllink,blth = linkshow(rbjoint1,[0.1+(step_d*0.0)+0.1-0.02,step_h*1.0],30)

#     m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
#     center = (m1 + m2 + m3 + m4 + m5)/6.8

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

# for i in range(step5+1):#set 5
#     frlink,frth = linkshow2(rfjoint1,[0.1+(step_d*3.0)+0.1-0.02,step_h*4.0],30)
#     fllink,flth = linkshow2(rfjoint1,rectraj([0.1+(step_d*2.0)+0.1,step_h*3.0],[0.1+(step_d*3.0)+0.1-0.02,step_h*4.0],i),30)
#     brlink,brth = linkshow(rbjoint1,[0.1+(step_d*0.0)+0.1-0.02,step_h*1.0],30)
#     bllink,blth = linkshow(rbjoint1,[0.1+(step_d*0.0)+0.1-0.02,step_h*1.0],30)

#     m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
#     center = (m1 + m2 + m3 + m4 + m5)/6.8

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


for i in range(step5+1):#flat 4
    frlink,frth = linkshow2(rfjoint1,rectraj([0.1+(step_d*2.0)+0.1,step_h*3.0],[0.1+(step_d*2.0)+0.4-0.02,step_h*3.0],i),30)
    fllink,flth = linkshow2(rfjoint1,[0.1+(step_d*2.0)+0.1,step_h*3.0],30)
    brlink,brth = linkshow(rbjoint1,[0.1+(step_d*0.0)+0.1-0.02,step_h*1.0],30)
    bllink,blth = linkshow(rbjoint1,[0.1+(step_d*0.0)+0.1-0.02,step_h*1.0],30)

    m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
    m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
    m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
    m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
    m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
    center = (m1 + m2 + m3 + m4 + m5)/6.8

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

for i in range(step5+1):#flat 5
    frlink,frth = linkshow2(rfjoint1,[0.1+(step_d*2.0)+0.4-0.02,step_h*3.0],30)
    fllink,flth = linkshow2(rfjoint1,rectraj([0.1+(step_d*2.0)+0.1,step_h*3.0],[0.1+(step_d*2.0)+0.4-0.02,step_h*3.0],i),30)
    brlink,brth = linkshow(rbjoint1,[0.1+(step_d*0.0)+0.1-0.02,step_h*1.0],30)
    bllink,blth = linkshow(rbjoint1,[0.1+(step_d*0.0)+0.1-0.02,step_h*1.0],30)

    m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
    m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
    m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
    m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
    m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
    center = (m1 + m2 + m3 + m4 + m5)/6.8

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

prfjoint1,prbjoint1 = rfjoint1,rbjoint1
rfjoint1,rbjoint1 = inclination_from_front(frontshoulder_frontfeet([0.1+(step_d*3.0)+0.1-0.02,step_h*4.0],-30),30)
rfjoint1,rbjoint1 = [rfjoint1[0]+0.1,rfjoint1[1]],[rbjoint1[0]+0.1,rbjoint1[1]]

# for i in range(step+1):#set 6
#     frlink,frth = linkshow2(linetraj(prfjoint1,rfjoint1,step,i),[0.1+(step_d*3.0)+0.1-0.02,step_h*4.0],30)
#     fllink,flth = linkshow2(linetraj(prfjoint1,rfjoint1,step,i),[0.1+(step_d*3.0)+0.1-0.02,step_h*4.0],30)
#     brlink,brth = linkshow(linetraj(prbjoint1,rbjoint1,step,i),[0.1+(step_d*0.0)+0.1-0.02,step_h*1.0],30)
#     bllink,blth = linkshow(linetraj(prbjoint1,rbjoint1,step,i),[0.1+(step_d*0.0)+0.1-0.02,step_h*1.0],30)

#     m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
#     m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
#     m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
#     m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
#     m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
#     center = (m1 + m2 + m3 + m4 + m5)/6.8

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

for i in range(step+1):#flat 6
    frlink,frth = linkshow2(linetraj(prfjoint1,rfjoint1,step,i),[0.1+(step_d*2.0)+0.4-0.02,step_h*3.0],30)
    fllink,flth = linkshow2(linetraj(prfjoint1,rfjoint1,step,i),[0.1+(step_d*2.0)+0.4-0.02,step_h*3.0],30)
    brlink,brth = linkshow(linetraj(prbjoint1,rbjoint1,step,i),[0.1+(step_d*0.0)+0.1-0.02,step_h*1.0],30)
    bllink,blth = linkshow(linetraj(prbjoint1,rbjoint1,step,i),[0.1+(step_d*0.0)+0.1-0.02,step_h*1.0],30)

    m1 = frlink[0][0]*0.4 + frlink[1][0]*0.4 + ((frlink[0][0] + frlink[1][0])/2.0) *0.2 + ((frlink[1][0] + frlink[2][0])/2.0) *0.2
    m2 = fllink[0][0]*0.4 + fllink[1][0]*0.4 + ((fllink[0][0] + fllink[1][0])/2.0) *0.2 + ((fllink[1][0] + fllink[2][0])/2.0) *0.2
    m3 = brlink[0][0]*0.4 + brlink[1][0]*0.4 + ((brlink[0][0] + brlink[1][0])/2.0) *0.2 + ((brlink[1][0] + brlink[2][0])/2.0) *0.2
    m4 = bllink[0][0]*0.4 + bllink[1][0]*0.4 + ((bllink[0][0] + bllink[1][0])/2.0) *0.2 + ((bllink[1][0] + bllink[2][0])/2.0) *0.2
    m5 = ((frlink[0][0] + brlink[0][0])/2.0) * 2.0
    center = (m1 + m2 + m3 + m4 + m5)/6.8

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
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-1.5, 5), ylim=(-0.5, 2.5))
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
    line6x = [0.1,0.1,
              0.1+step_d*1.0, 0.1+step_d*1.0,
              0.1+step_d*2.0, 0.1+step_d*2.0,
              0.1+step_d*3.0, 0.1+step_d*3.0,
              0.1+step_d*4.0, 0.1+step_d*4.0,
              0.1+step_d*5.0, 0.1+step_d*5.0,
              0.1+step_d*6.0, 0.1+step_d*6.0,
              0.1+step_d*7.0, 0.1+step_d*7.0,
              0.1+step_d*8.0, 0.1+step_d*8.0,
              0.1+step_d*9.0, 0.1+step_d*9.0,
              0.1+step_d*10.0, 0.1+step_d*10.0,
              0.1+step_d*11.0, 0.1+step_d*11.0]
    line6y = [0,
              step_h*1.0,step_h*1.0,
              step_h*2.0,step_h*2.0,
              step_h*3.0,step_h*3.0,
              step_h*4.0,step_h*4.0,
              step_h*5.0,step_h*5.0,
              step_h*6.0,step_h*6.0,
              step_h*7.0,step_h*7.0,
              step_h*8.0,step_h*8.0,
              step_h*9.0,step_h*9.0,
              step_h*10.0,step_h*10.0,
              step_h*11.0,step_h*11.0,
              step_h*12.0]
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
                              interval=100, blit=True, init_func=init, repeat=False)

# ani.save('double_pendulum.mp4', fps=15)
plt.show()
