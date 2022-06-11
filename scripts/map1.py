#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jun 11 18:04:43 2022

@author: paul
"""
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jun 11 15:29:57 2022

@author: paul
"""
import matplotlib.pyplot as plt
import numpy as np
from skimage.feature import peak_local_max
from skimage.morphology import watershed
from scipy import ndimage,signal
import cv2 as cv
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped, Twist, PoseWithCovarianceStamped, Pose, Point, Vector3, Quaternion, PoseStamped

import yaml

import glob

from shapely.geometry import Point, Polygon
from shapely.geometry.polygon import LinearRing, LineString
import pandas as pd
import matplotlib.pyplot as plt
import os.path
import math


import copy
from shapely.geometry import Point, Polygon
from shapely.geometry.polygon import LinearRing, LineString
from skimage.morphology import medial_axis, skeletonize

with open(r'big_map_hr.yaml') as file:
    # The FullLoader parameter handles the conversion from YAML
    # scalar values to Python the dictionary format
    param = yaml.load(file, Loader=yaml.FullLoader)

resolution = param['resolution']
origin = param['origin']
#%% Centerline extraction
with open(param['image'], 'rb') as pgmf:
    im = plt.imread(pgmf)
    
# plt.imshow(im)



ima = np.array(im)
ima[ima<254]=0

K = np.ones((6,6))/36

f1 = signal.convolve2d(ima, K, boundary='symm', mode='same')

f1[f1<253]=0
f1[f1>=253]=1

f1.astype

edt = ndimage.distance_transform_edt(ima)

edt1 = copy.deepcopy(edt)


# Compute the medial axis (skeleton) and the distance transform
skel, distance = medial_axis(f1, return_distance=True)

# Compare with other skeletonization algorithms
skeleton = skeletonize(f1)
skeleton_lee = skeletonize(f1, method='lee')

# Distance to the background for pixels of the skeleton
dist_on_skel = distance * skel

fig, axes = plt.subplots(2, 2, figsize=(8, 8), sharex=True, sharey=True)
ax = axes.ravel()

ax[0].imshow(f1, cmap=plt.cm.gray)
ax[0].set_title('original')
ax[0].axis('off')

ax[1].imshow(dist_on_skel, cmap='magma')
ax[1].contour(f1, [0.5], colors='w')
ax[1].set_title('medial_axis')
ax[1].axis('off')

ax[2].imshow(skeleton, cmap=plt.cm.gray, interpolation='nearest')
ax[2].set_title('skeletonize')
ax[2].axis('off')

ax[3].imshow(skeleton_lee, cmap=plt.cm.gray, interpolation='nearest')
ax[3].set_title('skeletonize_3d')
ax[3].axis('off')


y,x = np.where(skeleton_lee)
x=list(x)
y=list(y)

n= len(x)
xs=[x.pop(0)]
ys=[y.pop(0)]
i=0
while len(x)>0:
    dx = [xs[-1] - icx for icx in x]
    dy = [ys[-1] - icy for icy in y]
    d = [math.sqrt(idx ** 2 + idy ** 2)for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))
    
    if min(d)<10 and i%5==0:
        xs.append(x.pop(ind))
        ys.append(y.pop(ind))
    else :
        x.pop(ind)
        y.pop(ind)
    i+=1
    
    

fig, ((ax1, ax2, ax3), (ax4, ax5, ax6)) = plt.subplots(2, 3)
fig.suptitle('Sharing x per column, y per row')
imgResult = cv.cvtColor(ima,cv.COLOR_GRAY2BGR)
ax1.imshow(imgResult)
edt = ndimage.distance_transform_edt(ima)
ax2.imshow(edt)
ax3.imshow(f1)
ax4.plot(xs,ys, 'r+')
# ax4.plot(Y,X, 'b+')
ax4.imshow(imgResult)

xm=[]
ym=[]

for i in range(len(xs)):
    xim=xs[i]*resolution + origin[0]
    yim=(im.shape[0]-1-ys[i])*resolution + origin[1]
    xm.append(xim)
    ym.append(yim)

ax5.plot(xm,ym, 'g')

xf = signal.savgol_filter(xm, 15,8)
yf = signal.savgol_filter(ym, 15,8)

ax6.plot(xf,yf, 'g')

xm = xf
ym = yf

center_line=[]
inner_border=[]
outer_border=[]

width = 3

for i in range(len(xm)):
    
    vect = (xm[i]-xm[(i+width)%len(xm)],ym[i]-ym[(i+width)%len(xm)])
    vect_norm = vect/np.sqrt(vect[0]**2 + vect[1]**2)
    
    center_line.append((xm[i],ym[i]))
    per_vect = np.array((vect_norm[1],-vect_norm[0]))
    inner_border.append((xm[i],ym[i])-per_vect*resolution*edt[ys[i],xs[i]])
    outer_border.append((xm[i],ym[i])+per_vect*resolution*edt[ys[i],xs[i]])
     
center_line.append(center_line[0])
inner_border.append(inner_border[0])
outer_border.append(outer_border[0])   

print(min(outer_border[1]),min(inner_border[1]))

if min(outer_border[0])>min(inner_border[0]):
    a = outer_border
    outer_border = inner_border
    inner_border = a


l_center_line = LineString(center_line)
l_inner_border = LineString(inner_border)
l_outer_border = LineString(outer_border)
road_poly = Polygon(np.vstack((l_outer_border, np.flipud(l_inner_border))))
print("Is loop/ring? ", l_center_line.is_ring)
road_poly

def plot_coords(ax, ob):                                                        
    x, y = ob.xy                                                                
    ax.plot(x, y, '.', color='#999999', zorder=1)                               
                                                                                
def plot_bounds(ax, ob):                                                        
    x, y = zip(*list((p.x, p.y) for p in ob.boundary))                          
    ax.plot(x, y, '.', color='#000000', zorder=1)                               
                                                                                
def plot_line(ax, ob):                                                          
    x, y = ob.xy                                                                
    ax.plot(x, y, color='cyan', alpha=0.7, linewidth=3, solid_capstyle='round', zorder=2)
                                                                                
def print_border(ax, waypoints, inner_border_waypoints, outer_border_waypoints):
    line = LineString(waypoints)                                                
    plot_coords(ax, line)                                                       
    plot_line(ax, line)                                                         
                                                                                
    line = LineString(inner_border_waypoints)                                   
    plot_coords(ax, line)                                                       
    plot_line(ax, line)                                                         
                                                                                
    line = LineString(outer_border_waypoints)                                   
    plot_coords(ax, line)                                                       
    plot_line(ax, line)     

fig1 = plt.figure(1, figsize=(16, 10))
ax = fig1.add_subplot(111, facecolor='black')
plt.axis('equal')
print_border(ax, center_line, inner_border, outer_border)


#%% Optimization
# From https://github.com/e-koch/ewky_scripts/blob/master/curvature.py

# The MIT License (MIT)
#
# Copyright (c) 2014 Eric Koch
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

def menger_curvature(pt1, pt2, pt3, atol=1e-3):

    vec21 = np.array([pt1[0]-pt2[0], pt1[1]-pt2[1]])
    vec23 = np.array([pt3[0]-pt2[0], pt3[1]-pt2[1]])

    norm21 = np.linalg.norm(vec21)
    norm23 = np.linalg.norm(vec23)

    theta = np.arccos(np.dot(vec21, vec23)/(norm21*norm23))
    if np.isclose(theta-np.pi, 0.0, atol=atol):
        theta = 0.0

    dist13 = np.linalg.norm(vec21-vec23)

    return 2*np.sin(theta) / dist13


# Number of times to iterate each new race line point
# keep this at 3-8 for best balance of performance and desired result
XI_ITERATIONS=4

# Number of times to scan the entire race track to iterate
# 500 will get a good start, 1500 will be closer to optimal result
LINE_ITERATIONS=500

def improve_race_line(old_line, inner_border, outer_border):
    '''Use gradient descent, inspired by K1999, to find the racing line'''
    # start with the center line
    new_line = copy.deepcopy(old_line)
    ls_inner_border = Polygon(inner_border)
    ls_outer_border = Polygon(outer_border)
    for i in range(0,len(new_line)):
        xi = new_line[i]
        npoints = len(new_line)
        prevprev = (i - 2 + npoints) % npoints
        prev = (i - 1 + npoints) % npoints
        nexxt = (i + 1 + npoints) % npoints
        nexxtnexxt = (i + 2 + npoints) % npoints
        #print("%d: %d %d %d %d %d" % (npoints, prevprev, prev, i, nexxt, nexxtnexxt))
        ci = menger_curvature(new_line[prev], xi, new_line[nexxt])
        c1 = menger_curvature(new_line[prevprev], new_line[prev], xi)
        c2 = menger_curvature(xi, new_line[nexxt], new_line[nexxtnexxt])
        target_ci = (c1 + c2) / 2
        #print("i %d ci %f target_ci %f c1 %f c2 %f" % (i, ci, target_ci, c1, c2))

        # Calculate prospective new track position, start at half-way (curvature zero)
        xi_bound1 = copy.deepcopy(xi)
        xi_bound2 = ((new_line[nexxt][0] + new_line[prev][0]) / 2.0, (new_line[nexxt][1] + new_line[prev][1]) / 2.0)
        p_xi = copy.deepcopy(xi)
        for j in range(0,XI_ITERATIONS):
            p_ci = menger_curvature(new_line[prev], p_xi, new_line[nexxt])
            #print("i: {} iter {} p_ci {} p_xi {} b1 {} b2 {}".format(i,j,p_ci,p_xi,xi_bound1, xi_bound2))
            if np.isclose(p_ci, target_ci):
                break
            if p_ci < target_ci:
                # too flat, shrinking track too much
                xi_bound2 = copy.deepcopy(p_xi)
                new_p_xi = ((xi_bound1[0] + p_xi[0]) / 2.0, (xi_bound1[1] + p_xi[1]) / 2.0)
                if Point(new_p_xi).within(ls_inner_border) or not Point(new_p_xi).within(ls_outer_border):
                    xi_bound1 = copy.deepcopy(new_p_xi)
                else:
                    p_xi = new_p_xi
            else:
                # too curved, flatten it out
                xi_bound1 = copy.deepcopy(p_xi)
                new_p_xi = ((xi_bound2[0] + p_xi[0]) / 2.0, (xi_bound2[1] + p_xi[1]) / 2.0)

                # If iteration pushes the point beyond the border of the track,
                # just abandon the refinement at this point.  As adjacent
                # points are adjusted within the track the point should gradually
                # make its way to a new position.  A better way would be to use
                # a projection of the point on the border as the new bound.  Later.
                if Point(new_p_xi).within(ls_inner_border) or not Point(new_p_xi).within(ls_outer_border):
                    xi_bound2 = copy.deepcopy(new_p_xi)
                else:
                    p_xi = new_p_xi
        new_xi = p_xi
        # New point which has mid-curvature of prev and next points but may be outside of track
        #print((new_line[i], new_xi))
        new_line[i] = new_xi
    return new_line



print(len(center_line))
# start along centerline of track
race_line = copy.deepcopy(center_line[:-1])  # Use this for centerline being outer bound
for i in range(LINE_ITERATIONS):
    race_line = improve_race_line(race_line, inner_border, outer_border)
    if i % 20 == 0: print("Iteration %d" % i)




# need to put duplicate point race_line[0] at race_line[-1] to make a closed loops
loop_race_line = np.append(race_line, [race_line[0]], axis=0)

# These should be the same
print("These should be the same: ", (len(center_line), loop_race_line.shape))
print("Original centerline length: %0.2f" % l_center_line.length)
print("New race line length: %0.2f" % LineString(loop_race_line).length)

fig = plt.figure(1, figsize=(16, 10))
ax = fig.add_subplot(111, facecolor='black')
plt.axis('equal')
print_border(ax, loop_race_line, inner_border, outer_border)

if __name__ == '__main__':
    try:
        # init node
        rospy.init_node('pure_pursuit_with_avoidance')
        rate = rospy.Rate(15)
        pub_path = rospy.Publisher('mcp_path', Path, queue_size=10)
        # display the path to the look ahead point
        msg_path = Path()
        msg_path.header.frame_id = 'map'
        
        
            
        for i,position in enumerate(loop_race_line):
            
            
            pose = PoseStamped()
            
            pose.header.frame_id = "map"
            pose.header.seq = i
            
            print(position)
            pose.pose.position.x = position[0]
            pose.pose.position.y = position[1]
            
            msg_path.poses.append(pose)
            
        pub_path.publish(msg_path)
        while not rospy.is_shutdown():
            pub_path.publish(msg_path)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass



