import cv2
import numpy as np
import matplotlib.pyplot as plt
from pylepton import Lepton
from imutils import paths
import imutils
import math as m
import RPi.GPIO as GPIO
import time
from time import sleep


#
# Projectile motion equations for phi (corresponds to the phi servo motor)
# @param initial_velocity [ft/s] initial exit velocity from nozzle
# 
#def projectile():
#initialize variables
#velocity, gravity
v = 70.65
g = 32.2
#increment theta 25 to 60 then find  t, x, z
#define x and z as arrays
phi = np.arange(-m.pi/4, m.pi/5, m.pi/180)
phi_with_xend = np.zeros((len(phi),2))
phi_with_xend[:,0] = np.rad2deg(phi)
#print(phi_with_xend)

t = np.linspace(0, 5, num=100) # Set time as 'continous' parameter.
#print(t)

counter = -1
for i in phi: # Calculate trajectory for every angle
    counter += 1
    x1 = []
    z1 = []
    for k in t:
        x = ((v*k)*np.cos(i)) # get positions at every point in time
        z = 53 + ((v*k)*np.sin(i))-((0.5*g)*(k**2))
        x1.append(x)
        z1.append(z)
    p = [i for i, j in enumerate(z1) if j < 0] # Don't fall through the floor                          
    for i in sorted(p, reverse = True):
        del x1[i]
        del z1[i]
    phi_with_xend[counter,1] = x1[-1]
    #print(counter)
    #print(phi_with_xend)
    x1 = np.true_divide(x1,12)
    z1 = np.true_divide(z1,12)
    plt.plot(x1, z1) # Plot for every angle
    #plt.set_title("Projectile motion of water at various angles")
    plt.legend(np.rad2deg(phi), loc='center left', bbox_to_anchor=(1, 0.5), fontsize = 'xx-small')
    plt.xlabel("x distance [ft]")
    plt.ylabel("z distance [ft]")

    #plt.show() # And show on one graphic
    #print(phi_with_xend) #18x2 output, col1 = phi angle and col2 = distance
    #print(phi_with_xend)
    #return phi_with_xend # in inches

#projectile()