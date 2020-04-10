# import the necessary packages
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
#from motion_calc import phi_with_xend
from scipy.spatial import distance as dist
import RPi.GPIO as GPIO     # Import GPIO Library
import time                 # Import time library for a delay
from time import sleep

#
# Capture image data
# capture() is a 12-bit non-normalized raw sensor data
# we constrast extend it since bandwidth tends to be narrow
#
def capture_image(file_name):
	print("Capturing image named %s.jpg..." % file_name)
	with Lepton() as l:
		a,_ = l.capture()
		cv2.normalize(a, a, 0, 65535, cv2.NORM_MINMAX) #extend constrast
		np.right_shift(a, 8, a) #fit data into 8 bits
		print("Image capture complete!")
		cv2.imwrite(file_name + ".jpg", np.uint8(a)) #write it
		return file_name + ".jpg"

#
# Find brightest spot in image
#
def bright(img, radius):
	# load the image and convert it to grayscale
	image = cv2.imread(img) # load the image
	orig = image.copy() # clone original image
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #grayscale image

	# apply a Gaussian blur to the image then find the brightest region
	# first apply a Gaussian blue to the image to remove high frequency noise
	gray = cv2.GaussianBlur(gray, (radius, radius), 0)
	# apply Gaussian blur with the supplied radius
	# call cv2.minMaxLoc to find brightest pixel in image
	(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray) 
	image = orig.copy()
	cv2.circle(image, maxLoc, radius, (255, 0, 0), 2)

	# display the results of our newly improved method
	#cv2.imshow("Brighest pixel value", image)
	cv2.imwrite("brightest_pixel.jpg", image)
	cv2.waitKey(0)

	print("Pixel location for maxVal is: %s" % (maxLoc,))
	return maxLoc

#
# Order points
#
def order_points(pts):
	# sort the points based on their x-coordinates
	xSorted = pts[np.argsort(pts[:, 0]), :]
 
	# grab the left-most and right-most points from the sorted
	# x-roodinate points
	leftMost = xSorted[:2, :]
	rightMost = xSorted[2:, :]
 
	# now, sort the left-most coordinates according to their
	# y-coordinates so we can grab the top-left and bottom-left
	# points, respectively
	leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
	(tl, bl) = leftMost
 
	# now that we have the top-left coordinate, use it as an
	# anchor to calculate the Euclidean distance between the
	# top-left and right-most points; by the Pythagorean
	# theorem, the point with the largest distance will be
	# our bottom-right point
	D = dist.cdist(tl[np.newaxis], rightMost, "euclidean")[0]
	(br, tr) = rightMost[np.argsort(D)[::-1], :]
 
	# return the coordinates in top-left, top-right,
	# bottom-right, and bottom-left order
	return np.array([tl, tr, br, bl], dtype="float32")

# 
# @arg image: to apply perspective transform
# @arg pts: to be ordered
# @return warped image
#
def four_point_transform(image,pts):
	# obtain a consistent order of the points
	# unpack them individually
	print("Applying image distortion...")
	rect = order_points(pts)
	(tl, tr, br, bl) = rect

	# compute width of the new image
	# max distance between br/bl or tr/tl x-coordinates
	widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
	widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
	maxWidth = max(int(widthA), int(widthB))

	# compute height of new image
	# max distance between tr/br or tl/bl y-coordinates
	heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
	heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
	maxHeight = max(int(heightA), int(heightB))

	# now we have dimensions of new image

	# construct set of destination points to obtain top-down view
	# --specify points in tl,tr,br,bl order
	dst = np.array(
		[
		[0,0],
		[maxWidth - 1, 0],
		[maxWidth - 1, maxHeight - 1],
		[0, maxHeight - 1]
		],
		dtype = "float32")

	# compute the perscpective transform matrix and then apply it
	M = cv2.getPerspectiveTransform(rect,dst)
	warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))

	print("Distortion complete!")

	# return the warped image
	return warped

# 
# Take coordinate output and determine distance & theta from hose
# 
def maxLoc_to_distance(maxLoc):
	x_coord, y_coord = maxLoc 
	x = 1.5*(39-x_coord) #a
	y = (59-y_coord)*2.5 + 19 #19 distanced from nozzle to origin nozzle  #46.4324 #b
	c = np.sqrt(np.power(x,2) + np.power(y,2) - 2*x*y*np.cos(90))
	theta = np.arccos((np.power(x,2) + np.power(c,2) - np.power(y,2))/(2*x*c))
	print(c)
	print(theta)
	return c, theta

# 
# Projectile motion equations for phi (corresponds to the phi servo motor)
# @param initial_velocity [ft/s] initial exit velocity from nozzle
# 
def projectile():
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

    plt.show() # And show on one graphic
    cv2.waitKey(0)
    print(phi_with_xend) #18x2 output, col1 = phi angle and col2 = distance
    #print(phi_with_xend)
    return phi_with_xend # in inches

def sort_2d_array(angle2distance_array):
    return (angle2distance_array[angle2distance_array[:,1].argsort()])

def calculate_phi(object_distance, sorted_a2d_rray):
    distance = sorted_a2d_rray[:,1]
    x = len(distance)
    counter = 0
    while counter < x:
        if((distance[counter] <= object_distance) and (object_distance <= distance[counter+1])):
            desired_angle = sorted_a2d_rray[counter,0]
            return desired_angle
            break;
        else:
            counter += 1


"""
-----------------------------------------------------------------------------------------------------------
-------------------------------------------- MAIN  --------------------------------------------------------
-----------------------------------------------------------------------------------------------------------
"""

# capture image
orig_image = capture_image("11_14_test_4")

# load the image
image = cv2.imread("11_14_test_4.jpg")

# grab the source coordinates (i.e. the list of (e,y) coords)
PTS = "[(18,0), (61,0), (79,59), (0,59)]"

# apply four point transform to obtain top-down view of the image
pts = np.array(eval(PTS), dtype = "float32")
warped_image = four_point_transform(image, pts)
cv2.imwrite("warped_image.jpg", warped_image)

# show the original and warped images
cv2.imshow("Original", image)
cv2.imshow("Warped", warped_image)
cv2.waitKey(0)


coordinates = bright("warped_image.jpg",5)
print(coordinates)
distance, theta = maxLoc_to_distance(coordinates)
angle2distance_array = projectile()
sorted_a2d_array = sort_2d_array(angle2distance_array)
phi = calculate_phi(distance,sorted_a2d_array)

print("Desired theta angle is: %s" % theta)
print("Desired phi angle is: %s" % phi)
print()
"""
thetaServoPin = 8 #blue data
phiServoPin = 10 #yellow data
GPIO.setmode(GPIO.BOARD) #set the naming mode to board mode

GPIO.setup(thetaServoPin, GPIO.OUT)
GPIO.setup(phiServoPin, GPIO.OUT)

theta = GPIO.PWM(thetaServoPin, 50)
phi = GPIO.PWM(phiServoPin, 50)
theta.start(0)
phi.start(0)

#SetThetaAngle(?)
#SetPhiAngle(?)
"""
