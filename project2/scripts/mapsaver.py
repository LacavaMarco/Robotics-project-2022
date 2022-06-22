#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from project2.srv import SaveTrajectory, SaveTrajectoryResponse
import os

cwd = os.getcwd()

# take as input the message from topic /map and save the map and its parameters
def mapCallback(data):
    global resolution
    global x0
    global y0

    width = data.info.width
    height = data.info.height

    resolution = data.info.resolution
    x0 = data.info.origin.position.x
    y0 = data.info.origin.position.y

    print(resolution, width, height, x0, y0)

    # initialize a matrix of zeros with the size of the map image
    global size
    size = (width, height)
    global image
    image = np.zeros(size)

    # load map in variable image
    counter = 0
    for p in data.data:
        image[counter%width][int(counter/width)] = p
        counter+=1


# take as input the position of topic /amcl_pose, then translate and convert it to correctly visualize it on the map image
def trajectoryCallback(amclpose):
    global image

    x_meters = amclpose.pose.pose.position.x
    y_meters = amclpose.pose.pose.position.y

    x_pixels = (x_meters - x0) / resolution
    y_pixels = (y_meters - y0) / resolution
    print(x_meters, y_meters, int(x_pixels), int(y_pixels))

    # set the pixel corresponding to the position of the robot to a unique value to differentiate it from the mapping pixels
    image[int(x_pixels)][int(y_pixels)] = 255
    # cv2.circle sadly doesn't work for us :(
    # image = cv2.circle(image, (int(x_pixels), int(y_pixels)), 0, (0, 0, 255), -1)


# bgr convert the current image, then rotate and save it in the workspace folder as trajectory.png
def handle_savetrajectory(request):
    global image
    filename = cwd + '/trajectory.png'
    print ("Saving image...")

    # r, g and b represent the quantity of red, blue and green for each pixel of the map: each pixel colored during
    # the mapping is converted to grayscale (the lighter a pixel, the higher the probability of the pixel to be occupied),
    # while the pixels representing the trajectory are represented in green
    r = np.zeros(size)
    g = np.zeros(size)
    b = np.zeros(size)

    for i in range(0,101):
        r[image == i] = int(2.55*i)
        g[image == i] = int(2.55*i)
        b[image == i] = int(2.55*i)

    r[image == 255] = 0
    g[image == 255] = 255
    b[image == 255] = 0

    bgr_image = np.dstack([b, g, r])

    cv2.imwrite(filename, cv2.rotate(bgr_image, cv2.ROTATE_90_COUNTERCLOCKWISE))
    return "Trajectory saved in the workspace folder as trajectory.png"


def listener():
    rospy.init_node('map_saver', anonymous=False)

    server = rospy.Service('savetrajectory', SaveTrajectory, handle_savetrajectory)

    rospy.Subscriber("/map", OccupancyGrid, mapCallback)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, trajectoryCallback)

    rospy.spin()


if __name__ == '__main__':
    listener()
