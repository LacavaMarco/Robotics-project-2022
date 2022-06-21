#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from project2.srv import SaveTrajectory, SaveTrajectoryResponse
import os

cwd = os.getcwd()

def mapCallback(data):
    global resolution
    global x0
    global y0
    global width
    global height

    width = data.info.width
    height = data.info.height

    resolution = data.info.resolution
    x0 = data.info.origin.position.x
    y0 = data.info.origin.position.y

    # initialize a matrix of zeros with the size of the map image
    size = (width, height)
    global image
    image = np.zeros(size)

    # load map image in variable
    counter = 0
    for p in data.data:
      image[counter%width][int(counter/width)] = p
      counter+=1

    image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)


def trajectoryCallback(amclpose):
    global image
    x_meters = amclpose.pose.pose.position.x
    y_meters = amclpose.pose.pose.position.y

    # forse da invertire width e height
    x_pixels = (x_meters + x0) / resolution + width/2
    y_pixels = (y_meters + y0) / resolution + height/2
    # print(x_pixels, y_pixels)

    image = cv2.circle(image, (np.float32(x_pixels), np.float32(y_pixels)), 500, (0, 0, 255), -1)
    cv2.imshow('Image', image)
    print(x_pixels, y_pixels)

    # image = cv2.circle(image, center=(np.float32(x_pixels), np.float32(y_pixels)), radius=1, color=(0, 0, 255), thickness=-1)


def handle_savetrajectory(request):
    global image
    filename = cwd + '/trajectory.png'
    print ("Saving image")
    cv2.imwrite(filename, image)
    return "Trajectory saved in the workspace folder as trajectory.png"


def listener():
    rospy.init_node('map_saver', anonymous=False)

    server = rospy.Service('savetrajectory', SaveTrajectory, handle_savetrajectory)

    rospy.Subscriber("/map", OccupancyGrid, mapCallback)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, trajectoryCallback)

    rospy.spin()


if __name__ == '__main__':
    listener()
