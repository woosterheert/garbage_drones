# import the necessary packages
from imutils import paths
import numpy as np
import imutils
import cv2
import matplotlib.pyplot as plt
from djitellopy import Tello
import time

global counter


def get_frame(drone, output_file=None):
    frame_read = drone.get_frame_read()
    frame_out = frame_read.frame
    if output_file is not None:
        cv2.imwrite(output_file, frame_out)
    return frame_out


def show_image(image):
    while True:
        cv2.imshow('imageWindow', image)
        k = cv2.waitKey(20) & 0xFF
        if k == 27:
            break


def get_width_in_px(contour):
    (x, y), radius = cv2.minEnclosingCircle(contour)
    center = (int(x), int(y))
    radius = int(radius)

    return radius, center


def get_center_of_pad(contour):
    M = cv2.moments(contour)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])

    return cX, cY


def get_contour_of_pad(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray[:int(image.shape[0] / 2), :] = 0
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(gray, 35, 125)
    cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    c = max(cnts, key=cv2.contourArea)

    return c


def draw_results(image, contour, radius, center, cX, cY, ctr):
    plt_image = image.copy()
    cv2.circle(plt_image, center, radius, (0, 255, 0), 2)
    cv2.drawContours(plt_image, [contour], -1, (0, 255, 0), 2)
    cv2.circle(plt_image, (cX, cY), 7, (255, 255, 255), -1)
    plt.imshow(plt_image)
    cv2.imwrite(f'shot{ctr}.png', plt_image)


def calculate_distance(image, ctr):

    # from initial test
    f_px = (2 * 26) * 330 / 20

    # from yesterday
    # f = 2  # mm
    # sensor_width = 2.77  # mm
    # f_px = 960 / sensor_width * f

    contour = get_contour_of_pad(image)
    radius, center = get_width_in_px(contour)
    cX, cY = get_center_of_pad(contour)

    draw_results(image, contour, radius, center, cX, cY, ctr)

    return 20 * f_px / (radius * 2)


if __name__ == '__main__':
    myDrone = Tello()
    myDrone.connect()
    myDrone.takeoff()
    time.sleep(2)
    myDrone.streamon()

    image1 = get_frame(myDrone, 'image1.png')
    c = get_contour_of_pad(image1)
    cx1, cy1 = get_center_of_pad(c)
    print(f'cx1: {cx1}')
    radius, center = get_width_in_px(c)
    draw_results(image1, c, radius,center, cx1, cy1, 3)
    if cx1 < (960 / 2):
        myDrone.rotate_counter_clockwise(25)
    else:
        myDrone.rotate_clockwise(25)
    time.sleep(2)

    image2 = get_frame(myDrone, 'image2.png')
    c = get_contour_of_pad(image2)
    cx2, cy2 = get_center_of_pad(c)
    print(f'cx2: {cx2}')

    dxda = abs(cx2 - cx1) / 25
    if cx2 < (960 / 2):
        dx = int(960/2) - cx2
        da = int(dx / dxda)
        myDrone.rotate_counter_clockwise(da)

    if cx2 >= (960 / 2):
        dx = cx2 - int(960/2)
        da = int(dx / dxda)
        myDrone.rotate_clockwise(da)

    counter = 1
    image = get_frame(myDrone)
    distance1 = calculate_distance(image, 1)
    print(f'distance: {distance1}')

    myDrone.move_forward(100)
    image = get_frame(myDrone)
    distance2 = calculate_distance(image, 2)
    print(f'distance: {distance2}')

    if distance2 < (distance1 - 50):
        myDrone.move_forward(int(distance2))
    else:
        myDrone.move_forward(int(distance1-100))

    myDrone.streamoff()
    myDrone.land()
