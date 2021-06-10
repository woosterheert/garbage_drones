import cv2
from djitellopy import Tello
import numpy as np
import time

start_cs = [0.5, 1.5, 1.2]  # height to be updated after take off
calibration_world_cs = np.array([[4. - start_cs[0], 1. - start_cs[1], 0.],
                                 [4.5 - start_cs[0], 1.5 - start_cs[1], 0.],
                                 [5. - start_cs[0], 2.73 - start_cs[1], 0.],
                                 [3. - start_cs[0], 2.25 - start_cs[1], 0.]])

# calibration_px_coordinates_fl = np.array([[472., 583.], [438., 447.], [411., 210.], [578., 188.]])

f = 2 # mm
sensor_width = 2.77 # mm
f_px = 960/sensor_width * f
camera_matrix = np.array([[f_px, 0., 960/2], [0., f_px, 720/2], [0., 0., 1.]])


def get_frame(drone, output_file=None):
    drone.streamon()
    frame_read = drone.get_frame_read()
    frame_out = frame_read.frame
    if output_file is not None:
        cv2.imwrite(output_file, frame_out)
    drone.streamoff()
    return frame_out


def move_to_height(drone, target_height):
    h = drone.get_height()
    print(f'current height is {h} cm')
    dh = target_height - h
    print(f'moving drone {dh} up')
    drone.move_up(dh)


def calibrate_camera(start_frame):
    point_selection_completed = False
    while not point_selection_completed:
        calibration_px_cs = select_points(start_frame)
        if len(calibration_px_cs) == 4:
            point_selection_completed = True
        else:
            print('You must select exactly four points')

    _, rvec, tvec = cv2.solvePnP(calibration_world_cs, np.array(calibration_px_cs, dtype=np.float32), camera_matrix,
                                 None, flags=cv2.SOLVEPNP_ITERATIVE)
    r_mtx, _ = cv2.Rodrigues(rvec)
    return tvec, r_mtx


def select_points(start_frame):
    px_coords = []

    def get_px_coordinates(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            px_coords.append([y, x])
            print(px_coords)

    cv2.namedWindow('selectionWindow')
    cv2.setMouseCallback('selectionWindow', get_px_coordinates)

    while True:
        cv2.imshow('selectionWindow', start_frame)
        k = cv2.waitKey(20) & 0xFF
        if k == 27:
            break

    return px_coords


def load_calibration():
    calibration_px_cs = [[522, 592], [489, 459], [454, 226], [643, 209]]
    _, rvec, tvec = cv2.solvePnP(calibration_world_cs, np.array(calibration_px_cs, dtype=np.float32), camera_matrix,
                                 None, flags=cv2.SOLVEPNP_ITERATIVE)
    r_mtx, _ = cv2.Rodrigues(rvec)
    return tvec, r_mtx


def px_to_world(u, v):

    px_mtx = np.array([u, v, 1.]).reshape(3,1)
    left_mtx = np.matmul(np.matmul(np.linalg.inv(r_mtx), np.linalg.inv(camera_matrix)), px_mtx)
    right_mtx = np.matmul(np.linalg.inv(r_mtx), tvec)
    s = (0 + right_mtx[2, 0]) / left_mtx[2, 0]
    xyz = np.matmul(r_mtx, s * np.matmul(np.linalg.inv(camera_matrix), px_mtx) - tvec)

    return xyz


if __name__ == '__main__':
    myDrone = Tello()
    myDrone.connect()
    myDrone.takeoff()
    time.sleep(3)
    move_to_height(myDrone, 100)
    start_cs[2] = myDrone.get_height()

    start_frame = get_frame(myDrone, 'start_frame.png')
    print('select the four calibration points')
    tvec, r_mtx = calibrate_camera(start_frame)

    # tvec, r_mtx = load_calibration()
    # start_frame = cv2.imread('start_frame.png')

    print('select destination')
    destination_px = select_points(start_frame)[0]
    destination_xyz = px_to_world(destination_px[0], destination_px[1])
    dx = int(destination_xyz[0][0] * 100)
    dy = int(destination_xyz[1][0] * 100)
    _ = input(f'planning to move {dx} forward and {dy} sideways, safe to move forward?')
    myDrone.move_forward(dx)
    if dy > 0:
        myDrone.move_left(dy)
    else:
        myDrone.move_right(dy)
    myDrone.land()
