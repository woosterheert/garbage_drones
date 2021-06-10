import cv2
from djitellopy import Tello


def get_frame(drone, output_file=None):
    drone.streamon()
    frame_read = drone.get_frame_read()
    frame_out = frame_read.frame
    if output_file is not None:
        cv2.imwrite(output_file, frame_out)
    drone.streamoff()
    return frame_out


def show_one_frame(frame_in):
    while True:
        cv2.imshow('DroneViewer', frame_in)
        if cv2.waitKey(0) == 27:
            break


def move_to_height(drone, target_height):
    h = drone.get_height()
    print(f'current height is {h} cm')
    dh = target_height - h
    print(f'moving drone {dh} up')
    drone.move_up(dh)
    h = drone.get_height()
    print(f'current height is {h} cm')


def select_points():
    px_coords = []

    def get_px_coordinates(event, x, y, flags, param):
        if cv2.EVENT_LBUTTONDOWN:
            px_coords.append((x, y))
            print(px_coords)

    cv2.namedWindow('calibrationWindow')
    cv2.setMouseCallback('calibrationWindow', get_px_coordinates)

    while True:
        cv2.imshow('calibrationWindow', cv2.imread('calibration_frame.png'))
        k = cv2.waitKey(20) & 0xFF
        if k == 27:
            break

    return px_coords


if __name__ == '__main__':
    # myDrone = Tello()
    # myDrone.connect()
    # myDrone.takeoff()
    # move_to_height(myDrone, 120)
    # frame = get_frame(myDrone, 'calibration_frame.png')
    # # show_one_frame(frame)
    # myDrone.land()
    select_points()
