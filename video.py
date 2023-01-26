import time
import cv2
from cv2 import aruco

import os
import numpy as np

import olympe
from olympe.video.pdraw import Pdraw, PdrawState
from olympe.video.renderer import PdrawRenderer

RTSP_URL = 'rtsp://192.168.42.1/live'
os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'rtsp_transport;udp'
DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1")
### --- aruco設定 --- ###
dict_aruco = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters_create()

target_found = False
start_processing = False


def test_takeoff(drone):
    print("--------------------test_takeoff--------------------")
    assert drone(TakeOff()).wait().success()
    time.sleep(3)


def go(distance):
    os.system("python3 go.py -m {}".format(distance))


def height(z):
    os.system("python3 height.py -m {}".format(z))


def test_move(drone, F, H):
    print("--------------------test_move--------------------")
    assert drone(
        extended_move_by(F, 0, -H, 0, 0.7, 0.7, 0.7)
    ).wait().success()


def test_landing(drone):
    print("--------------------test_landing--------------------")
    drone(Landing()).wait().success()
    drone.disconnect()


def frame_processing(frame):
    if not start_processing:
        return
    global target_found
    try:
        # convert frame to rgb'
        cv2_convert_flag = {
            olympe.VDEF_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.VDEF_NV12: cv2.COLOR_YUV2BGR_NV12
        }[frame.format()]
        gray = cv2.cvtColor(frame.as_ndarray(), cv2_convert_flag)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, dict_aruco, parameters=parameters)
        frame_markers = aruco.drawDetectedMarkers(frame.as_ndarray(), corners, ids)
        cv2.imshow('frame', frame_markers)
        list_ids = list(np.ravel(ids))
        list_ids.sort()
        print(list_ids)

        if list_ids[0] == 0:
            index = np.where(ids == 0)[0][0]  # num_id が格納されているindexを抽出
            cornerUL = corners[index][0][0]
            cornerUR = corners[index][0][1]
            cornerBR = corners[index][0][2]
            cornerBL = corners[index][0][3]

            center = [(cornerUL[0] + cornerBR[0]) / 2, (cornerUL[1] + cornerBR[1]) / 2]

            print('左上 : {}'.format(cornerUL))
            print('右上 : {}'.format(cornerUR))
            print('右下 : {}'.format(cornerBR))
            print('左下 : {}'.format(cornerBL))
            print('中心 : {}'.format(center))
            time.sleep(1)
            print("着陸体制に入ります！！")
            # test_landing(drone)
            # if list_ids[-1] == 4 and len(list_ids) == 5:
            #     print("--------------------------着陸--------------------------")
            #     target_found = True
        if cv2.waitKey(1) & 0xFF == ord('q'):
            # cap.release()
            pass
    except cv2.error as e:
        print(e)
    except Exception as ne:
        print(ne)


if __name__ == '__main__':
    # connec drone
    drone = olympe.Drone(DRONE_IP)
    drone.connect()

    pdraw = Pdraw()
    pdraw.set_callbacks(raw_cb=frame_processing)

    pdraw.play(url=RTSP_URL)

    assert pdraw.wait(PdrawState.Playing, timeout=5)
    print('doe initialize')

    start_processing = True
    try:
        while True:
            if target_found:
                start_processing = False
                break
    except Exception as e:
        print(e)
        test_landing(drone)