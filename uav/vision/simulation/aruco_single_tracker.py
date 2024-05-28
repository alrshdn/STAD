import time
import math

import numpy as np

import cv2 as cv
from cv2 import aruco


# 180 deg rotation matrix around the x axis
R_FLIP = np.eye(3, dtype=np.float32)
R_FLIP[1, 1] = -1.0
R_FLIP[2, 2] = -1.0


def internal_is_rotation_matrix(R: np.ndarray):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# `estimatePoseSingleMarkers` is not available in openCV 4.9.* so this is only to compensate for previous versions.
def internal_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    marker_points = np.array([
        [-marker_size / 2, marker_size / 2, 0],
        [marker_size / 2, marker_size / 2, 0],
        [marker_size / 2, -marker_size / 2, 0],
        [-marker_size / 2, -marker_size / 2, 0]], 
        dtype=np.float32
    )

    rvecs = []
    tvecs = []

    for c in corners:
        _, rvec, tvec = cv.solvePnP(marker_points, c, mtx, distortion, False, cv.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(rvec)
        tvecs.append(tvec)

    return rvecs, tvecs


class ArucoSingleTracker:
    # source type flags (currently unused)
    SRC_OPENCV = 0
    SRC_GST = 1

    # detection result flags
    DET_TARGET_NOT_FOUND = 0
    DET_TARGET_FOUND = 1

    def __init__(self, id_to_find, marker_size, source, camera_size=[640, 640]):
        # detection properties
        self._id_to_find = id_to_find
        self._marker_size = marker_size

        # choosing dictionary for valid markers
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        # default parameters
        parameters = aruco.DetectorParameters()
        # setting up the markers to use and parameters
        self._detector = aruco.ArucoDetector(dictionary=aruco_dict, detectorParams=parameters)

        # detection calibration
        calib_data_path = "MultiMatrix.npz"
        calib_data = np.load(calib_data_path)
        self._camera_matrix = calib_data["camMatrix"]
        #self._camera_distortion = calib_data["distCoef"]
        self._camera_distortion = np.array([0, 0, 0, 0], dtype=np.float32)


        # video source
        self._video = source
        self._camera_size = camera_size

        # initially undetected
        self._is_detected = False

        # time and frames per second (FPS) readings
        self._t_read = time.time()
        self._t_detect = self._t_read
        self._fps_read = 0.0
        self._fps_detect = 0.0
    
    

    # Define the rotation matrix in Euler angles
    def _rotation_matrix_to_euler_angles(self, R: np.ndarray):
        assert (internal_is_rotation_matrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])


    # for monitoring reading speed (bottlenecked by detection speed)
    def _update_fps_read(self):
        t = time.time()
        self._fps_read = 1.0 / (t - self._t_read)
        self._t_read = t


    # for monitoring detection speed
    def _update_fps_detect(self):
        t = time.time()
        self._fps_detect = 1.0 / (t - self._t_detect)
        self._t_detect = t


    def track_once(self, draw_on_frame=False):
        while True:
            ret, frame = self._video.read()
            if ret:
                break

        # creating a grayscale version of the  frame
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # detecting markers using the Aruco library
        marker_corners, ids, _ = self._detector.detectMarkers(gray_frame)

        if ids is not None and self._id_to_find in ids:
            index = np.argwhere(ids == self._id_to_find)[0][0]
            marker_corner = marker_corners[0]

            marker_found = True
            self._update_fps_detect()

            # estimating pose of found markers
            rvecs, tvecs = internal_estimatePoseSingleMarkers(
                    marker_corners,
                    self._marker_size,
                    self._camera_matrix,
                    self._camera_distortion
                    )

            # tracking wanted marker (the marker with correct id)
            idx = np.where(ids == self._id_to_find)[0][0]
            rvec = rvecs[idx]
            tvec = tvecs[idx]

            x, y, z = tvec[0][0], tvec[1][0], tvec[2][0]

            # using Rodrigues rotation
            R_ct = np.matrix(cv.Rodrigues(rvec)[0])
            R_tc = R_ct.T

            # getting  marker's attitude in terms of roll, pitch, yaw
            roll_marker, pitch_marker, yaw_marker = \
                    self._rotation_matrix_to_euler_angles(R_FLIP @ R_tc)
            yaw = math.degrees(yaw_marker)

            # visualizing
            if draw_on_frame:
                # getting camera position in cartesian coordinates (x, y, z)
                tvec_mat = np.reshape(tvec, (3, 1))
                pos_camera = -R_tc @ tvec_mat

                cv.polylines(frame, [marker_corner.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)
                corners = marker_corner.reshape(4, 2).astype(int)
                top_right = corners[0].ravel()
                bottom_right = corners[2].ravel()
                cv.drawFrameAxes(frame, self._camera_matrix, self._camera_distortion, rvec, tvec, 10)

                font = cv.FONT_HERSHEY_PLAIN
                font_color = (0, 0, 255)

                cv.putText(frame, f"id: {self._id_to_find} Dist: {round(pos_camera[2, 0], 2)}", top_right, font, 1.3, font_color, 2, cv.LINE_AA)
                str_position = f"x={x:.0f}  y={y:.0f}  z={z:.0f}"
                cv.putText(frame, str_position, bottom_right, font, 1.3, font_color, 2, cv.LINE_AA)
                str_attitude = f"MARKER Attitude roll={math.degrees(roll_marker):.0f} pitch={math.degrees(pitch_marker):.0f} yaw={math.degrees(yaw_marker):.0f}"
                cv.putText(frame, str_attitude, (0, 20), font, 1.0, font_color, 2, cv.LINE_AA)
                str_position = f"CAMERA Position x={pos_camera[0, 0]:.0f}  y={pos_camera[1, 0]:.0f}  z={pos_camera[2, 0]:.0f}"
                cv.putText(frame, str_position, (0, 40), font, 1, font_color, 2, cv.LINE_AA)

            found = True
            target = [x, y, z, yaw]
            markers = [roll_marker, pitch_marker, yaw_marker]

        else:
            found = False
            target = None
            markers = None

        return (found, target, markers, frame)


    def track_live(self, verbose=False, show_video=False):
        marker_found = False
        x = y = z = 0

        while True:
            found, target, marker, frame = self.track_once(draw_on_frame=show_video)

            if found:
                # logging
                if verbose:
                    print(f"Marker detected:")
                    print(f"\tX={target[0]:.1f}, \
                            Y={target[1]:.1f}, \
                            Z={target[2]:.1f}, \
                            FPS={self._fps_detect:.0f}, \
                            Flag: self.shared_array[-1]")

            else:
                if verbose:
                    print(f"Nothing detected - fps = {self._fps_read:.0f} Flag = self.shared_array[-1]")

            if show_video:
                cv.imshow('frame', frame)
                key = cv.waitKey(10) & 0xFF

                if key == ord('q'):
                    time.sleep(2)
                    break
            time.sleep(0.05)
                    
        # releasing video source
        self._video.release()
        # closing `OpenCV` windows
        cv.destroyAllWindows()

