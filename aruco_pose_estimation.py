import numpy as np
import cv2
import time
import math

def get_euler_angles_from_rvec(rvec):
    # Convert rotation vector to rotation matrix
    R, _ = cv2.Rodrigues(rvec)
    
    roll  = math.atan2(R[1, 2], R[2, 2])
    pitch = -math.atan2(R[0, 2], 1)
    yaw   = math.atan2(R[0, 1], R[0, 0])

    # Convert to degrees
    return np.degrees([roll, pitch, yaw])


folder_name = "D:/Single Wheel Testbed/data_Jun_2025/Acq_IEEE_SPACE/2025-07-03_18-21-06_0deg"
aruco_video_path = f"{folder_name}/camera2_video.avi"
video = cv2.VideoCapture(0)
# video = cv2.VideoCapture(aruco_video_path)
time.sleep(2.0)

marker_length = 79 # mm

camera_matrix = np.matrix([[485.892400090055, 0, 314.712414862656],
                [0, 485.122944583653, 243.626311650583],
                [0, 0, 1]])

dist_coeffs = np.matrix([-0.00672360220673790, -0.0278217724946067, 0, 0, 0])

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# Define 3D coordinates of the marker's corners in marker frame (clockwise from top-left)
obj_points = np.array([
    [-marker_length / 2,  marker_length / 2, 0],
    [ marker_length / 2,  marker_length / 2, 0],
    [ marker_length / 2, -marker_length / 2, 0],
    [-marker_length / 2, -marker_length / 2, 0]
], dtype=np.float32)

# frame_zero = cv2.imread('orient_0.png')
# gray = cv2.cvtColor(frame_zero, cv2.COLOR_BGR2GRAY)

# corners, ids, _ = detector.detectMarkers(gray)

# if len(corners) > 0:
#     for corner, marker_id in zip(corners, ids.flatten()):
#         image_points = corner[0].astype(np.float32)

#         # Pose estimation using solvePnP
#         success, rvec, tvec = cv2.solvePnP(obj_points, image_points, camera_matrix, dist_coeffs)

#         if success:
#             euler_angles = get_euler_angles_from_rvec(rvec)
#             yaw_0 = euler_angles[2]
while True:
    ret, frame = video.read()

    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = detector.detectMarkers(gray)

    # If markers are detected
    if len(corners) > 0:
        # Draw a square around the markers
        frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        
        for corner, marker_id in zip(corners, ids.flatten()):
            image_points = corner[0].astype(np.float32)

            # Pose estimation using solvePnP
            success, rvec, tvec = cv2.solvePnP(obj_points, image_points, camera_matrix, dist_coeffs)

            if success:
                # Draw axis on the marker
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 30)

                euler_angles = get_euler_angles_from_rvec(rvec)

                yaw = euler_angles[2]
                # net_yaw = yaw - yaw_0
                net_yaw = yaw

                # Print translation and rotation vectors
                print(f"Marker ID: {marker_id}")
                print(f"Net Yaw :{net_yaw}")

    cv2.imshow('Estimated Pose', frame)

    key = cv2.waitKey(1)
    if key == 27:
        break

video.release()
cv2.destroyAllWindows()