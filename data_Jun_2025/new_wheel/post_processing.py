import numpy as np
import pandas as pd
import cv2 as cv
import math 
from scipy.signal import savgol_filter

### --- PARAMETERS ---
folder_name = "data_Jun_2025/new_wheel/2025-09-09_18-15-20_10deg"
video_path = f"{folder_name}/camera2_video.avi"
aruco_video_path = f"{folder_name}/camera1_video.avi"
serial_excel_path = f"{folder_name}/sensor_data.csv"
camera_time_excel = f"{folder_name}/camera_data.csv"
output_excel_path = f"{folder_name}/merged_sorted_velocity.xlsx"

FPS = 30
d = 16.5  # cm (actual distance)
f = 485.5077  # focal length (pixels)
MPP = d / f   # meters per pixel

marker_length = 79  # mm
camera_matrix = np.array([[485.892400090055, 0, 314.712414862656],
                          [0, 485.122944583653, 243.626311650583],
                          [0, 0, 1]])
dist_coeffs = np.array([-0.00672360220673790, -0.0278217724946067, 0, 0, 0])
obj_points = np.array([
    [-marker_length / 2,  marker_length / 2, 0],
    [ marker_length / 2,  marker_length / 2, 0],
    [ marker_length / 2, -marker_length / 2, 0],
    [-marker_length / 2, -marker_length / 2, 0]
], dtype=np.float32)

### --- FUNCTION: Interpolation ---
def interpolate(x, x_list, y_list):
    if x <= x_list[0]: return y_list[0]
    if x >= x_list[-1]: return y_list[-1]
    for i in range(len(x_list) - 1):
        if x_list[i] <= x <= x_list[i + 1]:
            x0, x1 = x_list[i], x_list[i + 1]
            y0, y1 = y_list[i], y_list[i + 1]
            return y0 + (x - x0) * (y1 - y0) / (x1 - x0)
    return None

### --- Load serial data ---
serial_df = pd.read_csv(serial_excel_path)
serial_df = serial_df.sort_values('time')

times = serial_df['time'].tolist()
vFiltered = serial_df['vFilt'].tolist()

serial_times = []
serial_vels = []

for i in range(1, len(times)):
    lin_vel = vFiltered[i]  # linear velocity in cm/s

    # Save using PC time (system_time)
    system_time = serial_df.loc[i, 'system_time']
    serial_times.append(system_time)
    serial_vels.append(lin_vel)

### --- Load actual camera frame timestamps ---
cam_time_df = pd.read_csv(camera_time_excel)  # frame_number, system_time
cam_time_map = dict(zip(cam_time_df['frame_number'], cam_time_df['system_time']))

### --- Compute velocities from optical flow ---
cap = cv.VideoCapture(video_path)
ret, old_frame = cap.read()
old_gray = cv.cvtColor(old_frame, cv.COLOR_BGR2GRAY)

feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)
lk_params = dict(winSize=(15, 15), maxLevel=2,
                 criteria=(cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))

p0 = cv.goodFeaturesToTrack(old_gray, mask=None, **feature_params)
roi = (61, 55, 543, 226)  # ROI fixed

frame_number = 1
cam_times = []
cam_velocities = []

while True:
    ret, frame = cap.read()
    if not ret:
        break
    frame_number += 1
    t = cam_time_map.get(frame_number)
    if t is None:
        continue  # Skip if timestamp not available

    frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    p1, st, err = cv.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)

    if p1 is not None:
        good_new = p1[st == 1]
        good_old = p0[st == 1]

        sum_dx = 0
        sum_dy = 0
        count = 0

        for new, old in zip(good_new, good_old):
            a, b = new.ravel()
            c, d = old.ravel()
            if roi[1] < d < roi[1] + roi[3] and roi[0] < c < roi[0] + roi[2]:
                sum_dx += (a - c)
                sum_dy += (b - d)
                count += 1

        if count > 0:
            disp_x = sum_dx / count
            disp_y = sum_dy / count

            vx = disp_x * MPP * FPS  # velocity in cm/s
            vy = disp_y * MPP * FPS
        else:
            vx = 0.0
            vy = 0.0

        cam_times.append(t)
        cam_velocities.append(math.sqrt(vx*vx + vy*vy))

        old_gray = frame_gray.copy()
        existing_features = good_new.reshape(-1, 1, 2)
        if frame_number % 5 == 0:
            new_features = cv.goodFeaturesToTrack(old_gray, mask=None, **feature_params)
            if new_features is not None:
                all_features = np.vstack((existing_features, new_features)).reshape(-1, 2)
                unique_features = np.unique(all_features, axis=0)
                p0 = unique_features.reshape(-1, 1, 2)
            else:
                p0 = existing_features
        else:
            p0 = existing_features

cap.release()
cv.destroyAllWindows()

### --- 
aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_50)

parameters = cv.aruco.DetectorParameters()
parameters.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX
# parameters.cornerRefinementWinSize = 7
# parameters.cornerRefinementMinAccuracy = 0.05 
# parameters.cornerRefinementMaxIterations = 100

detector = cv.aruco.ArucoDetector(aruco_dict, parameters)

cap = cv.VideoCapture(aruco_video_path)

frame_number = 0
# prev_tvec = None
# prev_time = None

aruco_times = []
aruco_vels = []
tvecs = []

# --- Initialize previous center ---
prev_center = None
alpha = 0.15  # smoothing factor
smoothed_velocity = 0

# --- Process video ---
while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_number += 1
    t = cam_time_map.get(frame_number)
    if t is None:
        continue

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Detect ArUco markers
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        # Assume only 1 marker is present

        # Refine corner positions
        cv.cornerSubPix(gray, corners[0],
                        winSize=(7, 7), zeroZone=(-1, -1),
                        criteria=(cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_COUNT, 30, 0.01))

        corner = corners[0][0]
        center_x = np.mean(corner[:, 0])
        center_y = np.mean(corner[:, 1])
        center = (center_x, center_y)

        # Draw marker and center
        cv.aruco.drawDetectedMarkers(frame, corners, ids)
        cv.circle(frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)

        # Compute velocity if previous center exists
        if prev_center is not None:
            dx = center[0] - prev_center[0]
            dy = center[1] - prev_center[1]
            velocity = np.sqrt(dx**2 + dy**2) * 4.9185  # cm/sec

            smoothed_velocity = alpha * velocity + (1 - alpha) * smoothed_velocity

            aruco_vels.append(smoothed_velocity)
            aruco_times.append(t)

        else:
            aruco_vels.append(0.0)

        prev_center = center

aruco_vels = list(aruco_vels[1:])
aruco_times = list(aruco_times[1:])

### --- Merge and interpolate both sources ---
all_times = sorted(set(cam_times + serial_times + aruco_times))

merged_data = []
for t in all_times:
    row = {'time': t}
    # Check if direct data available
    if t in cam_times:
        row['velocity_cam'] = cam_velocities[cam_times.index(t)]
    else:
        row['velocity_cam'] = interpolate(t, cam_times, cam_velocities)

    if t in aruco_times:
        row['velocity_aruco'] = aruco_vels[aruco_times.index(t)]
    else:
        row['velocity_aruco'] = interpolate(t, aruco_times, aruco_vels)

    if t in serial_times:
        row['velocity_serial'] = serial_vels[serial_times.index(t)]
    else:
        row['velocity_serial'] = interpolate(t, serial_times, serial_vels)

    if max(row['velocity_cam'], row['velocity_serial']) != 0:
        row['slip_ratio'] = (row['velocity_serial'] - row['velocity_cam']) / max(row['velocity_cam'], row['velocity_serial'])
    else:
        row['slip_ratio'] = 0

    merged_data.append(row)

merged_df = pd.DataFrame(merged_data)
merged_df.to_excel(output_excel_path, index=False)
print("Data written to excel sheet")