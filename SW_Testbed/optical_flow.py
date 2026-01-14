import numpy as np
import cv2 as cv
import argparse
import csv
import os

filename = "data_Jun_2025/new_wheel/2025-09-09_17-10-17_0deg/camera2_video.avi"

# capture the video
cap = cv.VideoCapture(filename)

FPS = 30
d = 16.5  # cm
f = 485.5077    # pixels
MPP = d / f

# params for ShiTomasi corner detection
feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )

# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15, 15),
                  maxLevel = 2,
                  criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))

# Create some random colors
color = np.random.randint(0, 255, (800, 3))

# Take first frame and find corners in it
ret, old_frame = cap.read()
old_gray = cv.cvtColor(old_frame, cv.COLOR_BGR2GRAY)
p0 = cv.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
current_frame = 0

roi = cv.selectROI("Select ROI", old_frame, False)
print(roi)
# roi = (30, 45, 554, 255)

# Create a mask image for drawing purposes
mask = np.zeros_like(old_frame)

csv_output_filename = f"optical_flow.csv"
fieldnames = ['frame_number', 'velocity_x', 'velocity_y']

with open(csv_output_filename, mode='w', newline='') as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()

    while(1):
        ret, frame = cap.read()
        if not ret:
            print('No frames grabbed!')
            break
        current_frame += 1

        frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # calculate optical flow
        p1, st, err = cv.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)

        # Select good points
        if p1 is not None:
            good_new = p1[st==1]
            good_old = p0[st==1]

        count = 0
        sum_pos_x = 0
        sum_pos_y = 0

        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()

            if  roi[1] < d < roi[1] + roi[3] and roi[0] < c < roi[0] + roi[2]:
                sum_pos_x += (a - c)
                sum_pos_y += (b - d)
                count += 1
            
                mask = cv.line(mask, (int(a), int(b)), (int(c), int(d)), color[i].tolist(), 2)
                frame = cv.circle(frame, (int(a), int(b)), 5, color[i].tolist(), -1)

        displacement_x = sum_pos_x / count
        displacement_y = sum_pos_y / count

        u = displacement_x * MPP * FPS
        v = displacement_y * MPP * FPS
        
        writer.writerow({'frame_number': current_frame, 'velocity_x': u, 'velocity_y': v})
        
        img = cv.add(frame, mask)

        cv.imshow('frame', img)
        k = cv.waitKey(30) & 0xff
        if k == 27:
            break

        # Now update the previous frame and previous points
        old_gray = frame_gray.copy()
        existing_features = good_new.reshape(-1, 1, 2)
        # p0 = existing_features
        if current_frame % 5 == 0:
            new_features = cv.goodFeaturesToTrack(old_gray, mask=None, **feature_params)

            if new_features is not None:
                all_features = np.vstack((existing_features, new_features)).reshape(-1, 2)
                unique_features = np.unique(all_features, axis=0)
                p0 = unique_features.reshape(-1, 1, 2)
            else:
                p0 = existing_features
        else:
            p0 = existing_features

    cv.destroyAllWindows()


