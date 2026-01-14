import serial
import time
import csv
import os
import cv2
import datetime
import threading
import numpy as np
from math import sqrt, pi

# Create a unique folder inside the 'data' folder using current date and time
run_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
folder_name = f"data_Jun_2025/Acq_IEEE_SPACE/{run_time}"

# Create the directory if it doesn't exist
os.makedirs(folder_name, exist_ok=True)

running = True

event_cam_open = threading.Event()
event_serial_open = threading.Event()

def cam_read():
    global running, folder_name, event_cam_open

    video1_output_filename = f"{folder_name}/camera1_video.avi"

    # Define video capture for two cameras
    cap1 = cv2.VideoCapture(0)  # Camera 1

    frame_number = 0

    FPS = 30
    d = 16.5  # cm
    f = 485.5077    # pixels
    MPP = d / f
    roi = (39, 55, 568, 280)

    # params for ShiTomasi corner detection
    feature_params = dict( maxCorners = 100,
                        qualityLevel = 0.3,
                        minDistance = 7,
                        blockSize = 7 )

    # Parameters for lucas kanade optical flow
    lk_params = dict( winSize  = (15, 15),
                    maxLevel = 2,
                    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    # Create some random colors
    # color = np.random.randint(0, 255, (800, 3))

    # Check if the cameras opened successfully
    if not cap1.isOpened():
        print("Error: Couldn't open one of the cameras.")
        exit()

    # Get the frame width and height (set the same for both cameras)
    frame_width = int(cap1.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap1.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Set up video writers for both cameras
    output_video1 = cv2.VideoWriter(video1_output_filename, cv2.VideoWriter_fourcc(*'XVID'), 30, (frame_width, frame_height))

    event_cam_open.set()
    event_serial_open.wait()

    print("Camera initialized")
    csv_output_filename = f"{folder_name}/camera_data.csv"

    with open(csv_output_filename, mode='a', newline='') as csvfile:
        fieldnames = ['system_time', 'frame_number', 'velocity_x', 'velocity_y', 'absolute_velocity']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        while running:
            system_time_cam = time.time()

            # Capture frames from both cameras
            ret1, frame1 = cap1.read()
            
            if ret1:
                frame_number += 1
                if frame_number == 1:
                    old_gray = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
                    p0 = cv2.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
                    mask = np.zeros_like(frame1)
                    u = 0
                    v = 0
                    print("frame:" + str(frame_number))
                else:
                    print("frame:" + str(frame_number))
                    frame_gray = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)

                    # calculate optical flow
                    p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)

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
                        
                            # mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), color[i].tolist(), 2)
                            # frame = cv2.circle(frame1, (int(a), int(b)), 5, color[i].tolist(), -1)

                    displacement_x = sum_pos_x / count
                    displacement_y = sum_pos_y / count

                    u = displacement_x * MPP * FPS
                    v = displacement_y * MPP * FPS

                    # img = cv2.add(frame, mask)
                    # cv2.imshow('frame', img)
                    # k = cv2.waitKey(30) & 0xff
                    # if k == 27:
                    #     break

                    # Now update the previous frame and previous points
                    old_gray = frame_gray.copy()
                    existing_features = good_new.reshape(-1, 1, 2)
                    # p0 = existing_features
                    if frame_number % 5 == 0:
                        new_features = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)

                        if new_features is not None:
                            all_features = np.vstack((existing_features, new_features)).reshape(-1, 2)
                            unique_features = np.unique(all_features, axis=0)
                            p0 = unique_features.reshape(-1, 1, 2)
                        else:
                            p0 = existing_features
                    else:
                        p0 = existing_features


                output_video1.write(frame1)  # Save video frame from camera 1
                writer.writerow({'system_time': system_time_cam, 'frame_number': frame_number, 'velocity_x': u, 'velocity_y': v, 'absolute_velocity': sqrt(u*u + v*v)})

    # Release the video capture and writer objects
    cap1.release()
    output_video1.release()
    cv2.destroyAllWindows()


def serial_read():
    global running, folder_name, event_serial_open

    # Create a unique filename using the same timestamp
    csv_output_filename = f"{folder_name}/sensor_data.csv"

    # Set up serial connection
    serial_port = 'COM3'  # Change this to the appropriate port if needed
    baud_rate = 115200
    ser = serial.Serial(serial_port, baud_rate)

    prev_tt = 0
    prev_ang_pos = 0

    print("Serial initialized")

    event_serial_open.set()
    event_cam_open.wait()

    # Create a CSV file to save the data
    with open(csv_output_filename, mode='w', newline='') as csvfile:
        fieldnames = ['system_time', 'time', 'vt', 'vFilt', 'pos', 'curr']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        ser.reset_output_buffer()
        ser.reset_input_buffer()
        # ser.write("START".encode())

        while running:
            try:
                system_time = time.time()

                # Read a line from the serial port
                line = ser.readline().decode('utf-8').strip()
                if line.startswith('<<'):
                    line = line[2:].strip()  # Remove '<<' and any leading/trailing spaces
                # print(line)

                # Try to convert the line into 3 floats
                values = line.split()
                if len(values) == 5:
                    try:
                        tt, vt, vFilt, pos, curr = map(float, values)

                        # Write the data to the CSV file
                        writer.writerow({'system_time': system_time, 'time': tt, 'vt': vt, 'vFilt': vFilt, 'pos': pos, 'curr': curr})
                        
                    except ValueError:
                        # If the conversion fails, skip the line
                        continue
            except Exception as e:
                print(f"Error reading data: {e}")

        print(f"Data collection completed. Data saved to {csv_output_filename}")
    
    # Close the serial port
    ser.close()

cam_thread = threading.Thread(target=cam_read)
serial_thread = threading.Thread(target=serial_read)

# start threads
cam_thread.start()
serial_thread.start()
running = True

event_cam_open.wait()
event_serial_open.wait()

start_time = time.time()

while time.time() - start_time < 8:
    time.sleep(0.1)

running = False

cam_thread.join()
serial_thread.join()

print("Program terminated")