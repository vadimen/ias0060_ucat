import cv2
import numpy as np

# Load the video
cap = cv2.VideoCapture('test2.mp4')

# Parameters for ShiTomasi corner detection
feature_params = dict(maxCorners=100, qualityLevel=0.01, minDistance=10, blockSize=7)

# Parameters for Lucas-Kanade optical flow
lk_params = dict(winSize=(31, 31), maxLevel=3, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

# Take first frame and find corners in it
ret, old_frame = cap.read()
old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)

# Let user select ROI
roi = cv2.selectROI(old_frame)
x, y, w, h = roi

# Use ROI to initialize the tracker
mask = np.zeros_like(old_gray)
mask[y:y+h, x:x+w] = 255
p0 = cv2.goodFeaturesToTrack(old_gray, mask=mask, **feature_params)


def calc_flow(old_gray, frame_gray, p0, lk_params):
    # Calculate optical flow
    p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
    # Select good points
    good_new = p1[st == 1]
    good_old = p0[st == 1]
    # Compute the bounding box coordinates
    x_min = np.min(good_new[:, 0])
    y_min = np.min(good_new[:, 1])
    x_max = np.max(good_new[:, 0])
    y_max = np.max(good_new[:, 1])
    x_min = int(x_min)
    y_min = int(y_min)
    x_max = int(x_max)
    y_max = int(y_max)

    return p1, st, err, good_new, good_old, x_min, y_min, x_max, y_max


while 1:
    ret, frame = cap.read()
    if not ret:
        break

    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    p1, st, err, good_new, good_old, x_min, y_min, x_max, y_max = calc_flow(old_gray, frame_gray, p0, lk_params)

    new_w = x_max - x_min
    new_h = y_max - y_min

    if len(good_new) < len(p0) * 0.9 or 1.1*w*h < new_h*new_w < 0.9*w*h:
        if 1.1*w*h < new_h*new_w:
            ofs = -20
        else:
            ofs = 20

        mask = np.zeros_like(frame_gray)
        #mask[y:y+h + ofs, x:x+w+ofs] = 255
        mask[y_min:y_max + ofs, x_min:x_max+ofs] = 255
        p0 = cv2.goodFeaturesToTrack(frame_gray, mask=mask, **feature_params)

        p1, st, err, good_new, good_old, x_min, y_min, x_max, y_max = calc_flow(old_gray, frame_gray, p0, lk_params)

    # Draw the bounding box
    frame = cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

    # Show the image
    cv2.imshow('Frame', frame)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Update the previous frame and previous points
    old_gray = frame_gray.copy()
    p0 = good_new.reshape(-1, 1, 2)

    x = x_min
    y = y_min
    w = new_w
    h = new_h

cv2.destroyAllWindows()
cap.release()