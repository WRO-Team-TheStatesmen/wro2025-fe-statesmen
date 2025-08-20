import cv2
import numpy as np

def nothing(x):
    pass

cv2.namedWindow('Trackbars')

cv2.createTrackbar('Hue Lower','Trackbars',0,179,nothing)
cv2.createTrackbar('Saturation Lower','Trackbars',0,255,nothing)
cv2.createTrackbar('Value Lower','Trackbars',0,255,nothing)
cv2.createTrackbar('Hue Upper','Trackbars',0,179,nothing)
cv2.createTrackbar('Saturation Upper','Trackbars',0,255,nothing)
cv2.createTrackbar('Value Upper','Trackbars',0,255,nothing)

cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hue_low = cv2.getTrackbarPos('Hue Lower','Trackbars')
    sat_low = cv2.getTrackbarPos('Saturation Lower','Trackbars')
    val_low = cv2.getTrackbarPos('Value Lower','Trackbars')
    hue_up = cv2.getTrackbarPos('Hue Upper','Trackbars')
    sat_up = cv2.getTrackbarPos('Saturation Upper','Trackbars')
    val_up = cv2.getTrackbarPos('Value Upper','Trackbars')

    lower = np.array([hue_low, sat_low, val_low])
    upper = np.array([hue_up, sat_up, val_up])

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow('Original', frame)
    cv2.imshow('Mask', mask)
    cv2.imshow('Result', result)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print(lower)
print(upper)