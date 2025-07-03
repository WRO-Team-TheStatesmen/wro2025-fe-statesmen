import cv2
import numpy as np

def nothing(x):
    pass

cv2.namedWindow('Trackbars')

cv2.createTrackbar('Red Lower','Trackbars',0,255,nothing)
cv2.createTrackbar('Green Lower','Trackbars',0,255,nothing)
cv2.createTrackbar('Blue Lower','Trackbars',0,255,nothing)
cv2.createTrackbar('Red Upper','Trackbars',0,255,nothing)
cv2.createTrackbar('Green Upper','Trackbars',0,255,nothing)
cv2.createTrackbar('Blue Upper','Trackbars',0,255,nothing)

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    red_low = cv2.getTrackbarPos('Red Lower','Trackbars')
    green_low = cv2.getTrackbarPos('Green Lower','Trackbars')
    blue_low = cv2.getTrackbarPos('Blue Lower','Trackbars')
    red_up = cv2.getTrackbarPos('Red Upper','Trackbars')
    green_up = cv2.getTrackbarPos('Green Upper','Trackbars')
    blue_up = cv2.getTrackbarPos('Blue Upper','Trackbars')

    lower = np.array([red_low, green_low, blue_low])
    upper = np.array([red_up, green_up, blue_up])

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mask = cv2.inRange(rgb, lower, upper)
    result = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow('Original', frame)
    cv2.imshow('Mask', mask)
    cv2.imshow('Result', result)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()