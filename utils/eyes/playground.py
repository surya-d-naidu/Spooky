import cv2

video = cv2.VideoCapture(0)
while True:
    __, frame = video.read()
    cv2.imshow("Stream", frame)
    cv2.waitKey(1)
