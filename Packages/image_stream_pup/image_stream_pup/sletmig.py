import cv2 as cv
import time

def testDevice(source):
    cap = cv.VideoCapture(source)
    if cap is None or not cap.isOpened():
        print("Kan ikke åbne: ", source)
    _, image = cap.read()

    cv.imshow("Image", image)
    cv.waitKey()

testDevice(0)
    