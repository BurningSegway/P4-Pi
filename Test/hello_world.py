import cv2 as cv

img = cv.imread("Test/Splish.jpg")

cv.imshow("Image", img)
cv.waitKey()