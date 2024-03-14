import cv2 as cv

img = cv.imread("Pierres Mappe/Splish.jpg")

cv.imshow("Image", img)
cv.waitKey()