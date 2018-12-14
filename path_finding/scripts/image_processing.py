import cv2 as cv
import numpy as np

def crop_image(img,tol=1):
    # img is image data
    # tol  is tolerance
    mask = img<tol
    return img[np.ix_(mask.any(1),mask.any(0))]

size_robot=10

#Open the map
img = cv.imread('../polytech.pgm',0)

print(img)


#Crop the image to remove the empty borders
cropped=crop_image(img)

#Thresholding
ret,thresh1 = cv.threshold(cropped,250,255,cv.THRESH_BINARY)

#Removing noise
kernel = np.ones((5,5),np.uint8)
opening = cv.morphologyEx(thresh1, cv.MORPH_OPEN, kernel)
#dilate = cv.dilate(opening,np.ones((2,2),np.uint8),iterations = 1)

#To take the size of the robot in consideration
kernel_r = np.ones((size_robot,size_robot),np.uint8)
erosion = cv.erode(opening,kernel_r,iterations = 1)

#Save
cv.imwrite('test_map.pgm',erosion)

#Show
cv.namedWindow('image',cv.WINDOW_NORMAL)
cv.imshow('image', erosion)
if cv.waitKey(): cv.destroyAllWindows()
