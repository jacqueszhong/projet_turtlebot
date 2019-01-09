import cv2 as cv
import numpy as np

GUI = 1

def crop_image(img,tol=1):
    # img is image data
    # tol  is tolerance
    mask = img<tol
    print(img)
    return img[np.ix_(mask.any(1),mask.any(0))]

size_robot=10


def image_process(img=None): #Open the map
	if GUI:
		img = cv.imread('../mymap.pgm',0)
		cv.namedWindow('image',cv.WINDOW_NORMAL)
		cv.imshow('image', img)
		if cv.waitKey(): cv.destroyAllWindows()

	#Crop the image to remove the empty borders

	# Mask of non-black pixels (assuming image has a single channel).
	mask = img < 1

	# Coordinates of non-black pixels.
	coords = np.argwhere(mask)

	# Bounding box of non-black pixels.
	y0, x0 = coords.min(axis=0)
	y1, x1 = coords.max(axis=0)
	print ("x0: "+str(x0)+" y0: "+str(y0))
	print ("x1: "+str(x1)+" y1: "+str(y1))

	# Get the contents of the bounding box.
	cropped = img[y0:y1, x0:x1]
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
	if GUI:
		cv.namedWindow('image',cv.WINDOW_NORMAL)
		cv.imshow('image', erosion)
		if cv.waitKey(): cv.destroyAllWindows()

	return [(x0,y0),erosion]


if __name__ == '__main__':
	image_process()