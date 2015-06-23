import cv
for x in range(-1,10):
	capture = cv.CaptureFromCAM(x)
	if capture:
		print capture
		frame = cv.QueryFrame(capture)
		print x, frame
	
