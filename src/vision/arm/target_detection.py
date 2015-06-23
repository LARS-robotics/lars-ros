#!/usr/bin/python
import cv2
import numpy as np
import math
from numpy import linalg as LA
import Queue

def smaller(x,y):
	if (x[1][0] <= y[1][0] and x[1][0] <= y[1][0]):
		return True
	return False
	
def minor(x):
	return min(x[1][0],x[1][1])
	
def major(x):
	return max(x[1][0],x[1][1])

def dist(x,y):
	return math.sqrt(pow(x[0][0] - y[0][0],2) + pow(x[0][1] - y[0][1],2))

def area(x):
	return x[1][0]*x[1][1]
	
def eccentricity(x):
	a = 0.5*major(x)
	b = 0.5*minor(x)
	return math.sqrt(1-pow((b/a),2))
	
def ratioAxis(x,y):
	if minor(x) < minor(y):
		return minor(x)/minor(y)
	else:	
		return minor(y)/minor(x)
	
def target(x,y,threshDist):
	d = dist(x,y)
	#a1 = area(x)/area(y)>=0.13
	#a2 = area(x)/area(y)<=0.18
	#a3 = area(y)/area(x)>=0.13
	#a4 = area(y)/area(x)<=0.18
	e = abs(eccentricity(x) - eccentricity(y)) < 100
	r = abs(ratioAxis(x,y) - 0.3934) < 0.05
	return d, d<=threshDist  and e and r and contains(x, y)
	
def contains(x, y):
	if major(x) >= major(y):
		larger = x
		smaller = y
	else:
		larger = y
		smaller = x
	p = cv2.cv.BoxPoints(larger)
	q = cv2.cv.BoxPoints(smaller)
	V = [[p[(i+1)%4][0] - p[i][0],p[(i+1)%4][1] - p[i][1]] for i in range(4)]
	contains = True
	for v in V:
		vbar = (-v[1], v[0])
		U = [[v[0] - q[i][0], v[1] - q[i][1]] for i in range(4)]
		onOneSide = all(np.dot(vbar, u) > 0 for u in U) or all(np.dot(vbar, u) for u in U)
		contains = contains and onOneSide
		
	return contains  
		
	
def boundingRect(vertices):
	xmax = xmin = vertices[0][0]
	ymax = ymin = vertices[0][1]
	for v in vertices:
		if v[0] > xmax:
			xmax = v[0]
		if v[0] < xmin:
			xmin = v[0]
		if v[1] > ymax:
			ymax = v[1]
		if v[1] < ymin:
			ymin = v[1]
	return [(xmin, ymin), (xmin, ymax), (xmax, ymax), (xmax, ymin)]
		
	
	

capture=cv2.VideoCapture(-1)
capture.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 640)
capture.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 480)
print capture.get(cv2.cv.CV_CAP_PROP_FPS)
cv2.waitKey(200)
flag, frame = capture.read()
concentricEllipses=[]
THRESHD = 20.0
while True:
	flag, frame = capture.read()
	grayscale = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
	grayscale = cv2.blur(grayscale, (5,5))
	flag, binary = cv2.threshold(grayscale, 70, 255, cv2.THRESH_BINARY_INV)
	
	contours, hierarchy = cv2.findContours(binary.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
	hierarchy = hierarchy[0]
	
	ellipses = [cv2.fitEllipse(contour[0]) for contour in zip(contours, hierarchy) if len(contour[0]) >= 5 and cv2.contourArea(contour[0]) > 49 and contour[1][3] < 0]
	
	ellipses.sort(lambda x,y: cmp(x[0][1], y[0][1])) # sort by center.y
	
	#concentricEllipses = [(x,y) for x in ellipses for y in ellipses if target(x,y) and x!=y]
	#featureIndex = [(-1,-1),(-1,-1),(-1,-1),(-1,-1)]
#	flags = [0,0,0,0]
#	k = 0
#	for i in range(0, len(ellipses)):
#		threshD = 10.0
#		if any(n[0] == i or n[1] == i for n in featureIndex):
#			continue
#		for j in range(i+1, len(ellipses)):
#			if any(n[0] == j or n[1] == j for n in featureIndex):
#				continue
#			distance, isCentroid = target(ellipses[i],ellipses[j], threshD)
#			if isCentroid:
#				flags[k]=1
#				featureIndex[k] = (i,j)
#				threshD = distance
#		if flags[k] == 1:
#			k += 1
#		if k > 3:
#			break
			
	featureQueue = Queue.PriorityQueue()
	
	for i in range(0, len(ellipses)): # for each blob find the best matching pair
		threshD = THRESHD
		for j in range(i+1, len(ellipses)):
			distance, isCentroid = target(ellipses[i],ellipses[j], threshD)
			if isCentroid:
				featureQueue.put((distance, (i,j)))
				threshD = distance
	
	#if featureQueue.qsize() > 4:
	#	THRESHD = (THRESHD - 0.25) if THRESHD > 5.0 else 5.0
	#elif featureQueue.qsize() < 4:
	#	THRESHD = (THRESHD + 0.25) if THRESHD < 25.0 else 25.0
	
	featureIndex = []
	for i in range(0,4):
		if(featureQueue.qsize() > 0):
			x = featureQueue.get(False)[1]
			if not any(x[0]==n[0] or x[0]==n[1] or x[1]==n[0] or x[1]==n[1] for n in featureIndex):
				featureIndex.append(x)
			else:
				while True:
					alreadyUsedEllipse = any(x[0]==n[0] or x[0]==n[1] or x[1]==n[0] or x[1]==n[1] for n in featureIndex)
					if alreadyUsedEllipse and featureQueue.qsize() > 0:
						x = featureQueue.get()[1]
					elif alreadyUsedEllipse and featureQueue.qsize() == 0:
						break
					else:
						featureIndex.append(x)
			
	if (len(featureIndex) == 4):
		concentricEllipses = []
		for n in featureIndex:
			concentricEllipses.append((ellipses[n[0]],ellipses[n[1]]))

				
	#print cocentricEllipses

	img = frame.copy()
	
	#for ellipse in ellipses:
	#	cv2.ellipse(img, ellipse, (255,128,255))
	
	#for ellipsePair in cocentricEllipses:
	#	for ellipse in ellipsePair:
			
	
	#for ellipsePair in cocentricEllipses:
	#	for ellipse in ellipsePair: 
	#		vertices = np.int0(cv2.cv.BoxPoints(ellipse))
	#		vertices = [ tuple(v) for v in vertices]
	#		#cv2.boundingRect(cv2.ellipse2Poly(ellipse[0], ellipse[1], ellipse[2], 0, 360, 5))
	#		print vertices
	#		vertices = boundingRect(vertices)
	#		for i in range(0, 4):
	#			cv2.line(img, vertices[i], vertices[(i+1)%4], (0, 255, 0), 2)
		
	for ellipsePair in concentricEllipses:
		cv2.ellipse(img, ellipsePair[0], (0,255,0), 2)
		#print cv2.cv.BoxPoints(ellipsePair[0])
		cv2.ellipse(img, ellipsePair[1], (0,255,0), 2)
	
	points = [((ellipsePair[0][0][0]+ellipsePair[1][0][0])/2,(ellipsePair[0][0][1]+ellipsePair[1][0][1])/2) for ellipsePair in concentricEllipses]
	
	points.sort(lambda x,y: cmp(x[0], y[0]))
	
	if len(points)==4 and points[0][1] > points[1][1]:
		temp = points[0]
		points[0] = points[1]
		points[1] = temp
	
	if len(points)==4 and points[2][1] < points[3][1]:
		temp = points[2]
		points[2] = points[3]
		points[3] = temp
	#print points
	if len(points)==4:
		ic = np.matrix([[p[0] for p in points], [p[1] for p in points], [1.0 for x in range(0,4)]])
		cal = np.matrix([[ 518.81428608, 0, 298.18411412],[0,517.07586175, 244.68941109],[0,0,1]])
		
		coordinatesSI = LA.inv(cal)*ic

		
		p = ((coordinatesSI[0,0] + coordinatesSI[0,3])/2, (coordinatesSI[1,0] + coordinatesSI[1,3])/2)
		q = ((coordinatesSI[0,1] + coordinatesSI[0,2])/2, (coordinatesSI[1,1] + coordinatesSI[1,2])/2)
		
		f = 1.0
		dv = 109.5375
		
		psi = math.atan(p[0]/f)

		z = f*dv/(q[1] - p[1])
		rho = z/math.cos(psi)
		
		zA = f*dv/(coordinatesSI[1,1] - coordinatesSI[1,0])
		zD = f*dv/(coordinatesSI[1,2] - coordinatesSI[1,3])
		#print coordinatesSI
		Abar = np.zeros(3)
		Bbar = np.zeros(3)
		Cbar = np.zeros(3)
		Dbar = np.zeros(3)
		AD = np.zeros(3)
		AB = np.zeros(3)
		DC = np.zeros(3)
		BC = np.zeros(3)
		#print zA, zD
		for i in range(0,2):
			Abar[i] = zA*coordinatesSI[i,0]/f
			Bbar[i] = zA*coordinatesSI[i,1]/f
			Dbar[i] = zD*coordinatesSI[i,3]/f
			Cbar[i] = zD*coordinatesSI[i,2]/f
			AD[i] = Dbar[i] - Abar[i]
			AB[i] = Bbar[i] - Abar[i]
			DC[i] = Cbar[i] - Dbar[i]
			BC[i] = Cbar[i] - Bbar[i]
		#print AB 
		Abar[2] = zA/f
		Bbar[2] = zA/f
		Dbar[2] = zD/f
		Cbar[2] = zD/f	
		
		AD[2] = Dbar[2] - Abar[2]
		AB[2] = Bbar[2] - Abar[2]
		DC[2] = Cbar[2] - Dbar[2]
		BC[2] = Cbar[2] - Bbar[2]
		
		nAD = LA.norm(AD)
		nAB = LA.norm(AB)
		nDC = LA.norm(DC)
		nBC = LA.norm(BC)
		
		for i in range(0,2):
			AD[i] /= nAD
			AB[i] /= nAB
			DC[i] /= nDC
			BC[i] /= nBC
		#print Abar, Bbar, AB
		n = ((np.cross(AD, DC) + np.cross(BC,AB))/2)
		gamma = 0
		if AD[2] < 0.0:
			gamma = math.atan2(math.sqrt(1-pow(n[2],2)), n[2])
		else :
			gamma = math.atan2(-math.sqrt(1-pow(n[2],2)), n[2])
		
		#print rho, psi*180/math.pi, gamma
	
	#print "Exposure: ", capture.get(cv2.cv.CV_CAP_PROP_EXPOSURE), "Gain: ",capture.get(cv2.cv.CV_CAP_PROP_GAIN),"Contrast: ",capture.get(cv2.cv.CV_CAP_PROP_CONTRAST),"Brightness: ",capture.get(cv2.cv.CV_CAP_PROP_BRIGHTNESS)
		
	cv2.imshow("bin", binary)
	cv2.imshow("ellipse", img)
	c=cv2.waitKey(2)
	if c==27: #Break if user enters 'Esc'.
		break 
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
