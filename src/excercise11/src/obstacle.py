#!/usr/bin/env python2

import roslib
import rospy
import time
import datetime
import math
import numpy as np
from std_msgs.msg import Int16
from PIL import Image
from PIL import ImageDraw
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from setup_values import Setup
from model_track import Track

setup = Setup()
logging = setup.logging
carID = setup.carID  # 5
laneID = setup.laneID  # 0
model = Track(laneID, logging)

# fuer 'realtime' data
global_curPos = (0,0)
global_orientation = 0.0
scanner_list = []

# fuer den plotter
obstacle_list = []
position_list = []
draw_timestamp = time.time()

odom_timestamp = time.time()

pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=100, latch=True)

def odom_callback(data):

	global odom_timestamp
	global position_list

	global global_curPos
	global global_orientation

	cur_x = int(round(data.pose.pose.position.x * 100))
	cur_y = int(round(data.pose.pose.position.y * 100))

	global_curPos = (cur_x, cur_y)

	cur_z = data.pose.pose.orientation.z
	cur_w = data.pose.pose.orientation.w

	global_orientation = np.arccos(cur_w)*2 * (np.sign(cur_z) * (-1))

	#fuer den plotter
	if time.time() - odom_timestamp >= 1:
		#die PIL liest es anders as die Welt
		position_list.append([cur_y, cur_x])
		odom_timestamp= time.time()
		#return orientation of car in rad where 0,0 of the map is the Ursprung


def scan_callback(data):
	# fuer den plotter
	global global_curPos
	global global_orientation
	plotter_list = []

	distances = data.ranges
	global scanner_list
	global obstacle_list
	scanner_list = []
	# soll so haesslich bleiben
	# inf -> 0
	# alles ueber 1,5m -> 0
	relev_d = distances[0:45] + distances[314:359]
	for x in relev_d:
		if x >=1.3:
			scanner_list.append(0)
		else:
			scanner_list.append(x)
			# fuer den plotter
			if x <= 0.4:
				plotter_list.append(x)
	transLidarInf(plotter_list, global_curPos, global_orientation)

def plotter():
	print("PLOTTER PLOTTET")
	global position_list
	global obstacle_list
	time.sleep(70)
	r=3
	image_in = "map.bmp"
	img = Image.open(image_in).convert('RGB')
	draw = ImageDraw.Draw(img)
	for pos in position_list:
		draw.ellipse((pos[0] - r, pos[1] - r, pos[0] +r, pos[1] +r), fill=(255,0,0))
	for obs in obstacle_list:
		draw.ellipse((pos[0] - r, pos[1] - r, pos[0] +r, pos[1] +r), fill=(0,255,0))
	image_out = "/home/plot.png"
	img.save(image_out)

def findpoint(point,laneID): #Finde den nahsten Punkt auf einer bestimmen Spur

	x= point[0]
	y= point[1]
	# Distanz vom inneren Oval festlegen
	lanedist = 0
	if setup.laneID == 0:
		lanedist = 15
	else:
		lanedist = 45

	ret = (0,0)

	if (x,y) == (195,215): #Spezialfall 1: Mitte oberer Kreis
		ret = (75-dist,215)
	elif (x,y) == (405,215): #Spezialfall 2: Mitte unterer Kreis
		ret = (525+dist,215)
	elif x <= 194: # Teil 1 (siehe Aufgabe 2, Aufteilung in Gebiete)
		if y < 215:
			ret = (195-round(math.sqrt(((120+lanedist)**2)-((((120+lanedist)**2)*(y-215)**2)/((x-195)**2+(y-215)**2)))),215-round(math.sqrt(((((120+lanedist)**2)*(y-215)**2)/((x-195)**2+(y-215)**2))))) #komplizierte, per Hand herbeigefuehrte Formel...
		else:
			ret = (195-round(math.sqrt(((120+lanedist)**2)-((((120+lanedist)**2)*(y-215)**2)/((x-195)**2+(y-215)**2)))),215+round(math.sqrt(((((120+lanedist)**2)*(y-215)**2)/((x-195)**2+(y-215)**2)))))#komplizierte, per Hand herbeigefuehrte Formel...
	elif 195 <= x <= 405 and y <= 215: # Teil 2
		ret = (x,95-lanedist)
	elif 195 <= x <= 405 and 216 <= y: # Teil 3
		ret = (x,335+lanedist)
	elif 406 <= x <= 600: # Teil 4
		if y < 215:
			ret = (405+round(math.sqrt(((120+lanedist)**2)-((((120+lanedist)**2)*(y-215)**2)/((x-405)**2+(y-215)**2)))),215-round(math.sqrt(((((120+lanedist)**2)*(y-215)**2)/((x-405)**2+(y-215)**2))))) #komplizierte, per Hand herbeigefuehrte Formel...

		else:
			ret = (405+round(math.sqrt(((120+lanedist)**2)-((((120+lanedist)**2)*(y-215)**2)/((x-405)**2+(y-215)**2)))),215+round(math.sqrt(((((120+lanedist)**2)*(y-215)**2)/((x-405)**2+(y-215)**2))))) #komplizierte, per Hand herbeigefuehrte Formel...

	ret = (int(ret[0]),int(ret[1]))

	return ret


def transLidarInf(A, currPos, orient):  # Lidar-Information in Koordinaten uebersetzen

# In Abhaengigkeit von getLidarInf
# A enthaelt die Distanz von Objekten, vom Lidar gemessen
	incr = math.pi * 2 / 360
	x = currPos[0]
	y = currPos[1]
	C = [] #Enthaelt nach dem for-loop die Richtung in RAD aller Lidar-Informationen, passend zu A
	for i in range(0, len(A)):
		if i < len(A) / 2:
			C.append(orient + i * incr)
		else:
			C.append(orient + -(len(A) - i) * incr)
		if C[i] > math.pi:
			C[i] = -2 * math.pi + C[i]
	D = []
	for i in range(0, len(C)):
		#x/y-Offset kann durch den Winkel Beta (C[i]) und die Länge der Hypotenuse (A[i]) berechnet werden
		xadd = round(A[i] * 100 * math.cos(C[i]), 2)
		if abs(C[i]) >= math.pi / 2:
			xadd = -xadd
		yadd = round(math.sqrt((A[i] * 100) ** 2 - xadd ** 2), 2)
		if A[i] == 0: # Nullen in A bedeuten, dass kein Objekt gefunden wurde, werden also aussortiert
			continue
		if 0 <= C[i] < math.pi / 2:#Je nachdem, in welche Richtung der Winkel zeigh, muss der Offset addiert oder subtrahiert werden
			D.append((x + xadd, y - yadd))
		elif 0 <= C[i]:
			D.append((x - xadd, y - yadd))
		elif math.pi / -2 < C[i] < 0:
			D.append((x + xadd, y + yadd))
		else:
			D.append((x - xadd, y + yadd))
	return (D)


def findRelevantObstacles(A,laneID): #Pruefen, ob es auf einer Bahn Hindernisse gibt
	ret = []
	for i in A:
		ref = findpoint(i,laneID)#Finde den Referenzpunkt auf der Bahn
		dist = math.sqrt((ref[0]-i[0])**2 + (ref[1]-i[1])**2) #Berechne die Distanz zwischen dem Punkt und dem Referenz-Punkt
		if dist <=15:#Falls der Punkt maximal 15 Zentimeter vom Zentrum der Bahn entfernt ist, ist das Obstacle relevant
			ret.append(ref)
	return (ret)

def closestObstacle(currPos,laneID,A): #Das nahste Hinderniss auf einer Bahn finden
	mindist = 500 #Große Zahl, die nicht vom Lidar geliefert werden kann (wird vorher rausgefiltert)
	for i in A:
		dist = findDistance(currPos,i,laneID,0)#Berechne die Distanz vom Auto zu den relevanten Obstacles
		if dist < mindist:
			mindist = dist#Speichere die minimale Distanz
	return (mindist)

def findDistance(point1,point2,laneID,distance):#Distanz zwischen 2 Punkten auf der
	#Bahn finden, wobei point1 vor point2 liegen muss.
	x1=point1[0]
	y1=point1[1]
	x2=point2[0]
	y2=point2[1]

	if laneID == 1: #Radius der Kreise, basieren auf der Bahn festlegen
		rad = 135
	else:
		rad = 165

	totUmf = rad*math.pi #Umfang eines Halbkreises
	
	if y1 <= 215: #Den Winkel von Punkt 1 auf dem Halbkreis berechnen (nutzlos, falls Punkt 1 auf keinem Halbkreis liegt.)
		beta = 90+math.degrees(math.asin((215-y1)/rad))
	else:
		beta = 90-math.degrees(math.asin((y1-215)/rad))

	if y2 <= 215: #Den Winkel von Punkt 2 auf dem Halbkreis berechnen (nutzlos, falls Punkt 2 auf keinem Halbkreis liegt.)
		alpha = 90+math.degrees(math.asin((215-y2)/rad))
	else:
		alpha = 90-math.degrees(math.asin((y2-215)/rad))

	if x1 <= 194: #oberer Halbkreis
		umf = totUmf-(beta/180)*totUmf #Restumfang von Punkt 1 bis zum Ende des oberen Halbkreises
		if x2 > 195: #Falls sich der 2.Punkt nicht im gleichen Bereich befindet
			distance += umf
			newPoint1 = (195,215-rad)
		else:
			umf2 = totUmf-(alpha/180)*totUmf
			distance += umf-umf2
			newPoint1=point2

	elif 195 <= x1 <= 405 and y1 <= 215: #linke Gerade
	    	if 195 > x2 or x2 > 405 or y2 > 215: #Falls sich der 2.Punkt nich im gleichen Bereich befindet
			newPoint1 = (406,215-rad)
			distance += 406-x1
		else:
			newPoint1 = point2
			distance += x2-x1

	elif 195 <= x1 <= 405 and 216 <= y1: #rechte Gerade
		if 105 > x2 or x2 > 405 or y2 < 216: #Falls sich der 2.Punkt nich im gleichen Bereich befindet
			newPoint1 = (194,215+rad)
			distance += x1-194
		else:
			newPoint1 = point2
			distance += x1-x2

	elif 406 <= x1: #unterer Halbkreis
		umf = totUmf-(beta/180)*totUmf #Restumfang von Punkt 1 bis zum Ende des unteren Halbkreises
		if x2 < 406: #Falls sich der 2.Punkt nich im gleichen Bereich befindet
			distance += umf
			newPoint1 = (405,215+rad)
		else:
			umf2 = totUmf-(alpha/180)*totUmf
			distance += umf-umf2
			newPoint1=point2

	if newPoint1 != point2: #Wenn der neue berechnete Punkt nicht mit Punkt 2 übereinstimmt, wird die Funktion erneut aufgerufen,
				#mit dem neuen Punkt als Punkt 1 und der bereits "zurückgelegten" Distanz (Punkt 1 bis neuer Punkt 1)
		return (findDistance(newPoint1,point2,laneID,distance))
	else:
		return (int(distance)) #Wenn newPoint1 == Point 2, wurde die richtige Distanz gefunden

def obstacle_based_lane_switch():
	global scanner_list
	global global_curPos
	global global_orientation
	setup.laneID
	A=transLidarInf(scanner_list, global_curPos, global_orientation) #Fetched die Lidar-Informationen

	obstacles1 = closestObstacle(findPoint(global_curPos,1),1,findRelevantObstacles(A,1)) #Nahstes Obstacle auf Bahn 1
	obstacles2 = closestObstacle(findPoint(global_curPos,2),2,findRelevantObstacles(A,2)) #Nahstes Obstacle auf Bahn 2
	if (setup.laneID == 0 and obstacles1 < 50) or (setup.laneID == 1 and obstacles2 < 50):
		rospy.on_shutdown(when_shutdown)
		#pub_speed.publish(0) #Wenn das Hinderniss zu nah ist, halte an.
	elif (setup.laneID == 0 and obstacles1 < 500) and not (obstacles2 < 500):
		setup.laneID = 1 #Wenn auf Bahn 1 gefahren wird, und sich ein dort ein Hinderniss befindet, weiche auf Bahn 2 aus.
	elif (setup.laneID == 1 and obstacles2 < 500) and not (obstacles1 < 500):
		setup.laneID = 0 #s.o.; Bahn 2 -> Bahn 1

rospy.init_node('obstacle', anonymous=True)

sub_scan = rospy.Subscriber("/scan", LaserScan, scan_callback)
sub_odom = rospy.Subscriber("/localization/odom/5",Odometry, odom_callback)

try:
	rospy.spin()
	plotter()
except KeyboardInterrupt:
	print("Shutting down")



