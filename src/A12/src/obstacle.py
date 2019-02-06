#!/usr/bin/env python2

import roslib
import rospy
import time
import datetime
import math
import subprocess
import numpy as np
from std_msgs.msg import Int16
from PIL import Image
from PIL import ImageDraw
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# fuer realtime data
global_curPos = (0,0)
global_orientation = 0.0
scanner_list = []

#bekomme die odometry daten
def odom_callback(data):
  # x,y
	global global_curPos
	# der Winkel in Rad
	global global_orientation

	cur_x = int(round(data.pose.pose.position.x * 100))
	cur_y = int(round(data.pose.pose.position.y * 100))

	global_curPos = (cur_x, cur_y)

	cur_z = data.pose.pose.orientation.z
	cur_w = data.pose.pose.orientation.w

	global_orientation = np.arccos(cur_w)*2 * (np.sign(cur_z) * (-1))

# bekommt die scan daten
def scan_callback(data):
	distances = data.ranges
	global scanner_list
	scanner_list = []
	# in relev_d ist kein fluessiger Uebergang dies wird
	# in den weiteren Rechnungen aber berücksichtigt
	# inf -> 0
	# alles ueber 1,5m -> 0
	relev_d = distances[0:45] + distances[314:359]
	for x in relev_d:
		if x >=1.3:
			scanner_list.append(0)
		else:
			scanner_list.append(x-0.2)

def findpoint(point,laneID): #Finde den nahsten Punkt auf einer bestimmen Spur

	x= point[0]
	y= point[1]
	# Distanz vom inneren Oval festlegen
	lanedist = 0
	if laneID == 0:
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
		if 0 <= C[i] < math.pi / 2:#Je nachdem, in welche Richtung der Winkel zeigt, muss der Offset addiert oder subtrahiert werden
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


def closestObstacles(currPos,laneID,A): #Das nahste Hinderniss auf einer Bahn finden
  distances = []
	for i in A:
		dist = findDistance(currPos,i,laneID,0)#Berechne die Distanz vom Auto zu den relevanten Obstacles
		distances.append(dist)
  if len(distances)>2: #Um false-positives entgegenzuwirken
    return (distances)
  else:
    return ([100]) #Also keine relevanten Obstacles


def findDistance(point1,point2,laneID,distance):#Distanz zwischen 2 Punkten auf der
	#Bahn finden, wobei point1 vor point2 liegen muss.
	x1=point1[0]
	y1=point1[1]
	x2=point2[0]
	y2=point2[1]

	if laneID == 0: #Radius der Kreise, basieren auf der Bahn festlegen
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


def obstacle_on_lane():
	global scanner_list
	global global_curPos
	global global_orientation
	pub = rospy.Publisher('/obstacle_on_lane', Int8, queue_size=10)
	rospy.init_node('obstacles', anonymous=True)
	rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    ob_int = 100 # Hier fuehren wir eine neue Logik ein:
		# 100 : lane 0 und 1 sind frei
    # 101 : lane 0 ist frei lane 1 nicht
    # 110 : lane 0 ist nicht frei lane 1 ist frei
    # 111 : keine lane frei
    # Wobei lane 0 die innere und 1 die aeussere ist
    obstacleList=transLidarInf(scanner_list, global_curPos, global_orientation) #Fetched die Lidar-Informationen

    obstacles0 = closestObstacles(findPoint(global_curPos,0),0,findRelevantObstacles(obstacleList,0)) #Nahstes Obstacle auf Bahn 0
    obstacles1 = closestObstacles(findPoint(global_curPos,1),1,findRelevantObstacles(obstacleList,1)) #Nahstes Obstacle auf Bahn 1
    if (min(obstacles0) < 100) and (min(obstacles1) < 100): #Wenn auf beiden Bahnen relevante Hindernisse sind
      ob_int = 111
    elif (min(obstacles0) < 100): #Wenn nur auf Bahn 0 Hindernisse sind
      ob_int = 110
    elif (min(obstacles1) < 100): #Wenn nur auf Bahn 1 Hindernisse sind
      ob_int = 101
    else: #Wenn es keinerlei relevante Hindernisse gibt
      ob_int = 100
    rospy.loginfo(ob_int)
    pub.publish(ob_int)
    rate.sleep()

sub_scan = rospy.Subscriber("/scan", LaserScan, scan_callback)
sub_odom = rospy.Subscriber("/localization/odom/5",Odometry, odom_callback)

if __name__ == '__main__':
  try:
		obstacle_on_lane()
	except rospy.ROSInterruprExeption as err:
    print(err)

