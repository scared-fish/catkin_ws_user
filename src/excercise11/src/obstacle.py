#!/usr/bin/env python

import math

def findpoint(point,laneID):

    x= point[0]
    y= point[1]
    # Distanz vom inneren Oval festlegen
    lanedist = 0
    if laneID == 1:
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

def getLidarInf(): #Lidar-Informationen sammeln
  A = []
  return A

def transLidarInf(): #Lidar-Information in Koordinaten übersetzen
  # In Abhängigkeit von getLidarInf
  A = []
  return A

def findRelevantObstacles(A,laneID): #Prüfen, ob es auf einer Bahn Hindernisse gibt
  ret = []
  for i in A:
    ref = findpoint(i,laneID)
    dist = math.sqrt((ref[0]-i[0])**2 + (ref[1]-i[1])**2)
    if dist <=15:
      ret.append(ref)
  return (ret)

def closestObstacle(currPos,laneID,A): #Das nahste Hindernisse auf einer Bahn finden
  mindist = 500
  for i in A:
    dist = findDistance(currPos,i,laneID,0)
    if dist < mindist:
      mindist = dist
      ret = i
  return (mindist)

def findDistance(point1,point2,laneID,distance):#Distanz zwischen 2 Punkten auf der
  #Bahn finden, wobei point1 vor point2 liegen muss.
  x1=point1[0]
  y1=point1[1]
  x2=point2[0]
  y2=point2[1]

  if laneID == 1:
    rad = 135
  else:
    rad = 165

  totUmf = rad*math.pi
  if y1 <= 215:
    beta = 90+math.degrees(math.asin((215-y1)/rad))
  else:
    beta = 90-math.degrees(math.asin((y1-215)/rad))

  if y2 <= 215:
    alpha = 90+math.degrees(math.asin((215-y2)/rad))
  else:
    alpha = 90-math.degrees(math.asin((y2-215)/rad))

  if x1 <= 194: #oberer Halbkreis
    umf = totUmf-(beta/180)*totUmf
    if x2 > 195: #Falls sich der 2.Punkt nich im gleichen Bereich befindet
      distance += umf
      newPoint1 = (195,215-rad)
    else:
      umf2 = totUmf-(alpha/180)*totUmf
      distance += umf-umf2
      newPoint1=point2

  elif 195 <= x1 <= 405 and y1 <= 215: #linke Gerade
    if 195 > x2 or x2 > 405 or y2 > 215:#Falls sich der 2.Punkt nich im gleichen Bereich befindet
      newPoint1 = (406,215-rad)
      distance += 406-x1
    else:
      newPoint1 = point2
      distance += x2-x1

  elif 195 <= x1 <= 405 and 216 <= y1: #rechte Gerade
    if 105 > x2 or x2 > 405 or y2 < 216:#Falls sich der 2.Punkt nich im gleichen Bereich befindet
      newPoint1 = (194,215+rad)
      distance += x1-194
    else:
      newPoint1 = point2
      distance += x1-x2

  elif 406 <= x1: #unterer Halbkreis
    umf = totUmf-(beta/180)*totUmf
    if x2 < 406:#Falls sich der 2.Punkt nich im gleichen Bereich befindet
      distance += umf
      newPoint1 = (405,215+rad)
    else:
      umf2 = totUmf-(alpha/180)*totUmf
      distance += umf-umf2
      newPoint1=point2

  if newPoint1 != point2:
    return (findDistance(newPoint1,point2,laneID,distance))
  else:
    return (int(distance))

def main():
  A=transLidarInf()
  obstacles1 = closestObstacle(findPoint(currPos,1),1,findRelevantObstacles(A,1))
  obstacles2 = closestObstacle(findPoint(currPos,2),2,findRelevantObstacles(A,2))

  if (laneID = 1 and obstacles1 < 50) or (laneID = 2 and obstacles2 < 50):
    stop() #Wenn das Hinderniss zu nah ist, halte an.
  elif (laneID = 1 and obstacles1 < 500) and not (obstacles2 < 500):
    driveOnLane(2) #Wenn auf Bahn 1 gefahren wird, und sich ein dort ein Hinderniss befindet, weiche auf Bahn 2 aus.
  elif (laneID = 2 and obstacles2 < 500) and not (obstacles1 < 500):
    driveOnLane(1)#s.o.; Bahn 2 -> Bahn 1


print (findRelevantObstacles([(60,215)],1))
print (closestObstacle((250,350),1,findRelevantObstacles([(60,215)],1)))
print (findDistance((250,350),(195,350),1,0))
