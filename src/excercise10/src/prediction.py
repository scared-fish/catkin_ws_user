import math

# Gebe naechst gelegensten Punkt auf der Linie wieder
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

def findNextPoint(point,laneID,distance):
    
    x= point[0]
    y= point[1]
    #radii der Halbkreise in Abhaengigkeit der laneID festlegen
    if laneID == 1:
        rad = 135
    else:
        rad = 165
    
    newPoint = (x,y)
    newDist = distance
    
    #Hilfsvariablen fuer die Berechnung der Punkte auf den Halbkreisen
    totUmf = rad*math.pi
    if y <= 215:
        beta = 90+math.degrees(math.asin((215-y)/rad))
    else:
        beta = 90-math.degrees(math.asin((y-215)/rad))
    
    if x <= 194: #oberer Halbkreis
        umf = totUmf-(beta/180)*totUmf
        alpha = min(((totUmf-umf+distance)/totUmf)*180,180)
        if alpha == 180:
            newPoint = (195,215-rad)
            newDist = distance - umf
        elif alpha <= 90:
            newX = rad*math.sin(math.radians(alpha))
            newPoint = (195-newX,215+math.sqrt(rad**2-newX**2))
            newDist = 0
        else:
            newX = rad*math.sin(math.radians(alpha))
            newPoint = (195-newX,215-math.sqrt(rad**2-newX**2))
            newDist = 0

    elif 195 <= x <= 405 and y <= 215: #linke Gerade
        if newDist <= 406-x:
            newPoint = (x+distance,y)
            newDist = 0
        else:
            newPoint = (406,y)
            newDist = distance - (406-x)

    elif 195 <= x <= 405 and 216 <= y: #rechte Gerade
        if distance <= x-194:
            newPoint = (x-distance,y)
            newDist = 0
        else:
            newPoint = (194,y)
            newDist = distance - (x-194)

    elif 406 <= x: #unterer Halbkreis
        beta = 180-beta
        umf = totUmf-(beta/180)*totUmf
        alpha = min(((totUmf-umf+distance)/totUmf)*180,180)
        if alpha == 180:
            newPoint = (405,215+rad)
            newDist = distance - umf
        elif alpha <= 90:
            newX = rad*math.sin(math.radians(alpha))
            newPoint = (405+newX,215-math.sqrt(rad**2-newX**2))
            newDist = 0
        else:
            newX = rad*math.sin(math.radians(alpha))
            newPoint = (405+newX,215+math.sqrt(rad**2-newX**2))
            newDist = 0

    if newDist > 0: #wenn noch Distanz uebrig ist, wird die Funktion rekursiv aufgerufen
        return findNextPoint(newPoint,laneID,newDist)
    else: # sonst wird der neue Punkt ausgegeben
        return (int(newPoint[0]),int(newPoint[1]))

def getClosestPoint(point,laneID,distance):
    ovalPoint = findpoint(point,laneID)
    ret = findNextPoint(ovalPoint,laneID,distance)
    return ret

print (getClosestPoint ((300,200),1,50))
print (getClosestPoint ((100,100),2,20))
