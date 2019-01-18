import math

# Gebe nächst gelegensten Punkt auf der Linie wieder
def findpoint(x,y):
	ret = (0,0)
	if (x,y) == (195,215): #Spezialfall 1: Mitte oberer Kreis
        ret = (75,215)
    elif (x,y) == (405,215): #Spezialfall 2: Mitte unterer Kreis
        ret = (525,215)
    elif x <= 194: # Teil 1 (siehe Aufgabe 2, Aufteilung in Gebiete)
        if y < 215:
            ret = (195-round(math.sqrt((120**2)-(((120**2)*(y-215)**2)/((x-195)**2+(y-215)**2)))),215-round(math.sqrt((((120**2)*(y-215)**2)/((x-195)**2+(y-215)**2))))) #komplizierte, per Hand herbeigeführte Formel...
        else:
            ret = (195-round(math.sqrt((120**2)-(((120**2)*(y-215)**2)/((x-195)**2+(y-215)**2)))),215+round(math.sqrt((((120**2)*(y-215)**2)/((x-195)**2+(y-215)**2))))) #komplizierte, per Hand herbeigeführte Formel...
    elif 195 <= x <= 405 and y <= 215: # Teil 2 (siehe Aufgabe 2, Aufteilung in Gebiete)
        ret = (x,95)
    elif 195 <= x <= 405 and 216 <= y: # Teil 3 (siehe Aufgabe 2, Aufteilung in Gebiete)
        ret = (x,335)
    elif 406 <= x <= 600: # Teil 4 (siehe Aufgabe 2, Aufteilung in Gebiete)
        if y < 215:
            ret = (405+round(math.sqrt((120**2)-(((120**2)*(y-215)**2)/((x-405)**2+(y-215)**2)))),215-round(math.sqrt((((120**2)*(y-215)**2)/((x-405)**2+(y-215)**2))))) #komplizierte, per Hand herbeigeführte Formel...
        else:
            ret = (405+round(math.sqrt((120**2)-(((120**2)*(y-215)**2)/((x-405)**2+(y-215)**2)))),215+round(math.sqrt((((120**2)*(y-215)**2)/((x-405)**2+(y-215)**2))))) #komplizierte, per Hand herbeigeführte Formel...
    ret = (int(ret[0]),int(ret[1]))
    return ret
    

#gib die Distanz zum nächsten Punkt wieder
#Pythagoras
def distance(x,y):
	n_p = findpoint(x,y)
	# x^2 + y^2 = dist^2
	x_abs = (n_p[0]- x)**2
	y_abs = (n_p[1]- y)**2
	dist = math.sqrt(x_abs + y_abs)
	return dist


#Beispiel mit unserern ermittelten Werten

position_list = [[406, 333], [405, 336], [316, 344], [243, 347], [204, 346], [169, 342], [133, 325], [101, 291], [80, 252], [73, 212], [73, 171], [99, 138], [142, 111], [189, 102], [234, 103], [272, 99], [309, 99], [353, 99], [393, 99], [421, 99], [460, 107], [497, 133], [522, 175], [528, 223], [514, 271], [481, 309], [440, 330], [395, 336], [357, 337]]

for x in position_list:
	x.append(distance(x[0],x[1]))

print position_list


"""
OUTPUT
[[406, 333, 2.0], [405, 336, 2.0], [316, 344, 2.0], [243, 347, 2.0], [204, 346, 2.0], [169, 342, 2.0], [133, 325, 2.0], [101, 291, 2.0], [80, 252, 2.0], [73, 212, 2.0], [73, 171, 2.0], [99, 138, 2.0], [142, 111, 2.0], [189, 102, 2.0], [234, 103, 2.0], [272, 99, 2.0], [309, 99, 2.0], [353, 99, 2.0], [393, 99, 2.0], [421, 99, 2.0], [460, 107, 2.0], [497, 133, 2.0], [522, 175, 2.0], [528, 223, 2.0], [514, 271, 2.0], [481, 309, 2.0], [440, 330, 2.0], [395, 336, 2.0], [357, 337, 2.0]]
"""

