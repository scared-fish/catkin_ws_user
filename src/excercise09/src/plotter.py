
#!/usr/bin/env python2
from PIL import Image
from PIL import ImageDraw

position_list = [[406, 333], [405, 336], [316, 344], [243, 347], [204, 346], [169, 342], [133, 325], [101, 291], [80, 252], [73, 212], [73, 171], [99, 138], [142, 111], [189, 102], [234, 103], [272, 99], [309, 99], [353, 99], [393, 99], [421, 99], [460, 107], [497, 133], [522, 175], [528, 223], [514, 271], [481, 309], [440, 330], [395, 336], [357, 337]]

#exchange x with y since PIL is perceving it the other way around
new_list = []

for x in position_list:
	new_list.append([x[1],x[0]])

image_in = "/home/bilbo/Pictures/map.bmp"

img = Image.open(image_in).convert('RGB')

draw = ImageDraw.Draw(img)
r=3

for pos in new_list:
	draw.ellipse((pos[0] - r, pos[1] - r, pos[0] +r, pos[1] +r), fill=(255,0,0))

image_out = "/home/bilbo/Pictures/track.png"

img.save(image_out)

