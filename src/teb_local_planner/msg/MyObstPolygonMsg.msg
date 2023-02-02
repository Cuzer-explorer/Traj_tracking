# Message that contains rotation angle and min,max of x,y of a obstacle envelope polygon
# They are acquired in this way: first, rotate a certain frame(here it's "laser") by some angle, to minimum the area of the obstacle envelope polygon in terms of the rotated frame, and put down the rotation angle as "theta". Then put down the min,max of coordinates of envelope polygon in terms of the rotated frame.

float64 theta
float64 xmin
float64 xmax
float64 ymin
float64 ymax


