from math import sin, cos, sqrt, atan2, pi

def get_polar(pos):
    r = sqrt(pos[0]**2 + pos[1]**2)
    alpha = atan2(pos[1], pos[0])
    return r, alpha

def get_cartesian(r, alpha):
    x = r * cos(alpha)
    y = r * sin(alpha)
    return int(x), int(y)

def middle(dps):
    pos_x, pos_y = 0, 0
    for x, y in dps:
        pos_x += x
        pos_y += y
    pos_x /= len(dps)
    pos_y /= len(dps)
    return int(pos_x), int(pos_y)

def rotate(dps, angle):
    new_dps = []
    m = middle(dps)
    for dp in dps:
        dp0 = [dp[0] - m[0], dp[1] - m[1]]
        dp0_pr, dp0_po = get_polar(dp0)
        dp0_po += angle
        r_dp0 = get_cartesian(dp0_pr, dp0_po)
        r_dp = [r_dp0[0] + m[0], r_dp0[1] + m[1]]
        new_dps.append(r_dp)
    return new_dps


# imported - https://www.codeproject.com/Tips/864704/Python-Line-Intersection-for-Pygame

def slope(p1, p2) :
   return (p2[1] - p1[1]) * 1. / (p2[0] - p1[0])
   
def y_intercept(slope, p1) :
   return p1[1] - 1. * slope * p1[0]
   
def intersect(line1, line2) :
   min_allowed = 1e-5   # guard against overflow
   big_value = 1e10     # use instead (if overflow would have occurred)
   m1 = slope(line1[0], line1[1])
   b1 = y_intercept(m1, line1[0])
   m2 = slope(line2[0], line2[1])
   b2 = y_intercept(m2, line2[0])
   if abs(m1 - m2) < min_allowed :
       x = big_value
   else :
       x = (b2 - b1) / (m1 - m2)
   y = m1 * x + b1
   y2 = m2 * x + b2
   return (int(x),int(y))

def segment_intersect(line1, line2) :
   intersection_pt = intersect(line1, line2)

   if (line1[0][0] < line1[1][0]) :
      if intersection_pt[0] < line1[0][0] or intersection_pt[0] > line1[1][0] :   
         return None
   else :
      if intersection_pt[0] > line1[0][0] or intersection_pt[0] < line1[1][0] :
         return None
         
   if (line2[0][0] < line2[1][0]) :
      if intersection_pt[0] < line2[0][0] or intersection_pt[0] > line2[1][0] :
         return None
   else :
      if intersection_pt[0] > line2[0][0] or intersection_pt[0] < line2[1][0] :
         return None

   return intersection_pt
