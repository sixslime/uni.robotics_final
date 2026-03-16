# DESCRIPTION:
# small personal utilities.

HEX_MULTS = [65536, 256, 1]

PI = 3.14
INF = float('Inf')

def clamp(x, minv, maxv):
    return max(minv, min(x, maxv))

def sign(x):
    return 1 if x >= 0 else -1

# multiplies a (r, g, b) 0..1 color by a scalar, returning it in decimal form (for display.setColor()). 
def colorMultiply(normalizedColor, multiplier):
     terms = [int(c*(255*multiplier))*m for m, c in zip(HEX_MULTS, normalizedColor)]
     return terms[0] + terms[1] + terms[2]

def range2d(minx, maxx, miny, maxy):
    return [(x, y) for x in range(minx, maxx) for y in range(miny, maxy)]

def withinRange(x, min, max):
    return x >= min and x < max

def windows(list, window_size):
    return [list[i : i + window_size] for i in range(len(list) - window_size + 1)]

# bresenhams line algorithm.
def get_line(p0, p1):
    x0, y0 = p0
    x1, y1 = p1
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    steep = dy > dx
    if steep:
        x0, y0, x1, y1 = y0, x0, y1, x1
        dx, dy = dy, dx
    if x0 > x1:
        x0, x1, y0, y1 = x1, x0, y1, y0
        rev = True
    else:
        rev = False
    err, ystep, y = dx // 2, 1 if y0 < y1 else -1, y0
    pts = []
    for x in range(x0, x1 + 1):
        pts.append((y, x) if steep else (x, y))
        err -= dy
        if err < 0:
            y += ystep
            err += dx
    return pts[::-1] if rev else pts

# dda line algorithm.
def get_full_line(p0, p1):
    x0, y0 = p0
    x1, y1 = p1
    dx, dy = x1 - x0, y1 - y0
    steps = max(abs(dx), abs(dy))
    x_inc, y_inc = dx / steps, dy / steps
    x, y = x0, y0
    pts = []
    for _ in range(steps + 1):
        pts.append((round(x), round(y)))
        x += x_inc
        y += y_inc
    return pts

# flattens a list.
def flatten(dlist):
    # directly yoinked from stackoverflow, this syntax is dumb and stupid.
    return [e for elist in dlist for e in elist]

# get the rotation error between <source_rot> and what the rotation would be facing <destination>, positioned at <source>.
def rotationError(source, source_rot, destination):
    import numpy as np
    dest_rot = np.arctan2(destination[1]-source[1], destination[0]-source[0])
    rob_rot = np.arctan2(np.sin(source_rot), np.cos(source_rot))
    rot_err_raw = dest_rot - rob_rot
    
    rot_err = np.arctan2(np.sin(rot_err_raw), np.cos(rot_err_raw))
    if (rot_err > np.pi):
        rot_err -= 2*np.pi
    return rot_err