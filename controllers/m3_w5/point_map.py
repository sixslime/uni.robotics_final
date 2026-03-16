import my_util as my
import math
import numpy as np
import my_robot

# -[ PointMap structure ]-
class PointMap:
    def __init__(self, center, width, threshold_sec, display, resolution, draw = False):
        self.center = center
        self.draw = draw
        self.origin = (center[0]-(width/2), center[1]-(width/2))
        self.width = width
        self.resolution = resolution
        self.pixel_size = width/self.resolution
        self.display = display
        self.threshold = int(threshold_sec*(1000/my_robot.TIME_STEP))
        self.map = np.zeros((self.resolution, self.resolution), dtype=int)
    
    # translates world pos to pixel pos:
    def getPixelValue(self, point):
        p = (point[0]-self.origin[0], point[1]-self.origin[1])
        vdot = [v/self.pixel_size for v in p]
        dot = [int(my.clamp(vdot[0], 0, self.resolution-1)), int(my.clamp(vdot[1], 1, self.resolution))]
        return (dot[0], self.resolution-dot[1])
    
    def getWorldValue(self, pixel):
        p = (pixel[0]*self.pixel_size, (self.resolution-pixel[1])*self.pixel_size)
        p = (p[0]+self.origin[0], p[1]+self.origin[1])
        return p

    # add a point to this map:
    def addPoint(self, point):
        px, py = self.getPixelValue(point)
        
        self.map[px, py] += 1
        value = self.map[px, py]
        
        if self.draw:
            fill_ratio = value/self.threshold
            color = 0x00FF00 if (fill_ratio >= 1) else my.colorMultiply((0.5, 0.8, 1), fill_ratio)
            self.display.setColor(color)
            self.display.drawPixel(px, py)
        
    
    # returns and optionally draws configuration space, given a radius:
    def getConfigurationSpace(self, radius):
        o = np.array([[False for _ in range(self.resolution)] for _ in range(self.resolution)])
        # create a circle mask:
        radius = radius/self.pixel_size
        diameter = radius*2
        mask_size = int(math.ceil(diameter))+1
        mask_center = (radius, radius)
        mask = np.array([[(math.dist((x, y), mask_center) < radius+0.25) for x in range(mask_size)] for y in range(mask_size)])
        # for every confirmed pixel on self.map, apply the circle mask:
        for x, y in my.range2d(0, self.resolution, 0, self.resolution):
            if self.map[x, y] < self.threshold:
                continue
            for rx, ry in my.range2d(0, mask_size, 0, mask_size):
                if mask[rx, ry] == False:
                    continue
                rx, ry = (int(rx+x-int(mask_size/2)), int(ry+y-int(mask_size/2)))
                if not my.withinRange(rx, 0, self.resolution):
                    continue
                if not my.withinRange(ry, 0, self.resolution):
                    continue
                if o[rx, ry] == True:
                    continue
                o[rx, ry] = True
                if self.draw == True:
                    self.display.setColor(0xFFFFFF)
                    self.display.drawPixel(rx, ry)
        return o
            
