import math
import numpy as np
from scipy import signal

class SmoothPath:
    PATH = []

    @classmethod
    def init(cls, waypoints):
        cls.PATH = cls.smoothen(waypoints)

    @classmethod
    def path(cls):
        if not cls.PATH:
            raise Exception('must init first')
        return cls.PATH

    @staticmethod
    def calc_distance(prev_point, next_point):
        delta_x = next_point[0] - prev_point[0]
        delta_y = next_point[1] - prev_point[1]
        return math.hypot(delta_x, delta_y)

    @staticmethod
    def smoothen(center_line, max_offset = 0.31980, pp=0.10, p=0.05, c=0.70, n=0.05, nn=0.10, iterations=72, skip_step=1):
        if SmoothPath.PATH:
            return SmoothPath.PATH
        
        print('calculating smooth path')
        if max_offset < 0.0001:
            return center_line
        if skip_step < 1:
            skip_step = 1
        smoothed_line = center_line
        for i in range(0, iterations):
            smoothed_line = SmoothPath.smooth_central_line_internal(center_line, max_offset, smoothed_line, pp, p, c, n, nn, skip_step)
        return smoothed_line

    @staticmethod
    def smooth_central_line_internal(center_line, max_offset, smoothed_line, pp, p, c, n, nn, skip_step):
        length = len(center_line)
        new_line = [[0.0 for _ in range(2)] for _ in range(length)]
        for i in range(0, length):
            wpp = smoothed_line[(i - 2 * skip_step + length) % length]
            wp = smoothed_line[(i - skip_step + length) % length]
            wc = smoothed_line[i]
            wn = smoothed_line[(i + skip_step) % length]
            wnn = smoothed_line[(i + 2 * skip_step) % length]
            new_line[i][0] = pp * wpp[0] + p * wp[0] + c * wc[0] + n * wn[0] + nn * wnn[0]
            new_line[i][1] = pp * wpp[1] + p * wp[1] + c * wc[1] + n * wn[1] + nn * wnn[1]
            while SmoothPath.calc_distance(new_line[i], center_line[i]) >= max_offset:
                new_line[i][0] = (0.98 * new_line[i][0]) + (0.02 * center_line[i][0])
                new_line[i][1] = (0.98 * new_line[i][1]) + (0.02 * center_line[i][1])
        return new_line
    

class Path:
    def __init__(self, waypoints, reversed = False, upsample = True):
        self._path = SmoothPath.init(waypoints)
        if upsample:
            self._path = self.up_sample( self._path )

    def up_sample(waypoints, factor = 20):
        return [ list(point) for point in list( signal.resample(np.array(waypoints), len(waypoints) * factor) ) ]
    
    def closest(self, point, n):
        '''
        returns index of the closest n points in path
        '''
        distances = [ Utils.distance(point, path_point) for path_point in self._path ]
        offset = distances.index( min(distances) )
        path = [ path[ (i+offset)%len(self._path) ] for i in range( len(self._path) )  ]
        return path[:n]

    def closest_within(self, point, threshold = 0.9*1.067):
        closest = self.closest(point, len(self._path))
        for close_point in closest:
            if Utils.distance( point, close_point ) > threshold:
                return close_point

class PrevRewards:
    pass

class Utils:

    @staticmethod
    def distance(p1, p2):
        """ Euclidean distance between two points """ 
        return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5
    
    @staticmethod
    def rect(r, theta):
        x = r * math.cos(math.radians(theta))
        y = r * math.sin(math.radians(theta))
        return x, y
    
    @staticmethod
    def polar_coordinates(x, y):
        r = (x ** 2 + y ** 2) ** .5
        theta = math.degrees(math.atan2(y, x))
        return r, theta
    
    @staticmethod
    def normalize_angle(angle):
        n = math.floor(angle / 360.0)
        ang = angle - n * 360.0
        if ang <= 180.0:
            return ang
        else:
            return ang - 360
        
