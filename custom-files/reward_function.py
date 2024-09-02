import math
import numpy as np
from scipy import signal

class SmoothPath:
    PATH = []

    @classmethod
    def init(cls, waypoints):
        cls.PATH = cls.smoothen(waypoints)
        return cls.PATH

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
    def smoothen(center_line, max_offset = 1.07*0.65*0.5, pp=0.10, p=0.05, c=0.70, n=0.05, nn=0.10, iterations=72, skip_step=1):
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

    MIN_SPEED = 1.3
    MAX_SPEED = 5.0
    MAX_SPEED_FOR_REWARD = 4.0
    LOOK_AHEAD = 12

    MAKE_SMOOTH = True
    UPSAMPLE    = False

    def __init__(self, waypoints, upsample = 1):
        if self.MAKE_SMOOTH:
            self._path = SmoothPath.init(waypoints)
        else:
            self._path = waypoints
        if upsample>1 and self.UPSAMPLE:
            self._path = Path.up_sample( self._path, upsample )

    def get(self):
        return self._path.tolist() if hasattr( self._path, 'tolist' ) else self._path

    @staticmethod
    def up_sample(waypoints, factor = 2):
        return [ list(point) for point in list( signal.resample(np.array(waypoints), len(waypoints) * factor) ) ]
    
    def closest(self, point, n = None):
        '''
        returns index of the closest n points in path
        '''
        n = n or len(self._path)
        distances = [ Utils.distance(point, path_point) for path_point in self._path ]
        offset = distances.index( min(distances) )
        path = [ self._path[ (i+offset)%len(self._path) ] for i in range( len(self._path) )  ]
        return path[:n]

    def closest_within(self, point, threshold = 0.9*1.07):
        closest = self.closest(point, len(self._path))
        for close_point in closest:
            if Utils.distance( point, close_point ) > threshold:
                return close_point

    def distance(self, point):
        closest = self.closest(point)
        prev = closest[0]
        nex = closest[1]
        return Utils.distanceFromLine(prev, nex, point)
    
    def on_track_reward(self, params):
        current_point = ( params['x'], params['y'] )
        distance = self.distance( current_point )
        reward = max(1e-3, 0.5 - (abs(distance)/(params['track_width'])))
        return max(reward, 1e-3)*0.5
    
    def optimal_speed(self, params):
        optimal_velocities = SpeedUtils.optimal_velocity( self.get(), Path.MIN_SPEED, Path.MAX_SPEED, Path.LOOK_AHEAD )
        next = self.closest( (params['x'], params['y']) )[1]
        index = self.get().index( next )
        optimal_velocity = optimal_velocities[ index ]
        return optimal_velocity

    def optimal_speed_reward(self, params):
        optimal_speed = self.optimal_speed(params)
        diff = abs( params['speed'] - optimal_speed )/(Path.MAX_SPEED_FOR_REWARD-Path.MIN_SPEED)
        reward = max(1e-3, 0.5 - diff)*2
        return reward
class SpeedUtils:

    OPTIMAL_VELOCITES = []

    @staticmethod
    def circle_radius(coords):

        # Flatten the list and assign to variables (makes code easier to read later)
        x1, y1, x2, y2, x3, y3 = [i for sub in coords for i in sub]

        a = x1*(y2-y3) - y1*(x2-x3) + x2*y3 - x3*y2
        b = (x1**2+y1**2)*(y3-y2) + (x2**2+y2**2)*(y1-y3) + (x3**2+y3**2)*(y2-y1)
        c = (x1**2+y1**2)*(x2-x3) + (x2**2+y2**2)*(x3-x1) + (x3**2+y3**2)*(x1-x2)
        d = (x1**2+y1**2)*(x3*y2-x2*y3) + (x2**2+y2**2) * \
            (x1*y3-x3*y1) + (x3**2+y3**2)*(x2*y1-x1*y2)

        # In case a is zero (so radius is infinity)
        try:
            r = abs((b**2+c**2-4*a*d) / abs(4*a**2)) ** 0.5
        except:
            r = 999

        return r

    @staticmethod
    def circle_indexes(mylist, index_car, add_index_1=0, add_index_2=0):

        list_len = len(mylist)

        # if index >= list_len:
        #     raise ValueError("Index out of range in circle_indexes()")

        # Use modulo to consider that track is cyclical
        index_1 = (index_car + add_index_1) % list_len
        index_2 = (index_car + add_index_2) % list_len

        return [index_car, index_1, index_2]

    @staticmethod
    def optimal_velocity(track, min_speed, max_speed, look_ahead_points):
        if SpeedUtils.OPTIMAL_VELOCITES:
            return SpeedUtils.OPTIMAL_VELOCITES

        # Calculate the radius for every point of the track
        radius = []
        for i in range(len(track)):
            indexes = SpeedUtils.circle_indexes(track, i, add_index_1=-1, add_index_2=1)
            coords = [track[indexes[0]],
                    track[indexes[1]], track[indexes[2]]]
            radius.append(SpeedUtils.circle_radius(coords))

        # Get the max_velocity for the smallest radius
        # That value should multiplied by a constant multiple
        v_min_r = min(radius)**0.5
        constant_multiple = min_speed / v_min_r
        # print(f"Constant multiple for optimal speed: {constant_multiple}")

        if look_ahead_points == 0:
            # Get the maximal velocity from radius
            max_velocity = [(constant_multiple * i**0.5) for i in radius]
            # Get velocity from max_velocity (cap at MAX_SPEED)
            velocity = [min(v, max_speed) for v in max_velocity]
            SpeedUtils.OPTIMAL_VELOCITES = velocity
            return velocity

        else:
            # Looks at the next n radii of points and takes the minimum
            # goal: reduce lookahead until car crashes bc no time to break
            LOOK_AHEAD_POINTS = look_ahead_points
            radius_lookahead = []
            for i in range(len(radius)):
                next_n_radius = []
                for j in range(LOOK_AHEAD_POINTS+1):
                    index = SpeedUtils.circle_indexes(
                        mylist=radius, index_car=i, add_index_1=j)[1]
                    next_n_radius.append(radius[index])
                radius_lookahead.append(min(next_n_radius))
            max_velocity_lookahead = [(constant_multiple * i**0.5)
                                    for i in radius_lookahead]
            velocity_lookahead = [min(v, max_speed)
                                for v in max_velocity_lookahead]
            
            new_velocity = []
            rev_v = velocity_lookahead[::-1]
            for i, vel in enumerate(rev_v, LOOK_AHEAD_POINTS):
                meanN = np.mean(rev_v[i-LOOK_AHEAD_POINTS:i])
                if vel > 2.1 and vel >= meanN:
                    new_velocity.append(meanN)
                else:
                    new_velocity.append(vel)

            new_velocity = new_velocity[::-1]

            SpeedUtils.OPTIMAL_VELOCITES = new_velocity
            return new_velocity

class Utils:

    @staticmethod
    def distance(p1, p2):
        """ Euclidean distance between two points """ 
        return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5
    
    @staticmethod
    def distanceFromLine(p1, p2, p3):
        '''
        calc the perpendicular dist of point p3 from the line passing through point p1 and p2
        '''
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3

        a = y1-y2
        b = x2-x1
        c = (x1-x2)*y1 + (y2-y1)*x1

        distance = abs( a*x3 + b*y3 + c )/math.sqrt( a**2 + b**2 )
        return distance
        
    @staticmethod
    def angle(dx, dy):
        '''given dy and dx find the angle in degrees'''
        return math.degrees(math.atan2(dy, dx))
    
    @staticmethod
    def angle_between_points(prev, nex):
        '''
        find angle between the next - prev
        '''
        return Utils.angle(nex[0] - prev[0], nex[1]-prev[1])
    
    @staticmethod
    def normalize_angle(angle):
        ang = angle % 360.0
        if ang <= 180.0:
            return ang
        else:
            return ang - 360
class SteeringUtils:

    @staticmethod
    def right_steering(params):
        current_point = ( params['x'], params['y'] )
        target_point = Path(params['waypoints'], 2).closest_within( current_point, 0.9*params['track_width'] )
        path_angle = Utils.angle_between_points(current_point, target_point)
        steering_angle = path_angle - params['heading']
        return Utils.normalize_angle(steering_angle)
    
    @staticmethod
    def reward(params):
        ideal_aangle = SteeringUtils.right_steering(params)
        current_angle = params['steering_angle']
        diff = abs(current_angle - ideal_aangle)/60.0
        reward = 0.5 - diff
        return max(reward, 1e-3)*0.5
    
def progress_reward_factor(params):
    # Read input variable
    steps = params['steps']
    progress = params['progress']

    # Total num of steps we want the car to finish the lap, it will vary depends on the track length
    # maybe 500, since track length is 354
    TOTAL_NUM_STEPS = 480

    # Initialize the reward with typical value
    factor = 1.0

    if steps == 0 or progress < 5:
        return 1

    # Give additional reward if the car pass every 100 steps faster than expected
    if (steps % 100) == 0 and progress > (steps / TOTAL_NUM_STEPS) * 100 :
        factor = factor*1.5

    return float(factor)
    
def reward_function(params):

    print('-'*100)
    print('parmas: ', {key: value for key, value in params.items() if key!='waypoints'})
    print('-'*100)

    path_object = Path( params['waypoints'], 2)

    off_track_penalty = -2.0
    if path_object.optimal_speed(params) <= 2.5:
        off_track_penalty = -5.0

    if params["is_offtrack"] or params["is_crashed"]:
        return off_track_penalty
    
    distance_reward = path_object.on_track_reward( params )
    steering_reward = SteeringUtils.reward( params )
    speed_reward    = path_object.optimal_speed_reward( params )

    # if path_object.optimal_speed(params) <= 2:
    #     steering_reward = 1.5*steering_reward

    reward = steering_reward + speed_reward + distance_reward
    factor = progress_reward_factor(params)
    reward = reward*factor

    print('steering_reward: ', steering_reward, 'distance_reward: ', distance_reward, 'speed_reward: ', speed_reward, 'progress factor: ', factor, 'total: ', reward)

    return reward