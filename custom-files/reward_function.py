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
    def smoothen(center_line, max_offset = 1.07*0.55*0.5, pp=0.10, p=0.05, c=0.70, n=0.05, nn=0.10, iterations=72, skip_step=1):
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

    MIN_SPEED = 1.2
    MAX_SPEED = 4.0
    LOOK_AHEAD = 10

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
        reward = max(1e-3, 1 - (abs(distance)/(params['track_width'])))
        return max(reward, 1e-3)
    
    def optimal_speed(self, params):
        optimal_velocities = SpeedUtils.optimal_velocity( self.get(), Path.MIN_SPEED, Path.MAX_SPEED, Path.LOOK_AHEAD )
        next = self.closest( (params['x'], params['y']) )[1]
        index = self.get().index( next )
        optimal_velocity = optimal_velocities[ index ]
        return optimal_velocity

    def optimal_speed_reward(self, params):
        optimal_speed = self.optimal_speed(params)
        diff = abs( params['speed'] - optimal_speed )/(Path.MAX_SPEED-Path.MIN_SPEED)
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
        reward = 1 - diff
        return max(reward, 1e-3)
    
def progress_reward_factor(params):
    # Read input variable
    steps = params['steps']
    progress = params['progress']

    # Total num of steps we want the car to finish the lap, it will vary depends on the track length
    # maybe 500, since track length is 354
    TOTAL_NUM_STEPS = 500

    # Initialize the reward with typical value
    factor = 1.0

    if steps == 0 or progress < 10:
        return 1

    # Give additional reward if the car pass every 100 steps faster than expected
    if (steps % 100) == 0 and progress > (steps / TOTAL_NUM_STEPS) * 100 :
        factor = factor*1.5

    return float(factor)
    
# def reward_function(params):

#     print('-'*100)
#     print('parmas: ', {key: value for key, value in params.items() if key!='waypoints'})
#     print('-'*100)

#     path_object = Path( params['waypoints'], 2)

#     off_track_penalty = -2.0
#     if path_object.optimal_speed(params) <= 2.5:
#         off_track_penalty = -5.0

#     if params["is_offtrack"] or params["is_crashed"]:
#         return off_track_penalty
    
#     distance_reward = path_object.on_track_reward( params )
#     steering_reward = SteeringUtils.reward( params )
#     speed_reward    = path_object.optimal_speed_reward( params )

#     # if path_object.optimal_speed(params) <= 2:
#     #     steering_reward = 1.5*steering_reward

#     reward = steering_reward + speed_reward + distance_reward
#     factor = progress_reward_factor(params)
#     reward = reward*factor

#     print('steering_reward: ', steering_reward, 'distance_reward: ', distance_reward, 'speed_reward: ', speed_reward, 'progress factor: ', factor, 'total: ', reward)

#     return reward

# def reward_function(params):

#     if params["all_wheels_on_track"] and params["steps"] > 0:
#         reward = ((params["progress"] / params["steps"]) * 100) + (params["speed"]**2)
#     else:
#         reward = 0.01
        
#     return float(reward)

import math


class Reward:
    EVERYTHING = None
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):

        # Import package (needed for heading)
        import math

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            
            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list 
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):

            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time/current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        def getEverything(racing_track, min_speed = 1.3, max_speed = 4, lookahead = 10):
            if Reward.EVERYTHING:
                return Reward.EVERYTHING

            def dist_2_points(x1, x2, y1, y2):
                return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

            velocity =  SpeedUtils.optimal_velocity(racing_track, min_speed, max_speed, lookahead)

            distance_to_prev = []
            for i in range(len(racing_track)):
                indexes = SpeedUtils.circle_indexes(racing_track, i, add_index_1=-1, add_index_2=0)[0:2]
                coords = [racing_track[indexes[0]],racing_track[indexes[1]]]
                dist_to_prev = dist_2_points(x1=coords[0][0], x2=coords[1][0], y1=coords[0][1], y2=coords[1][1])
                distance_to_prev.append(dist_to_prev)
                
            time_to_prev = [(distance_to_prev[i]/velocity[i]) for i in range(len(racing_track))]

            total_time = sum(time_to_prev)
            print(f"Total time for track, if racing line and speeds are followed perfectly: {total_time} s")

            # Now we have list with columns (x,y,speed,distance,time)
            racing_track_everything = []
            for i in range(len(racing_track)):
                racing_track_everything.append([racing_track[i][0],
                                                racing_track[i][1],
                                                velocity[i],
                                                time_to_prev[i]])
            # Round to 5 decimals
            racing_track_everything = np.around(racing_track_everything, 5).tolist()

            Reward.EVERYTHING = racing_track_everything

            return racing_track_everything

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]

        racing_track = getEverything(Path(params['waypoints'], 0).get(), Path.MIN_SPEED, Path.MAX_SPEED, Path.LOOK_AHEAD)

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.first_racingpoint_index is None:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1 
        STANDARD_TIME = 26
        FASTEST_TIME = 20
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3
            
        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3
            
        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 500 # should be adapted to track length and other rewards
        STANDARD_TIME = 26  # seconds (time that is easily done by model)
        FASTEST_TIME = 20  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                      (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward
        
        ## Zero reward if off track ##
        if is_offtrack == False:
            reward = 1e-3

        ####################### VERBOSE #######################
        
        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)
            
        #################### RETURN REWARD ####################
        
        # Always return a float value
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)