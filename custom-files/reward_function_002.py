"""
params = {
    "all_wheels_on_track": Boolean,        # flag to indicate if the agent is on the track
    "x": float,                            # agent's x-coordinate in meters
    "y": float,                            # agent's y-coordinate in meters
    "closest_objects": [int, int],         # zero-based indices of the two closest objects to the agent's current position of (x, y).
    "closest_waypoints": [int, int],       # indices of the two nearest waypoints.
    "distance_from_center": float,         # distance in meters from the track center 
    "is_crashed": Boolean,                 # Boolean flag to indicate whether the agent has crashed.
    "is_left_of_center": Boolean,          # Flag to indicate if the agent is on the left side to the track center or not. 
    "is_offtrack": Boolean,                # Boolean flag to indicate whether the agent has gone off track.
    "is_reversed": Boolean,                # flag to indicate if the agent is driving clockwise (True) or counter clockwise (False).
    "heading": float,                      # agent's yaw in degrees
    "objects_distance": [float, ],         # list of the objects' distances in meters between 0 and track_length in relation to the starting line.
    "objects_heading": [float, ],          # list of the objects' headings in degrees between -180 and 180.
    "objects_left_of_center": [Boolean, ], # list of Boolean flags indicating whether elements' objects are left of the center (True) or not (False).
    "objects_location": [(float, float),], # list of object locations [(x,y), ...].
    "objects_speed": [float, ],            # list of the objects' speeds in meters per second.
    "progress": float,                     # percentage of track completed
    "speed": float,                        # agent's speed in meters per second (m/s)
    "steering_angle": float,               # agent's steering angle in degrees
    "steps": int,                          # number steps completed
    "track_length": float,                 # track length in meters.
    "track_width": float,                  # width of the track
    "waypoints": [(float, float), ]        # list of (x,y) as milestones along the track center

}
"""
import math
import numpy as np
from scipy import signal

smoothPath = []

def calc_distance(prev_point, next_point):
    delta_x = next_point[0] - prev_point[0]
    delta_y = next_point[1] - prev_point[1]
    return math.hypot(delta_x, delta_y)

def smoothen(center_line, max_offset = 0.45305, pp=0.10, p=0.05, c=0.70, n=0.05, nn=0.10, iterations=72, skip_step=1):
    if max_offset < 0.0001:
        return center_line
    if skip_step < 1:
        skip_step = 1
    smoothed_line = center_line
    for i in range(0, iterations):
        smoothed_line = smooth_central_line_internal(center_line, max_offset, smoothed_line, pp, p, c, n, nn, skip_step)
    return smoothed_line

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
        while calc_distance(new_line[i], center_line[i]) >= max_offset:
            new_line[i][0] = (0.98 * new_line[i][0]) + (0.02 * center_line[i][0])
            new_line[i][1] = (0.98 * new_line[i][1]) + (0.02 * center_line[i][1])
    return new_line

def _get_waypoints(params):
    global smoothPath
    if smoothPath:
        return smoothPath
    smoothPath = smoothen( up_sample( params['waypoints'] ) )
    return smoothPath

def waypoint_at(params, index):
    '''map the real way point at index to the point in the smooth line and return that point'''
    return _get_waypoints(params)[index]

def get_smooth_point_for_waypoint(params, waypoint):
    index = params['waypoints'].index(waypoint)
    return waypoint_at(params, index)

def distance(p1, p2):
    """ Euclidean distance between two points """ 
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

def angle(p1, p2):
    """
    """
    dy = p2[1]-p1[1]
    dx = p2[0]-p1[0]
    return math.degrees(math.atan2(dy,dx))

def normalize_angle_to_360(angle):
    if angle < 0:
        return 360 + angle
    return angle

def up_sample(waypoints, factor = 10):
    """
    Adds extra waypoints in between provided waypoints
    :param waypoints:
    :param factor: integer. E.g. 3 means that the resulting list has 3 times as many points.
    :return:
    """
    return [ list(point) for point in list( signal.resample(np.array(waypoints), len(waypoints) * factor) ) ]

def get_waypoints(params):
    """ Way-points """
    waypoints = _get_waypoints(params)
    waypoints = waypoints[params["closest_waypoints"][1]: ]
    return waypoints

def calculate_angle(p1, p2, p3):
    # Calculate the angle between three points p1, p2, p3
    angle = math.degrees(
        math.atan2(p3[1] - p2[1], p3[0] - p2[0]) - math.atan2(p1[1] - p2[1], p1[0] - p2[0])
    )
    return angle % 360

def get_turn_points(coordinates):
    turn_points = []
    window_size = 8
    threshold_angle = 4.0  # Set a threshold angle to determine a significant turn

    for i in range(len(coordinates) - window_size + 1):
        window = coordinates[i : i + window_size]
        angles = [
            calculate_angle(window[j], window[j + 1], window[j + 2]) for j in range(window_size - 2)
        ]
        max_angle_change = abs( max(angles) - min(angles) )
        if max_angle_change >= threshold_angle:
            turn_points.append(coordinates[i])
    return turn_points

def is_a_turn_coming_up( params ):
    next_way_point = waypoint_at(params, params["closest_waypoints"][1]) #getting smooth point at the index of the closest waypoint
    if next_way_point in get_turn_points( _get_waypoints(params) ):
        return True
    return False

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

def is_higher_speed_favorable(params):
    """ no high difference in heading  """
    reward = 10.0
    # base reward * speed if straight path
    # base reward / speed if turn is comming up
    return reward * ( params["speed"] ** (-1 if is_a_turn_coming_up( params ) else 1) )

def is_steps_favorable(params):
    # if number of steps range (1-150) > (0.66 - 100)
    # if number of steps range (1-900) > (0.11 - 100)
    #############################################################################
    '''
    Example of using steps and progress
    '''

    # Read input variable
    steps = params['steps']
    progress = params['progress']

    # Total num of steps we want the car to finish the lap, it will vary depends on the track length
    TOTAL_NUM_STEPS = 300

    # Initialize the reward with typical value
    reward = 1.0

    # Give additional reward if the car pass every 100 steps faster than expected
    if (steps % 100) == 0 and progress > (steps / TOTAL_NUM_STEPS) * 100 :
        reward += 10.0

    return float(reward)

def get_heading_reward(params):
    ###############################################################################
    '''
    Example of using waypoints and heading to make the car point in the right direction
    '''
    # Initialize the reward with typical value
    reward = 10

    # Read input variables
    waypoints = _get_waypoints(params)
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']

    # Calculate the direction of the center line based on the closest waypoints
    next_point = waypoints[closest_waypoints[1]]
    prev_point = waypoints[closest_waypoints[0]]

    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
    # Convert to degree
    track_direction = math.degrees(track_direction)

    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = abs(track_direction - heading)
    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    # Penalize the reward if the difference is too large
    DIRECTION_THRESHOLD = 10.0
    if direction_diff > DIRECTION_THRESHOLD:
        reward *= 0.5

    return float(reward)

def is_progress_favorable(params):
    # progress range is 1-100 > reward range is 0-1
    return params["progress"] / 100

def following_smooth_path_reward(params):
    prev = waypoint_at( params, params['closest_waypoints'][0] )
    next = waypoint_at( params, params['closest_waypoints'][1] )
    self = (params['x'], params['y'])
    distance = distanceFromLine( prev, next, self )
    return 5/float(distance)

def score_steer_to_point_ahead(params):
    heading_reward      = get_heading_reward(params)
    steps_reward        = is_steps_favorable(params)
    progress_reward     = is_progress_favorable(params)
    speed_reward        = is_higher_speed_favorable(params)
    on_smooth_track_reward = following_smooth_path_reward(params)
    reward              = (speed_reward) * (heading_reward) + steps_reward + progress_reward + (3*on_smooth_track_reward)
    return reward

def calculate_reward(params):
    if params["is_offtrack"] or params["is_crashed"]:
        return -5.0
    return float(score_steer_to_point_ahead(params))

def reward_function(params):
    reward = float(calculate_reward(params))

    if reward <=0:
        ans = 10/(1 + np.exp(np.abs(reward)/100) )
        ans*=-1
    if reward >=0:
        ans = 100/(1 + np.exp(np.abs(reward)/100) )

    return ans