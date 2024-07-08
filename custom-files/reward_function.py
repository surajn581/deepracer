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
turnPoints = []
angleChange = []
optimalVelocity = []

def calc_distance(prev_point, next_point):
    delta_x = next_point[0] - prev_point[0]
    delta_y = next_point[1] - prev_point[1]
    return math.hypot(delta_x, delta_y)

def smoothen(center_line, max_offset = 0.31980, pp=0.10, p=0.05, c=0.70, n=0.05, nn=0.10, iterations=72, skip_step=1):
    print('calculating smooth path')
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

def circle_indexes(mylist, index_car, add_index_1=0, add_index_2=0):

    list_len = len(mylist)

    # if index >= list_len:
    #     raise ValueError("Index out of range in circle_indexes()")

    # Use modulo to consider that track is cyclical
    index_1 = (index_car + add_index_1) % list_len
    index_2 = (index_car + add_index_2) % list_len

    return [index_car, index_1, index_2]

def optimal_velocity(track, min_speed, max_speed, look_ahead_points):
    global optimalVelocity
    if optimalVelocity:
        return optimalVelocity

    print('caclulating optimal velocity')
    # Calculate the radius for every point of the track
    radius = []
    for i in range(len(track)):
        indexes = circle_indexes(track, i, add_index_1=-1, add_index_2=1)
        coords = [track[indexes[0]],
                  track[indexes[1]], track[indexes[2]]]
        radius.append(circle_radius(coords))

    # Get the max_velocity for the smallest radius
    # That value should multiplied by a constant multiple
    v_min_r = min(radius)**0.5
    constant_multiple = min_speed / v_min_r

    if look_ahead_points == 0:
        # Get the maximal velocity from radius
        max_velocity = [(constant_multiple * i**0.5) for i in radius]
        # Get velocity from max_velocity (cap at MAX_SPEED)
        velocity = [min(v, max_speed) for v in max_velocity]
        return velocity

    else:
        # Looks at the next n radii of points and takes the minimum
        # goal: reduce lookahead until car crashes bc no time to break
        LOOK_AHEAD_POINTS = look_ahead_points
        radius_lookahead = []
        for i in range(len(radius)):
            next_n_radius = []
            for j in range(LOOK_AHEAD_POINTS+1):
                index = circle_indexes(
                    mylist=radius, index_car=i, add_index_1=j)[1]
                next_n_radius.append(radius[index])
            radius_lookahead.append(min(next_n_radius))
        max_velocity_lookahead = [(constant_multiple * i**0.5)
                                  for i in radius_lookahead]
        velocity_lookahead = [min(v, max_speed)
                              for v in max_velocity_lookahead]
        
        optimalVelocity = velocity_lookahead
        return velocity_lookahead

def _get_waypoints(params):
    global smoothPath
    if smoothPath:
        return smoothPath
    smoothPath = smoothen( params['waypoints'] )
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
    global turnPoints
    global angleChange
    if turnPoints and angleChange:
        return turnPoints, angleChange
    
    turn_points = []
    angle_change = []
    window_size = 8
    threshold_angle = 4.5  # Set a threshold angle to determine a significant turn

    for i in range(len(coordinates) - window_size + 1):
        window = coordinates[i : i + window_size]
        angles = [
            calculate_angle(window[j], window[j + 1], window[j + 2]) for j in range(window_size - 2)
        ]
        max_angle_change = abs( max(angles) - min(angles) )
        if max_angle_change >= threshold_angle:
            turn_points.append(coordinates[i])
            angle_change.append(abs(max_angle_change))
    turnPoints = turn_points
    angleChange = angle_change
    return turnPoints, angleChange

def is_a_turn_coming_up( params ):
    waypoints = _get_waypoints(params)
    next_way_point = waypoints[params["closest_waypoints"][1]] #getting smooth point at the index of the closest waypoint
    tp, ac = get_turn_points( waypoints )
    if next_way_point in tp:
        return ac[ tp.index(next_way_point) ]
    return 0

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
    """ no high difference in heading
        output range: 1 - 10
    """
    global angleChange
    max_speed = 3.8
    min_speed = 1.25
    sigma     = (max_speed-min_speed)/6
    optimal_velocity_ = optimal_velocity( _get_waypoints(params), min_speed, max_speed, 10 )[ params['closest_waypoints'][1] ]
    # Calculate reward for speed
    speed_diff = abs(params['speed'] - optimal_velocity_)
    reward_speed = math.exp(-0.5*((speed_diff**2)/sigma))
    return reward_speed*10

def is_steering_angle_correct(params):

    reward = 20
    steer = params['steering_angle']
    if not is_a_turn_coming_up(params):
        reward = reward/( 1+abs(steer) )
    return reward

import math

MAX_TURN = 20

def calc_angle_bw_heading(p1, p2, heading):
    angle =  math.degrees(
        math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    ) -  heading
    if angle < 0:
        angle = angle + 360

    if angle > 180:
        angle = angle - 360
    
    return angle

def calc_angle(p1, p2):
    angle =  math.degrees(
        math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    )
    if angle < 0:
        angle = angle + 360

    if angle > 180:
        angle = angle - 360
    
    return angle

def calc_angle_bw_points(p1, p2, p3):
    angle =  math.degrees(
        math.atan2(p3[1] - p1[1], p3[0] - p1[0])
    ) -  math.degrees(
        math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    )
    if angle < 0:
        angle = angle + 360

    if angle > 180:
        angle = angle - 360
    
    return angle

def get_future_heading(heading, steering_angle):
    return heading + steering_angle

# def steering_angle_factor(params):
    MAX_TURN = 20

    waypoints = _get_waypoints(params)
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    steering = params['steering_angle']
    curr_point = (params['x'], params['y'])
    next_point = waypoints[ (closest_waypoints[1]+1)%len(waypoints) ]
    next_to_next_point = waypoints[ (closest_waypoints[1]+2)%len(waypoints) ]

    # W.r.t heading
    angle_of_coming_turn = calc_angle_bw_heading(curr_point, next_point, heading)
    # turn between points
    angle_bw_points = calc_angle_bw_points(curr_point, next_point, next_to_next_point)

    # if abs(angle_of_coming_turn) > 90:
    #     return 1e-6

    if abs(angle_of_coming_turn) <= 4 and abs(angle_bw_points) <= 4:
        # if angle of coming turn is less than 1 degrees meaning 
        # more or less a straight line, penalize any steering angle
        # above 0.5 degrees forcing the car to go straight
        return 10 if steering < 4 else 1e-6

    # if steering in opposite direction to angle, return negatively
    
    # If trck turns left and i turn right, i want to penalize
    if ( angle_bw_points > 0 and steering < 0 ) or ( steering > 0 and angle_bw_points < 0 ):
        return 1e-6    
        
    if abs(angle_bw_points) > MAX_TURN:
        return 1e-6

    # I feel the steering reward should be accoridng to angle_bw_points
    # if it is close to angle_bw_points then reward it

    # and heading reward should be close to angle_of_coming_turn
    steering_reward = 10 * math.exp(-abs(steering - angle_bw_points) / 100)
    heading_reward = 10 * math.exp(-abs(angle_of_coming_turn) / 100)
    
    return heading_reward * steering_reward

def is_heading_correct(params):
    # Read input variables
    waypoints = _get_waypoints(params)
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']

    # heading = params['steering_angle'] + heading

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
        return False
    return True

def get_heading_reward(params):
    ###############################################################################
    '''
    Example of using waypoints and heading to make the car point in the right direction
    output range: 0.01 to 10
    '''
    # Initialize the reward with typical value
    reward = 10
    if not is_heading_correct(params):
        reward *= 0.0001
    return float(reward)

def following_smooth_path_reward(params):
    '''
    output range: 0 to 10
    '''
    waypoints = _get_waypoints(params)
    closest_waypoints = params['closest_waypoints']
    next = waypoints[closest_waypoints[1]]
    prev = waypoints[closest_waypoints[0]]
    self = (params['x'], params['y'])
    distance = distanceFromLine( prev, next, self )
    return 10/( (1+float(distance))**4 )

def calc_sub_reward_and_aggregate(params):
    heading_reward          = get_heading_reward(params) # 0.01 to 10
    speed_reward            = is_higher_speed_favorable(params) # 1 to 10
    on_smooth_track_reward  = following_smooth_path_reward(params) # 0 to 10
    # reward                  = ( speed_reward + heading_reward + on_smooth_track_reward )**2 + ( speed_reward * heading_reward * on_smooth_track_reward )
    # reward                  = reward/100 # to ensure that the output is not crazily high
    print("heading_reward: ", heading_reward)
    print("speed_reward: ", speed_reward)
    print("on_smooth_track_reward: ", on_smooth_track_reward)

    reward = (speed_reward * heading_reward * 5) + (on_smooth_track_reward * 3)
    return reward

def calculate_reward(params):
    if params["is_offtrack"] or params["is_crashed"]:
        return 0.001
    return float(calc_sub_reward_and_aggregate(params))

def reward_function(params):
    reward = float(calculate_reward(params))
    print("Total reward: ", reward)
    return reward