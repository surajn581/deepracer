import math

calledOnce = False
optimumpointlist = []

class PARAM:
    total_reward = None
    avg_reward = None
    actual_steps = 0
    accumulated_progress_reward = 0

def straightening_unnecessary_curves(race_line):
    imp_race_line = race_line
    unnecessary_curves_list = [[55,66],[79,93],[24,32]]
    for each_section in unnecessary_curves_list:
        section_start_idx = each_section[0]
        section_end_idx = each_section[1]
        if section_end_idx > section_start_idx:
            section_length = section_end_idx - section_start_idx - 2
        section_start_coords = race_line[section_start_idx]
        section_end_coords = race_line[section_end_idx]
        dx = (section_end_coords[0] - section_start_coords[0]) / (section_length + 2)
        dy = (section_end_coords[1] - section_start_coords[1]) / (section_length + 2)
        for idx in range(0, section_length+2):  
            imp_x_cord = section_start_coords[0] + idx * dx
            imp_y_cord = section_start_coords[1] + idx * dy
            imp_race_line[section_start_idx+idx] = (imp_x_cord,imp_y_cord)
    return imp_race_line
def singlecall(params):
    print("Calling singlecall function..")
    global calledOnce
    waypoints = params['waypoints']
    i = 0
    now = len(waypoints)
    for point in waypoints:
        estimated_x_cord = (point[0] + 2*waypoints[(i + 1) % now][0] + 2*waypoints[(i - 1) % now][0] +
                            2*waypoints[(i + 2) % now][0] + 2*waypoints[(i - 2) % now][0] + waypoints[(i + 3) % now][0] +
                            waypoints[(i - 3) % now][0] + waypoints[(i + 4) % now][0] + waypoints[(i - 4) % now][0] +
                            waypoints[(i + 5) % now][0] + waypoints[(i - 5) % now][0] + waypoints[(i + 6) % now][0] +
                            waypoints[(i - 6) % now][0] + waypoints[(i + 7) % now][0] + waypoints[(i - 7) % now][
                                0]) / 19
        estimated_y_cord = (point[1] + 2*waypoints[(i + 1) % now][1] + 2*waypoints[(i - 1) % now][1] +
                            2*waypoints[(i + 2) % now][1] + 2*waypoints[(i - 2) % now][1] + waypoints[(i + 3) % now][1] +
                            waypoints[(i - 3) % now][1] + waypoints[(i + 4) % now][1] + waypoints[(i - 4) % now][1] +
                            waypoints[(i + 5) % now][1] + waypoints[(i - 5) % now][1] + waypoints[(i + 6) % now][1] +
                            waypoints[(i - 6) % now][1] + waypoints[(i + 7) % now][1] + waypoints[(i - 7) % now][
                                1]) / 19
        optimumpointlist.append((estimated_x_cord, estimated_y_cord))
        i += 1
    
    finaloptimumpointlist = straightening_unnecessary_curves(optimumpointlist)
    
    print("key: >@{%6g742p#p2[!fp2jf72gh824pf?24[f*247g49n4")
    print(params['waypoints'])
    print(optimumpointlist)
    print(finaloptimumpointlist)
    calledOnce = True


def dist(point1, point2):
    return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5
def rect(r, theta):
    x = r * math.cos(math.radians(theta))
    y = r * math.sin(math.radians(theta))
    return x, y
def polar(x, y):
    r = (x ** 2 + y ** 2) ** .5
    theta = math.degrees(math.atan2(y, x))
    return r, theta
def angle_mod_360(angle):
    n = math.floor(angle / 360.0)
    angle_between_0_and_360 = angle - n * 360.0
    if angle_between_0_and_360 <= 180.0:
        return angle_between_0_and_360
    else:
        return angle_between_0_and_360 - 360
def get_waypoints_ordered_in_driving_direction(params):
    if params['is_reversed']:
        return list(reversed(optimumpointlist))
    else:
        return optimumpointlist
def up_sample(waypoints, factor):
    p = waypoints
    n = len(p)
    return [[i / factor * p[(j + 1) % n][0] + (1 - i / factor) * p[j][0],
             i / factor * p[(j + 1) % n][1] + (1 - i / factor) * p[j][1]] for j in range(n) for i in range(factor)]
def get_target_point(params):
    waypoints = up_sample(get_waypoints_ordered_in_driving_direction(params), 20)
    car = [params['x'], params['y']]
    distances = [dist(p, car) for p in waypoints]
    min_dist = min(distances)
    i_closest = distances.index(min_dist)
    n = len(waypoints)
    waypoints_starting_with_closest = [waypoints[(i + i_closest) % n] for i in range(n)]
    r = params['track_width'] * 0.9
    is_inside = [dist(p, car) < r for p in waypoints_starting_with_closest]
    i_first_outside = is_inside.index(False)
    if i_first_outside < 0:
        return waypoints[i_closest]
    return waypoints_starting_with_closest[i_first_outside]
def get_target_steering_degree(params):
    tx, ty = get_target_point(params)
    car_x = params['x']
    car_y = params['y']
    dx = tx - car_x
    dy = ty - car_y
    heading = params['heading']
    _, target_angle = polar(dx, dy)
    steering_angle = target_angle - heading
    return angle_mod_360(steering_angle)
def get_steer_to_point_ahead_reward(params):
    best_stearing_angle = get_target_steering_degree(params)
    steering_angle = params['steering_angle']
    error = (steering_angle - best_stearing_angle) / 60.0
    score = 1.0 - abs(error)
    #return max(score, 0.01)**2
    return max(score, 0.0001)


def get_closest_waypoints_for_optimized_waypoints(waypoints, params):
    list_of_points = waypoints
    agents_position = [params['x'], params['y']]
    position_waypoint_matrix = {}
    for each_point in list_of_points:
        dist_agent_and_point = dist(agents_position, each_point)
        position_waypoint_matrix[dist_agent_and_point] = {}
        position_waypoint_matrix[dist_agent_and_point]["index"] = list_of_points.index(each_point)
    sorted_position_waypoint_matrix = {}
    for each_key in sorted(position_waypoint_matrix):
        sorted_position_waypoint_matrix[each_key] = {}
        sorted_position_waypoint_matrix[each_key] = (position_waypoint_matrix[each_key])
    keys_sorted_position_waypoint_matrix = list(sorted_position_waypoint_matrix.keys())
    nearest_point_1 = int(sorted_position_waypoint_matrix[keys_sorted_position_waypoint_matrix[0]]['index'])
    nearest_point_2 = int(sorted_position_waypoint_matrix[keys_sorted_position_waypoint_matrix[1]]['index'])
    if nearest_point_1 > nearest_point_2:
        return [nearest_point_2, nearest_point_1]
    return [nearest_point_1, nearest_point_2]
def get_distance_from_racing_line_reward(params):
    optimized_closest_waypoints = get_closest_waypoints_for_optimized_waypoints(optimumpointlist, params)
    prev_point = optimumpointlist[optimized_closest_waypoints[0]]
    next_point = optimumpointlist[optimized_closest_waypoints[1]]
    try:
        dist_cal_sim = (next_point[0] - prev_point[0]) / (next_point[1] - prev_point[1])
    except:
        dist_cal_sim = 1
    distance_from_racing_line = abs(params['x'] - dist_cal_sim * params['y'] + dist_cal_sim * next_point[1] - next_point[0]) / abs(math.sqrt(1 + dist_cal_sim ** 2))
    #distance_from_racing_line /= params['track_width'] * 0.5
    distance_from_racing_line /= params['track_width'] * 0.75
    if distance_from_racing_line >= 1:
        dist_reward = 0
    else:
        dist_reward = 1.0 - distance_from_racing_line
    return optimized_closest_waypoints, max(dist_reward,0.0001)


def check_for_unpardonable_action(params):
    if params['is_offtrack']:
        return True
    # TODO: more conditions to be added if required
    return False


def past_heading_calculation(optimized_closest_waypoints):
    no_of_points = len(optimumpointlist)
    prev_point_1 = optimumpointlist[(optimized_closest_waypoints[0]) % no_of_points]
    prev_point_2 = optimumpointlist[(optimized_closest_waypoints[1]) % no_of_points]
    next_point_1 = optimumpointlist[(optimized_closest_waypoints[1]) % no_of_points]
    next_point_2 = optimumpointlist[(optimized_closest_waypoints[1] + 1) % no_of_points]
    prev_point = [(prev_point_2[0] + prev_point_1[0]) / 2, (prev_point_2[1] + prev_point_1[1]) / 2]
    next_point = [(next_point_2[0] + next_point_1[0]) / 2, (next_point_2[1] + next_point_1[1]) / 2]
    expected_past_heading = math.degrees(math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0]))
    return expected_past_heading
def future_heading_calculation(optimized_closest_waypoints):
    no_of_points = len(optimumpointlist)
    prev_point_1 = optimumpointlist[(optimized_closest_waypoints[0]) % no_of_points]
    prev_point_2 = optimumpointlist[(optimized_closest_waypoints[1]) % no_of_points]
    next_point_1 = optimumpointlist[(optimized_closest_waypoints[1] + 4) % no_of_points]
    next_point_2 = optimumpointlist[(optimized_closest_waypoints[1] + 5) % no_of_points]
    prev_point = [(prev_point_2[0] + prev_point_1[0]) / 2, (prev_point_2[1] + prev_point_1[1]) / 2]
    next_point = [(next_point_2[0] + next_point_1[0]) / 2, (next_point_2[1] + next_point_1[1]) / 2]
    expected_future_heading = math.degrees(math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0]))
    return expected_future_heading
def check_for_speed_zone(optimized_closest_waypoints):
    past_heading = past_heading_calculation(optimized_closest_waypoints)
    future_heading = future_heading_calculation(optimized_closest_waypoints)
    diff_heading = abs(future_heading - past_heading)
    if diff_heading < 7:
        return True
    return False
def get_speed_reward(params,optimized_closest_waypoints):
    is_speed_zone = check_for_speed_zone(optimized_closest_waypoints)
    speed_reward = 0
    for idx in optimized_closest_waypoints:
        if idx < 4 or idx > 144 or 23 < idx < 32 or 113 < idx < 126:
            is_speed_zone = True
        #elif 55 < idx < 60 or 80 < idx < 87 :
        #    is_speed_zone = True
        else :
            is_speed_zone = False
    if is_speed_zone:
        #speed_reward = 0.16 * (params['speed']-1.5)**2 
        speed_reward = 0.4 * (params['speed']-1.5)
    return is_speed_zone, max(speed_reward, 0.0001)


def get_progress_reward(params):
    progress_reward = max(10 * params['progress']/params['steps'] - 3, 0.0001)
    PARAM.accumulated_progress_reward += progress_reward*params['steps']/10
def check_progress_reward_criteria(reward):
    if reward > PARAM.avg_reward:
        return True
    return False


def reward_function(params):
    global calledOnce
    if not calledOnce:
        singlecall(params)
    else:
        pass

    is_unpardonable_action = check_for_unpardonable_action(params)
    if is_unpardonable_action:
        return float(-2.0)
    optimized_closest_waypoints, distance_reward = get_distance_from_racing_line_reward(params)
    steer_reward = get_steer_to_point_ahead_reward(params)
    is_speed_zone, speed_reward = get_speed_reward(params,optimized_closest_waypoints)

    if not is_speed_zone:
        reward = 1.5*steer_reward + 1.5*distance_reward
    else:
        reward = 0.6*steer_reward + 1.2*speed_reward + 1.2*distance_reward

    if params['progress'] > 10 and params['progress'] % 10 == 0:
        get_progress_reward(params)
    progress_reward_check = False

    if PARAM.accumulated_progress_reward != 0:
        if PARAM.total_reward is not None and PARAM.avg_reward is not None:
            progress_reward_check = check_progress_reward_criteria(reward)
        if progress_reward_check:
            # progress reward to be added in the total reward if the values come right
            # reward += accumulated_progress_reward
            PARAM.accumulated_progress_reward = 0

    print('r:', reward, ' str:', steer_reward, ' dr:', distance_reward)
    print('isz:', is_speed_zone, ' spr:', speed_reward)
    print('pgrc:', progress_reward_check, ' acpgr:', PARAM.accumulated_progress_reward, ' avpgr:', PARAM.avg_reward)
    print('x:', params['x'], ' y:', params['y'], ' cwi:', optimized_closest_waypoints)
    print('cw0:', optimumpointlist[optimized_closest_waypoints[0]][0], optimumpointlist[optimized_closest_waypoints[0]][1])
    print('cw1:', optimumpointlist[optimized_closest_waypoints[1]][0], optimumpointlist[optimized_closest_waypoints[1]][1])

    if PARAM.total_reward is not None and PARAM.avg_reward is not None:
        PARAM.total_reward += reward
        PARAM.actual_steps += 1
        PARAM.avg_reward = PARAM.total_reward / PARAM.actual_steps
    else:
        PARAM.total_reward = reward
        PARAM.actual_steps += 1
        PARAM.avg_reward = PARAM.total_reward / max(PARAM.actual_steps, 1)
    return reward
