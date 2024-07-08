import math

MAX_TURN = 20

def calc_angle(p1, p2):
    angle =  math.degrees(
        math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    )
    if angle < 0:
        angle = angle + 360

    return angle

def test_calc_angle():
    assert calc_angle((0,0), (1, 1)) == 45
    assert calc_angle((0,0), (-1, 1)) == 135
    assert calc_angle((0,0), (1, -1)) == 360-45
    assert calc_angle((0,0), (-1, -1)) == 360-135

def normalize(angle):
    if angle < 0:
        return angle + 360
    return angle


# 'x': 1.188178678564717, 'y': -0.9192579176817912, 'heading': -85.3885947558048, 'speed': 2.35766795836389, 'steering_angle': -20.0, 'progress': 47.84267000613694, 'closest_waypoints': [75, 76], 'steps': 305.0, 'is_offtrack': False, 'is_crashed': False}
# closest_waypoint[0] = array([ 0.99947775, -0.90993411])
# closest_waypoint[1] = array([ 0.87307025, -1.12438654])

def is_right_turn(
    curr_point, 
    next_point, 
    next_to_next_point 
):
    target_angle = calc_angle(curr_point, next_point)
    next_target_angle = calc_angle(curr_point, next_to_next_point)

    return next_target_angle < target_angle

def is_a_turn(
    curr_point, 
    next_point, 
    next_to_next_point 
):
    target_angle = calc_angle(curr_point, next_point)
    next_target_angle = calc_angle(curr_point, next_to_next_point)

    return abs(next_target_angle - target_angle) > 2

def test_is_right_turn():
    assert is_right_turn((0, 0), (1, 1), (1, 2)) == False
    assert is_right_turn((0, 0), (1, 1), (1, 0.5)) == True

    assert is_right_turn((0,0), (1, -1), (1, -0.5)) == False
    assert is_right_turn((0,0), (1, -1), (0.5, 1)) == True


# No current points to be used
def steering_angle_factor(
        curr_point, # let this be T+1
        next_point, # T+2
        next_to_next_point,# T+3 
        heading, 
        steering
    ):
    # W.r.t heading
    target_angle = calc_angle(curr_point, next_point)
    next_target_angle = calc_angle(curr_point, next_to_next_point)

    heading = normalize(heading)

    # This shoudl incorporate whethr im moving in 
    # right so it shoud be -ve
    # left if i need to move +ve
    diff_bw_heading_and_target = target_angle - heading
    diff_bw_heading_and_next_target = next_target_angle - heading
        
    if is_a_turn(curr_point, next_point, next_to_next_point):
        # If it is truing to go in opposite direction, penalize
        if is_right_turn(curr_point, next_point, next_to_next_point) and steering > 0:
            return 1e-6
        
        if not is_right_turn(curr_point, next_point, next_to_next_point) and steering < 0:
            return 1e-6
    
    # If heading is close to target, steer should be close to next angle target
    if abs(diff_bw_heading_and_target) <= 4:
        return math.exp(-0.5 * min(MAX_TURN, abs(
            steering - 
            diff_bw_heading_and_next_target
        ) ) / MAX_TURN )

    # if heading is not close to target, heading + steer should move towards current angle
    return math.exp(-0.5 * min(MAX_TURN, abs(
        steering -
        diff_bw_heading_and_target
    )) / MAX_TURN )    

def test_steering_opposite_turns():
    assert steering_angle_factor((0, 0), (1, 1), (1, 1.2), 45, -20) == 1e-6
    assert steering_angle_factor((0, 0), (1, 1), (1, 0.8), 45, 10) == 1e-6
    assert steering_angle_factor((0, 0), (1, -1), (1, -0.8), 45, -10) == 1e-6
    assert steering_angle_factor((0, 0), (1, -1), (1, -1.2), 45, 10) == 1e-6

def test_smooth_roads():
    # When the line is straight or the current curve is less than 5 degrees, focus on the next curve
    assert steering_angle_factor((0, 0), (1, 1), (2, 2), 45, 2) > steering_angle_factor((0, 0), (1, 1), (2, 2), 45, 5)
    # 3 degrees is the required steer to be smooth (operator)
    assert steering_angle_factor((0, 0), (1, 1), (1, 1.1), 45, 3) > steering_angle_factor((0, 0), (1, 1), (1, 1.1), 45, 1)

    # because the curve should be towards -2 and not 2
    assert steering_angle_factor((0, 0), (1, -1), (1, -1.1), -45, -2) > steering_angle_factor((0, 0), (1, -1), (1, -1.1), -45, 2)

    assert abs( steering_angle_factor((0, 0), (-1, -0.5), (-1, -0.6), -156, 6) - 0.97 ) < 0.1

    # When we curve in the different direction
    assert steering_angle_factor((0, 0), (-1, -0.5), (-1, -0.6), -156, -2) == 1e-6
    assert steering_angle_factor((0, 0), (-1, -0.5), (-1, -0.4), -156, 2) == 1e-6

def test_turning():
    assert steering_angle_factor((0,0), (1, 1), (1, 2), 10, 20) > steering_angle_factor((0,0), (1, 1), (1, 2), 10, 10)
    # already over steered. Reward lesser steer
    assert steering_angle_factor((0, 0), (-1, -0.5), (-1, -0.6), -146, 0) > steering_angle_factor((0, 0), (-1, -0.5), (-1, -0.6), -146, 4)

def run_tests():
    test_is_right_turn()
    test_calc_angle()
    test_smooth_roads()
    test_steering_opposite_turns()
    test_turning()