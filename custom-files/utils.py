class SteeringUtils:

    MAX_TURN = 20

    @staticmethod
    def calc_angle(p1, p2):
        angle =  math.degrees(
            math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        )
        if angle < 0:
            angle = angle + 360

        return angle

    @staticmethod
    def test_calc_angle():
        assert SteeringUtils.calc_angle((0,0), (1, 1)) == 45
        assert SteeringUtils.calc_angle((0,0), (-1, 1)) == 135
        assert SteeringUtils.calc_angle((0,0), (1, -1)) == 360-45
        assert SteeringUtils.calc_angle((0,0), (-1, -1)) == 360-135

    @staticmethod
    def normalize(angle):
        if angle < 0:
            return angle + 360
        return angle

    @staticmethod
    def is_right_turn(
        curr_point, 
        next_point, 
        next_to_next_point 
    ):
        target_angle = SteeringUtils.calc_angle(curr_point, next_point)
        next_target_angle = SteeringUtils.calc_angle(curr_point, next_to_next_point)

        return next_target_angle < target_angle

    @staticmethod
    def is_a_turn(
        curr_point, 
        next_point, 
        next_to_next_point 
    ):
        target_angle = SteeringUtils.calc_angle(curr_point, next_point)
        next_target_angle = SteeringUtils.calc_angle(curr_point, next_to_next_point)

        return abs(next_target_angle - target_angle) > 2

    @staticmethod
    def test_is_right_turn():
        assert SteeringUtils.is_right_turn((0, 0), (1, 1), (1, 2)) == False
        assert SteeringUtils.is_right_turn((0, 0), (1, 1), (1, 0.5)) == True

        assert SteeringUtils.is_right_turn((0,0), (1, -1), (1, -0.5)) == False
        assert SteeringUtils.is_right_turn((0,0), (1, -1), (0.5, 1)) == True


    # No current points to be used
    @staticmethod
    def steering_angle_factor(
            curr_point, # let this be T+1
            next_point, # T+2
            next_to_next_point,# T+3 
            heading, 
            steering
        ):
        # W.r.t heading
        target_angle = SteeringUtils.calc_angle(curr_point, next_point)
        next_target_angle = SteeringUtils.calc_angle(curr_point, next_to_next_point)

        heading = SteeringUtils.normalize(heading)

        # This shoudl incorporate whethr im moving in 
        # right so it shoud be -ve
        # left if i need to move +ve
        diff_bw_heading_and_target = target_angle - heading
        diff_bw_heading_and_next_target = next_target_angle - heading
            
        if SteeringUtils.is_a_turn(curr_point, next_point, next_to_next_point):
            # If it is truing to go in opposite direction, penalize
            if SteeringUtils.is_right_turn(curr_point, next_point, next_to_next_point) and steering > 0:
                return 1e-6
            
            if not SteeringUtils.is_right_turn(curr_point, next_point, next_to_next_point) and steering < 0:
                return 1e-6
        
        # If heading is close to target, steer should be close to next angle target
        if abs(diff_bw_heading_and_target) <= 4:
            return math.exp(-0.5 * min(SteeringUtils.MAX_TURN, abs(
                steering - 
                diff_bw_heading_and_next_target
            ) ) / SteeringUtils.MAX_TURN )

        # if heading is not close to target, heading + steer should move towards current angle
        return math.exp(-0.5 * min(SteeringUtils.MAX_TURN, abs(
            steering -
            diff_bw_heading_and_target
        )) / SteeringUtils.MAX_TURN )    

    @staticmethod
    def test_steering_opposite_turns():
        assert SteeringUtils.steering_angle_factor((0, 0), (1, 1), (1, 1.2), 45, -20) == 1e-6
        assert SteeringUtils.steering_angle_factor((0, 0), (1, 1), (1, 0.8), 45, 10) == 1e-6
        assert SteeringUtils.steering_angle_factor((0, 0), (1, -1), (1, -0.8), 45, -10) == 1e-6
        assert SteeringUtils.steering_angle_factor((0, 0), (1, -1), (1, -1.2), 45, 10) == 1e-6

    @staticmethod
    def test_smooth_roads():
        # When the line is straight or the current curve is less than 5 degrees, focus on the next curve
        assert SteeringUtils.steering_angle_factor((0, 0), (1, 1), (2, 2), 45, 2) > SteeringUtils.steering_angle_factor((0, 0), (1, 1), (2, 2), 45, 5)
        # 3 degrees is the required steer to be smooth (operator)
        assert SteeringUtils.steering_angle_factor((0, 0), (1, 1), (1, 1.1), 45, 3) > SteeringUtils.steering_angle_factor((0, 0), (1, 1), (1, 1.1), 45, 1)

        # because the curve should be towards -2 and not 2
        assert SteeringUtils.steering_angle_factor((0, 0), (1, -1), (1, -1.1), -45, -2) > SteeringUtils.steering_angle_factor((0, 0), (1, -1), (1, -1.1), -45, 2)

        assert abs( SteeringUtils.steering_angle_factor((0, 0), (-1, -0.5), (-1, -0.6), -156, 6) - 0.97 ) < 0.1

        # When we curve in the different direction
        assert SteeringUtils.steering_angle_factor((0, 0), (-1, -0.5), (-1, -0.6), -156, -2) == 1e-6
        assert SteeringUtils.steering_angle_factor((0, 0), (-1, -0.5), (-1, -0.4), -156, 2) == 1e-6

    @staticmethod
    def test_turning():
        assert SteeringUtils.steering_angle_factor((0,0), (1, 1), (1, 2), 10, 20) > SteeringUtils.steering_angle_factor((0,0), (1, 1), (1, 2), 10, 10)
        # already over steered. Reward lesser steer
        assert SteeringUtils.steering_angle_factor((0, 0), (-1, -0.5), (-1, -0.6), -146, 0) > SteeringUtils.steering_angle_factor((0, 0), (-1, -0.5), (-1, -0.6), -146, 4)

    @staticmethod
    def run_tests():
        SteeringUtils.test_is_right_turn()
        SteeringUtils.test_calc_angle()
        SteeringUtils.test_smooth_roads()
        SteeringUtils.test_steering_opposite_turns()
        SteeringUtils.test_turning()