# MUDR21-MODEL-4
import math
# Going fast parameters
FUTURE_STEP = 7
TURN_THRESHOLD_ANGLE = 12    
SPEED_THRESHOLD_SLOW = 1.2  
SPEED_THRESHOLD_FAST_1 = 1.5  
SPEED_THRESHOLD_FAST_2 = 2
SPEED_THRESHOLD_FAST_3 = 2.5
FUTURE_STEP_STRAIGHT = 8
TURN_THRESHOLD_STRAIGHT_ANGLE = 20    
STEERING_THRESHOLD = 8 
STRAIGHT_LINE_OPTIMAL_SPEED = 3
STEEP_TURNING_OPTIMAL_SPEED = 1.8
ACUTE_TURNING_OPTIMAL_SPEED = 1.8
MEDIUM_TURNING_OPTIMAL_SPEED = 2.6
LESS_TURNING_OPTIMAL_SPEED = 2.8
STEEP_TURNING_ANGLE_THRESHOLD = 75
ACUTE_TURNING_ANGLE_THRESHOLD = 75
MEDIUM_TURNING_ANGLE_THRESHOLD = 45
LESS_TURNING_ANGLE_THRESHOLD = 25
previous_speed = 0
previous_progress=0
previous_progress_gain = 0
WAYPOINTS_BEFORE=2
WAYPOINTS_AFTER=3
TOTAL_NUM_STEPS=230
def reward_function(params):
	# Read input parameters
	track_width = params['track_width']
	distance_from_center = params['distance_from_center']
	all_wheels_on_track = params['all_wheels_on_track']
	is_offtrack = params['is_offtrack']
	speed = params['speed']
	steering_angle = params['steering_angle']
	abs_steering_angle = abs(params['steering_angle'])
	is_left_of_center = params['is_left_of_center']
	waypoints = params['waypoints']
	closest_waypoints = params['closest_waypoints']
	heading = params['heading']
	progress = params['progress']
	steps = params['steps']
	track_length = params['track_length']
	x = params["x"]
	y = params["y"]
	car_position = [x,y]
	track_direction = get_track_direction(waypoints,closest_waypoints)
	future_track_direction = get_future_track_direction(waypoints,closest_waypoints,FUTURE_STEP)
	#direction_diff = get_direction_diff(track_direction,heading)
	global previous_progress
	global previous_progress_gain
	global previous_speed
	#Calculate 5 markers that are at varying distances away from the center line
	marker_1 = 0.1 * track_width
	marker_2 = 0.15 * track_width
	marker_3 = 0.25 * track_width
	marker_4 = 0.4 * track_width
	marker_5 = 0.5 * track_width
	
	#print("progress", progress)
	#print("steps", steps)
	#print("speed", speed)
	
	#normalised_direction_diff = float(direction_diff/180)
	normalised_distance_from_center = float(distance_from_center/track_width)/2
	normalised_steering_angle = float(abs(steering_angle)/180)
	normalised_speed = float(speed/4.0)

	shortest_line_length, shortest_line_direction = get_shortest_straight_line_length_and_direction(waypoints,closest_waypoints)
	distance_from_shortest_line = get_distance_from_shortest_straight_line(waypoints,closest_waypoints,car_position)
	direction_diff = get_direction_diff(shortest_line_direction,heading)
	#track_direction_diff = get_direction_diff(track_direction,future_track_direction)
	track_direction_diff = get_direction_diff(shortest_line_direction,track_direction)
	normalised_direction_diff = (direction_diff/180)
	normalised_distance = (distance_from_shortest_line/track_width)/2
	is_road_straight = is_straight_road_ahead(track_direction,future_track_direction)
	progress_gain = progress - previous_progress
	print("shortest line direction:" + str(shortest_line_direction) + " future direction:" + str(future_track_direction))
	print(" direction diff with future is:",track_direction_diff)
	print("heading and direction diff is:",direction_diff)
	print("distance from shortest line:", distance_from_shortest_line)
	print("marker_3 is:", marker_3)
	#print("distaince is: " + str(distance_from_shortest_line) + " and normalised_distance is " + str(normalised_distance))
	reward= 1.0
	optimal_speed_reward = 0
	progress_gain_reward = 0
	faster_speed_reward = 0
	
	#reward = reward + (1 - normalised_direction_diff)
	
	if distance_from_shortest_line>marker_1:
		print("car moving away/distance penalty applied")
		reward = 0 - normalised_distance
		
	#if speed > previous_speed:
	#        faster_speed_reward = 0
			
	#reward = reward + progress/steps
			
	if is_road_straight and distance_from_shortest_line<=marker_1 and steering_angle<=STEERING_THRESHOLD:
		optimal_speed_reward = get_speed_reward(speed,STRAIGHT_LINE_OPTIMAL_SPEED)
		if speed >= 0.7*STRAIGHT_LINE_OPTIMAL_SPEED and speed <= 1.1 * STRAIGHT_LINE_OPTIMAL_SPEED:
			print("giving optimal_speed_reward")
			optimal_speed_reward = optimal_speed_reward + speed
		print("on straight road reward:",  optimal_speed_reward)
		#reward = reward + optimal_speed_reward+faster_speed_reward
	elif not is_road_straight and track_direction_diff<LESS_TURNING_ANGLE_THRESHOLD and distance_from_shortest_line<=marker_1:
		optimal_speed_reward = get_speed_reward(speed,LESS_TURNING_OPTIMAL_SPEED)
		if speed >= 0.7*LESS_TURNING_OPTIMAL_SPEED and speed <= 1.1 * LESS_TURNING_OPTIMAL_SPEED:
			print("giving optimal_speed_reward")
			optimal_speed_reward = optimal_speed_reward + speed
		#reward = reward + optimal_speed_reward+faster_speed_reward
		print("less turning reward:", optimal_speed_reward)
	elif not is_road_straight and track_direction_diff<MEDIUM_TURNING_ANGLE_THRESHOLD and distance_from_shortest_line<=marker_1:
		optimal_speed_reward = get_speed_reward(speed,MEDIUM_TURNING_OPTIMAL_SPEED)
		if speed >= 0.7*MEDIUM_TURNING_OPTIMAL_SPEED and speed <= 1.1 * MEDIUM_TURNING_OPTIMAL_SPEED:
			print("giving optimal_speed_reward")
			optimal_speed_reward = optimal_speed_reward + speed
		#reward = reward + optimal_speed_reward * (1-normalised_direction_diff)+faster_speed_reward
		print("medium turning reward:", optimal_speed_reward)
	elif not is_road_straight and track_direction_diff<STEEP_TURNING_ANGLE_THRESHOLD and distance_from_shortest_line<=marker_1:
		optimal_speed_reward = get_speed_reward(speed,STEEP_TURNING_OPTIMAL_SPEED)
		if speed >= 0.7*STEEP_TURNING_OPTIMAL_SPEED and speed <= 1.1 * STEEP_TURNING_OPTIMAL_SPEED:
			print("giving optimal_speed_reward")
			optimal_speed_reward = optimal_speed_reward + speed
		#reward = reward + optimal_speed_reward * (1-(normalised_direction_diff*2))
		print("steep turning reward:", optimal_speed_reward)
	elif not is_road_straight and track_direction_diff>ACUTE_TURNING_ANGLE_THRESHOLD and distance_from_shortest_line<=marker_2:
		optimal_speed_reward = get_speed_reward(speed,ACUTE_TURNING_OPTIMAL_SPEED)
		#reward = reward + optimal_speed_reward * (1-(normalised_direction_diff*2))
		if speed >= 0.7*ACUTE_TURNING_OPTIMAL_SPEED and speed <= 1.1 * ACUTE_TURNING_OPTIMAL_SPEED:
			print("giving optimal_speed_reward")
			optimal_speed_reward = optimal_speed_reward + speed
		print("steep turning reward:", optimal_speed_reward)

	reward = reward + optimal_speed_reward * (1-normalised_direction_diff)**2
	
	if (steps % 10) == 0 and progress > (steps / TOTAL_NUM_STEPS) * 100 :
		steps_reward = 2 * (1 + (progress/100) - (steps / TOTAL_NUM_STEPS))
	else:
		steps_reward = 0
		
	reward += steps_reward

	progress_reward=0
	if progress == 100 :
		progress_reward += max(0.2, 1+((TOTAL_NUM_STEPS-steps)/TOTAL_NUM_STEPS))

	reward += progress_reward
	
	print(f"steps_reward: {steps_reward} {progress_reward}")
	

	previous_progress_gain = progress_gain
	previous_progress = progress
	previous_speed = speed


	if is_offtrack:
		reward = 0.001
	
	return float(reward)
	
def get_speed_reward(speed, optimal_speed):
	speed_deviation_ratio = abs(optimal_speed-speed)/optimal_speed
	speed_deviation_penalty = speed_deviation_ratio*speed
	speed_reward = (speed - speed_deviation_penalty)/4.0
	#enhanced_speed_reward =  speed_reward**3 if speed_reward>=3 else speed_reward**2 if speed_reward>=2 else speed_reward
	return speed_reward

def is_car_turning_to_right_or_left(steering_angle):
	if steering_angle < 0:
		direction = "right"
	else:
		direction = "left"
	
	return direction
	
def is_car_turning_right(steering_angle):
	return (is_car_turning_to_right_or_left(steering_angle)=="right")
		
def is_car_turning_left(steering_angle):
	return (is_car_turning_to_right_or_left(steering_angle)=="left")
		
def is_car_heading_to_right_of_track(heading,track_direction):

	if heading > 0 and track_direction > 0:
		if track_direction > heading:
			return True
		else:
			return False
	elif heading > 0 and track_direction < 0 and track_direction < -90 :
		return True
	elif heading > 0 and track_direction < 0 and track_direction > -90 :
		return False
	elif heading < 0 and track_direction > 0 and heading > -90 :
		return True
	elif heading < 0 and track_direction > 0 and heading < -90 :
		return False
	elif heading < 0 and track_direction < 0:
		if track_direction > heading:
			return True
		else:
			return False
			
def is_car_moving_away_from_track_direction(steering_angle,heading,track_direction):

	if is_car_heading_to_right_of_track(heading,track_direction) and is_car_turning_right(steering_angle):
		return True
	elif not is_car_heading_to_right_of_track(heading,track_direction) and is_car_turning_left(steering_angle):
		return True
	else:
		return False
	
def get_track_direction(waypoints,closest_waypoints):
	# Calculate the direction of the center line based on the closest waypoints
	next_point = waypoints[closest_waypoints[1]]
	prev_point = waypoints[closest_waypoints[0]]
	
	# Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
	track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
	# Convert to degree
	track_direction = math.degrees(track_direction)
	return track_direction
	
def get_direction_diff(direction_1,direction_2):
	# Calculate the difference between 2 directions
	direction_diff = abs(direction_1 - direction_2)
	if direction_diff > 180:
		direction_diff = 360 - direction_diff
	return direction_diff
	
def get_direction_between_two_waypoints(fist_waypoint, second_waypoint):
	# Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
	track_direction = math.atan2(second_waypoint[1] - fist_waypoint[1], second_waypoint[0] - fist_waypoint[0])
	# Convert to degree
	track_direction = math.degrees(track_direction)
	return track_direction

def get_future_track_direction(waypoints, closest_waypoints, FUTURE_STEP):       
		point_prev = waypoints[closest_waypoints[0]]
		point_next = waypoints[closest_waypoints[1]]
		point_future = waypoints[min(len(waypoints) - 1,
									 closest_waypoints[1] + FUTURE_STEP)]

		track_direction_future = math.degrees(math.atan2(point_future[1] - point_next[1], 
											   point_future[0] - point_next[0]))

		return track_direction_future

def is_road_turning(waypoints, closest_waypoints,FUTURE_STEP):

	current_track_direction = get_track_direction(waypoints,closest_waypoints)
	future_trck_direction = get_future_track_direction(waypoints,closest_waypoints,FUTURE_STEP)

	diff_track_direction = get_direction_diff(future_trck_direction,current_track_direction)

	return diff_track_direction>TURN_THRESHOLD_ANGLE

def get_road_turning_direction(waypoints, closest_waypoints,FUTURE_STEP):

	current_track_direction = get_track_direction(waypoints,closest_waypoints)
	future_trck_direction = get_future_track_direction(waypoints,closest_waypoints,FUTURE_STEP)
	direction = "straight"

	if is_road_turning (waypoints,closest_waypoints,FUTURE_STEP):
		if future_trck_direction > 0 and current_track_direction > 0:
			if future_trck_direction > current_track_direction:
				direction = "left"
			else:
				direction = "right"
		elif future_trck_direction > 0 and current_track_direction < 0 and current_track_direction < -90 :
			direction = "right"
		elif future_trck_direction > 0 and current_track_direction < 0 and current_track_direction > -90 :
			direction = "left"
		elif future_trck_direction < 0 and current_track_direction > 0 and future_trck_direction > -90 :
			direction = "right"
		elif future_trck_direction < 0 and current_track_direction > 0 and future_trck_direction < -90 :
			direction = "left"
		elif future_trck_direction < 0 and current_track_direction < 0:
			if future_trck_direction > current_track_direction:
				direction = "left"
			else:
				direction =  "right" 

	return direction

def is_straight_road_ahead(direction, future_direction):
	direction_diff = get_direction_diff(future_direction,direction)
	#print("current dir: " + str(direction) + "future dir:" + str(future_direction))
	return direction_diff < TURN_THRESHOLD_ANGLE
	



def get_shortest_straight_line_length_and_direction(waypoints,closest_waypoints):
	
	waypoints_n_units_back, waypoints_n_units_forward = get_waypoint_n_units_further(waypoints,closest_waypoints,WAYPOINTS_BEFORE,WAYPOINTS_AFTER)
	length = abs(math.sqrt((waypoints_n_units_forward[0]-waypoints_n_units_back[0])**2 + (waypoints_n_units_forward[1]-waypoints_n_units_back[1])**2))
	direction = get_direction_between_two_waypoints(waypoints_n_units_back,waypoints_n_units_forward)
	return length, direction

def get_waypoint_n_units_further(waypoints,closest_waypoints,WAYPOINTS_BEFORE,WAYPOINTS_AFTER):
	prev_waypoint = closest_waypoints[0]
	next_waypoint = closest_waypoints[1]
	#print("first waypoint is ", first_waypoint)
	waypoints_n_units_back_index = get_waypoint_index_n_units_back(prev_waypoint,WAYPOINTS_BEFORE,waypoints)
	waypoint_n_units_forward_index = get_waypoint_index_n_units_ahead(prev_waypoint,WAYPOINTS_AFTER,waypoints)
	waypoints_n_units_back = waypoints[waypoints_n_units_back_index]
	waypoints_n_units_forward = waypoints[waypoint_n_units_forward_index]
	print("back waypoint:" + str(waypoints_n_units_back_index) + " forward wapoint:" + str(waypoint_n_units_forward_index))
	return waypoints_n_units_back, waypoints_n_units_forward

def get_waypoint_index_n_units_ahead(current_waypoint_index,n_units,waypoints):
	#print("first waypoint is ", first_waypoint)
	waypoint_n_units_forward_index = current_waypoint_index + n_units
	if waypoint_n_units_forward_index > len(waypoints)-1:
		waypoint_n_units_forward_index = waypoint_n_units_forward_index - len(waypoints)
	return waypoint_n_units_forward_index

def get_waypoint_index_n_units_back(current_waypoint_index,n_units,waypoints):
	#print("first waypoint is ", first_waypoint)
	waypoints_n_units_back_index = current_waypoint_index - n_units
	if waypoints_n_units_back_index < 0:
		waypoints_n_units_back_index = len(waypoints) + waypoints_n_units_back_index
	return waypoints_n_units_back_index

def get_distance_from_shortest_straight_line(waypoints,closest_waypoints, car_position):
	waypoint_n_units_back, waypoint_n_units_forward = get_waypoint_n_units_further(waypoints,closest_waypoints,WAYPOINTS_BEFORE,WAYPOINTS_AFTER)
	x1 = waypoint_n_units_back[0]
	y1 = waypoint_n_units_back[1]
	#print("first waypoint is ", first_waypoint)
	#print("last waypoint is ", last_waypoint)
	x2 = waypoint_n_units_forward[0]
	y2 = waypoint_n_units_forward[1]
	# co-ordinates from car
	x = car_position[0]
	y = car_position[1]
	distance = abs((x2-x1)*(y1-y) - (x1-x)*(y2-y1)) / abs(math.sqrt((x2-x1)**2 + (y2-y1)**2))
	return distance

def get_test_params(heading,speed,x,y):
	return {
	"all_wheels_on_track": True,
	"x": x,
	"y": y,
	"distance_from_center": .0001,
	"is_left_of_center": True,
	"is_reversed": False,
	"is_offtrack": False,
	"heading": heading,
	"progress": 1.0,
	"is_crashed": False,
	"steps": 1,
	"speed": speed,
	"steering_angle": 1.0,
	"track_length": 24.05,
	"track_width": 0.607175005164503,
	"waypoints": [(2.909995283569139, 0.6831924746239328), (3.3199952311658905, 0.6833390533713652),
				  (3.41999521838461, 0.6833748042853732), (3.6300023417267235, 0.6834498837610459),
				  (4.189995119968753, 0.6836500863232341), (4.500002230529587, 0.6837609167129147),
				  (4.549995073956144, 0.6837787896136626), (5.320002125723089, 0.6840540742077795),
				  (5.420002112941809, 0.6840898251217875), (5.7800020669292005, 0.684218528412216),
				  (6.289747858140073, 0.6921400142174), (6.460906484698166, 0.7123063542781353),
				  (6.5136980596947165, 0.7210294115664316), (6.704287871536597, 0.799598672280553),
				  (6.836281775656231, 0.8817004790362547), (6.991663362669656, 1.0062653214908401),
				  (7.1142074641408275, 1.1693225137564909), (7.165830682349035, 1.263426756737598),
				  (7.280019741788613, 1.7628308313393968), (7.272892208655982, 1.8132370038722583),
				  (7.265960701310593, 1.8622568749360433), (7.1045747673751585, 2.3014874894475916),
				  (7.011749008840918, 2.419260292916218), (6.727273712845888, 2.6474924751765463),
				  (6.536921216759571, 2.7266447610626687), (6.079802178702642, 2.773360773339069),
				  (5.919813651266964, 2.772005974951175), (5.719827991972368, 2.7703124769663074),
				  (5.670000926947205, 2.7698905365406308), (5.200034627604903, 2.765910816276192),
				  (5.049876033335467, 2.7646392587170006), (5.002030872389276, 2.768980714618128),
				  (4.942709994269048, 2.775327848322301), (4.561340171137485, 2.898322513024676),
				  (4.258533108743229, 3.166955220685885), (4.092728535429521, 3.3703748558215287),
				  (4.001121969780925, 3.482763638518189), (3.774000078716213, 3.761411273431655),
				  (3.6823935130676184, 3.8738000561283137), (3.5490587458571623, 4.037383660336441),
				  (3.2758532950668884, 4.333295323360169), (3.1911463583891155, 4.385684825652305),
				  (3.0954945192403103, 4.435922305057415), (2.9549738926202442, 4.484413606024224),
				  (2.8089822299540046, 4.500038654567632), (2.8110045575773057, 4.499832029419236),
				  (2.5003276964136627, 4.498718163592657), (2.249377566090162, 4.491428972830993),
				  (1.990177178741659, 4.483900142037221), (1.7395172672798365, 4.476619381080485),
				  (1.1871156114665855, 4.391792930201858), (1.1054389398706574, 4.3402307341807065),
				  (0.7316196323127645, 3.819658838269335), (0.7080468873794841, 3.5295953182618844),
				  (0.8747319412102282, 2.7251244177375193), (0.8863119620897287, 2.6692358445815714),
				  (0.9180990438541362, 2.5158220758940644), (0.9380374746317692, 2.4195933679559642),
				  (1.0212099341560652, 2.0181787127447155), (1.043063552869095, 1.912706746772055),
				  (1.0936256517149223, 1.6686792454688633), (1.219724413480236, 1.169889412099395),
				  (1.2404620134668318, 1.1182110370035536), (1.286611404297767, 1.0270193376917442),
				  (1.3195344250237366, 0.9895904728963364), (1.3897426105955222, 0.9097735962139227),
				  (1.4563853812178036, 0.8435308547287804), (1.4996428710531535, 0.8193608401945228),
				  (2.0400025449490777, 0.6828814442283201), (2.7500024542019887, 0.6831352757177762),
				  (2.909995283569139, 0.6831924746239328)],
	"closest_waypoints": [3,4]
}

from reward_function import reward_function as rf

def test_rewards():
	heading = [0.45,5.24, .002, 10.24, 15.4678]
	speed = [1.24, 2.48, 2.0, 1.3, 3,4]
	x = [2.1345, 2.4567, 3,4325, 3.5678, 2.7890]
	y = [0.6873,0.6739,0.7980,0.9840,0.8465]

	for i in range(0,5):
		reward = rf(get_test_params(heading[i],speed[i],x[i],y[i]))
		print("reward", reward)

test_rewards()