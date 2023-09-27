import math

def angle_between_lines(x1, y1, x2, y2, x3, y3):
    dx1 = x2 - x1
    dy1 = y2 - y1
    dx2 = x3 - x2
    dy2 = y3 - y2
    angle = math.atan2(dy2, dx2) - math.atan2(dy1, dx1)
    return math.degrees(angle)
def reward_function(params):
    if params['is_offtrack'] or params['is_crashed']:
        return 1e-9
        # Reward weights
    speed_weight = 100
    heading_weight = 100
    steering_weight = 50
    progress_weight = 17
    
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    max_speed_reward = 5 * 5
    min_speed_reward = 1.3 * 1.3
    abs_speed_reward = params['speed'] * params['speed']
    speed_reward = (abs_speed_reward - min_speed_reward) / (max_speed_reward - min_speed_reward) * speed_weight
    heading_reward = 0.0
    progress_reward = 0.0
    if params['steps']>0:
        progress_reward = (params['progress']/(params['steps']*progress_weight))**2
    # Penalize if the car goes off track
    if  params['is_crashed'] or params['is_offtrack']:
        return -20.0
    # Calculate the direction of the center line based on the closest waypoints
    waypoints_length= len(waypoints)
    next_point_1 = waypoints[closest_waypoints[1]]
    next_point_2 = waypoints[(closest_waypoints[1]+1)%waypoints_length]
    prev_point_1 = waypoints[closest_waypoints[0]]
    prev_point_2 = waypoints[(closest_waypoints[0]-1+waypoints_length)%waypoints_length]

    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    track_direction = math.atan2(next_point_1[1] - prev_point_1[1], next_point_1[0] - prev_point_1[0])
    # Convert to degree
    track_direction = math.degrees(track_direction)

    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = abs(track_direction - params['heading'])
    if direction_diff > 180:
        direction_diff = 360 - direction_diff
    else:
        abs_heading_reward = 1 - (direction_diff / 180.0)
        heading_reward = abs_heading_reward * heading_weight

    # Penalize the reward if the difference is too large
    angle_f= angle_between_lines(prev_point_1[0],prev_point_1[1],next_point_1[0],next_point_1[1],next_point_2[0],next_point_2[1])
    angle_b= angle_between_lines(prev_point_2[0],prev_point_2[1],prev_point_1[0],prev_point_1[1],next_point_1[0],next_point_1[1])
    total_angle = angle_f- angle_b
    reward = 1e-9
    abs_steering_reward = 1 - (abs(params['steering_angle'] - total_angle) / 180.0)
    steering_reward = abs_steering_reward * steering_weight
    if params['steps'] > 0:
        progress_reward =(5*params['progress'])/(params['steps'])
        reward += progress_reward
        reward += steering_reward
        reward += heading_reward
        reward += speed_reward
    else:
        return 1e-9
    reward=reward+ steering_reward
    DIRECTION_THRESHOLD = 10.0
    if direction_diff <= DIRECTION_THRESHOLD:
        reward += 10.0
#    if not params['all_wheels_on_track']:
#        if params['is_left_of_center'] and params['steering_angle'] >0:
#            reward*=0.1
#        if not params['is_left_of_center'] and params['steering_angle'] <0:
#            reward*=0.1
    return float(reward)