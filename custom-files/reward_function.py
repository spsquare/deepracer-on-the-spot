import math

def reward_function(params):
    if params['is_offtrack'] or params['is_crashed']:
        return 1e-9
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    # Calculate the direction of the center line based on the closest waypoints
    waypoints_length= len(waypoints)
    next_point_1 = waypoints[closest_waypoints[1]]
    next_point_2 = waypoints[(closest_waypoints[1]+1)%waypoints_length]
    next_point_3 = waypoints[(closest_waypoints[1]+2+waypoints_length)%waypoints_length]
    next_point_4 = waypoints[(closest_waypoints[1]+3+waypoints_length)%waypoints_length]
    prev_point = waypoints[closest_waypoints[0]]
    prev_point_2 = waypoints[(closest_waypoints[1]-1)%waypoints_length]

    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    track_direction = math.atan2(next_point_1[1] - prev_point[1], next_point_1[0] - prev_point[0])
    # Convert to degree
    track_direction = math.degrees(track_direction)

    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = abs(track_direction - params['heading'])
    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    # Penalize the reward if the difference is too large
    angle1 = abs(math.degrees(math.atan2(next_point_2[1]-next_point_1[1],next_point_2[0]-next_point_1[0])))
    angle2 = abs(math.degrees(math.atan2(next_point_4[1]-next_point_3[1],next_point_4[0]-next_point_3[0])))
    angle0 = abs(math.degrees(math.atan2(prev_point[1]-prev_point_2[1],prev_point[0]-prev_point_2[0])))
    reward = 1e-9
    angle = angle2 - angle1
    angle_b = angle1 - angle0
    total_angle = angle- angle_b
    if abs(total_angle) > 30:
        optimal_speed=1.2
    else:
        optimal_speed = 5*math.tanh(8/(1+abs(total_angle)))
    optimal_speed = max(optimal_speed,1.2)
    steering_reward = 64/(1+abs(params['steering_angle']-total_angle)**2)
    if params['steps'] > 0:
        progress_reward =(params['progress'])/(params['steps']*10)
        speed_penalty = (5 - abs(params['speed']- optimal_speed))
        reward += speed_penalty**2
        reward += progress_reward
        reward += params['progress']/50
    else:
        return 1e-9
    reward=reward+steering_reward
    DIRECTION_THRESHOLD = 10.0
    if direction_diff <= DIRECTION_THRESHOLD:
        reward += 8.0
#    if not params['all_wheels_on_track']:
#        if params['is_left_of_center'] and params['steering_angle'] >0:
#            reward*=0.1
#        if not params['is_left_of_center'] and params['steering_angle'] <0:
#            reward*=0.1
    return float(reward)