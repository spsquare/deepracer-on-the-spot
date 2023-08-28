import math

def reward_function(params):
    if params['is_offtrack'] or params['is_crashed']:
        return 1e-9
    waypoints = params['waypoints']
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
    expected_direction= abs(track_direction - heading)
    direction_diff = abs(track_direction - heading-params['steering_angle'])
    if direction_diff > 180:
        direction_diff = 360 - direction_diff
    if expected_direction > 180:
        expected_direction = 360 - expected_direction

    # Penalize the reward if the difference is too large
    reward = 1
    if direction_diff>45:
        return 1e-9
    steering_reward = 100/(1+expected_direction)
    if params['steps'] > 0:
        progress_reward =(params['progress'])/(params['steps']*50)
        speed_penalty = 2- (abs(params['speed']-3.2*(30/(30+abs(params['steering_angle']))))/3.2)
        reward *= speed_penalty**4
    else:
        return 1e-9
    reward=reward+steering_reward
    if not (params['is_left_of_center'] and params['distance_from_center'] >= 0.4*params['track_width']):
        reward=reward * 1.25 if params['steering_angle']>=0 else reward
    return float(reward+progress_reward)