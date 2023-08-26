import math

def reward_function(params):
    if params['is_offtrack'] or params['is_crashed']:
        return 1e-9
    shorter_turn_reward = 1.1
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    # Calculate the direction of the center line based on the closest waypoints
    next_point = waypoints[closest_waypoints[1]]
    prev_point = waypoints[closest_waypoints[0]]
    if len(waypoints) > closest_waypoints[1]+1:
        next_point_2 = waypoints[closest_waypoints[1]+1]
    else:
        next_point_2 = waypoints[0]
    angle = math.degrees(math.atan2(next_point_2[1]-next_point[1], next_point_2[0]-next_point[0]) - math.atan2(prev_point[1]-next_point[1], prev_point[0]-next_point[0]))
    angle= abs(angle)

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
    direction_penalty = max(1-abs(expected_direction / 30), 1e-9)
    if params['steps'] > 0:
        progress_reward =(params['progress'])/(params['steps']*10)
        speed_penalty = 2- (abs(params['speed']-4.0*(90/(90+angle)))/4.0)
        reward *= speed_penalty**4
        if angle < 20:
            reward*=speed_penalty**2
            if params['distance_from_centre']/params['track_width']  <0.1 and angle <10:
                reward*=0.1
        else:
            progress_reward=progress_reward*2.5

    else:
        return 1e-9
    if params['is_left_of_center']:
        reward*=shorter_turn_reward
    reward=reward*direction_penalty
    return float(reward+progress_reward)