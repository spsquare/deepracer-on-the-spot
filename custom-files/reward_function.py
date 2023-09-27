
import math
def reward_function(params):

    # Reward weights
    speed_weight = 100
    heading_weight = 100
    steering_weight = 50
    progress_weight = 17

    # Initialize the reward based on current speed
    max_speed_reward = 5 * 5
    min_speed_reward = 1.3 * 1.3
    abs_speed_reward = params['speed'] * params['speed']
    speed_reward = (abs_speed_reward - min_speed_reward) / (max_speed_reward - min_speed_reward) * speed_weight
    heading_reward = 0.0
    progress_reward = 0.0
    # - - - - - 
    if params['steps']>0:
        progress_reward = (params['progress']/(params['steps']*progress_weight))**2
    # Penalize if the car goes off track
    if  params['is_crashed'] or params['is_offtrack']:
        return -20.0
    
    # - - - - - 
    
    # Calculate the direction of the center line based on the closest waypoints
    next_point = params['waypoints'][params['closest_waypoints'][1]]
    prev_point = params['waypoints'][params['closest_waypoints'][0]]
    
    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0]) 
    
    # Convert to degree
    track_direction = math.degrees(track_direction)

    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = abs(track_direction - params['heading'])
    if direction_diff > 180:
        direction_diff = 360 - direction_diff
    else:
        abs_heading_reward = 1 - (direction_diff / 180.0)
        heading_reward = abs_heading_reward * heading_weight
    
    # - - - - -
    
    # Reward if steering angle is aligned with direction difference
    if params['steering_angle'] - direction_diff > 7:
        steering_reward = 1e-3
    else:
        abs_steering_reward = 1 - (abs(params['steering_angle'] - direction_diff) / 180.0)
        steering_reward = abs_steering_reward * steering_weight

    # - - - - -
    # Penalize if slow speed action space
    if params['speed'] > 2.0:
        return speed_reward*2


    
    return speed_reward + heading_reward + steering_reward + progress_reward
