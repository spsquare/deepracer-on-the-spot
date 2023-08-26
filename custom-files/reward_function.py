import math

def steering_reward(reward, steering, is_left_of_center, speed):
    if (steering < -25 and is_left_of_center == False) or (steering > 25 and is_left_of_center == True):
        reward *= 1.2
    else:
        reward *= 0.8

    if abs(steering) < 15 and speed < 4:
        reward *= 1.1
    else:
        reward *= 0.9

    if abs(steering) < 1 and speed > 4:
        reward *= 1.4
    
    return reward

def reward_function(params):
    all_wheels_on_track = params['all_wheels_on_track']         
    distance_from_center = params['distance_from_center']   
    progress = params['progress']                           
    steps = params['steps']                                 
    speed = params['speed']                                 
    steering_angle = abs(params['steering_angle'])              
    track_width = params['track_width']
    is_left_of_center = params['is_left_of_center']

    reward = 1e-3
    
    marker_0 = 0.05 * track_width
    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.5 * track_width

    if (all_wheels_on_track and (steps > 0)):
        reward = ((progress / steps) * 100) + (speed ** 2)
        if (distance_from_center >= 0.0 and distance_from_center <= marker_0):
            reward += 7.5
        elif (distance_from_center <= marker_1):
            reward += 4.0
        elif (distance_from_center <= marker_2):
            reward += 2.5
        elif (distance_from_center <= marker_3):
            reward += 0.1
    else:
        reward -= 0.01
        
    reward = steering_reward(reward, steering_angle, is_left_of_center, speed)

    # ABS_STEERING_THRESHOLD = 22
    # if steering_angle > ABS_STEERING_THRESHOLD:
    #     reward *= 0.8
    
    return float(reward)