import math

# def reward_function(params):
#     if params['is_offtrack'] or params['is_crashed']:
#         return 1e-9
#     reward = 1;
#     waypoints = params['waypoints']
#     closest_waypoints = params['closest_waypoints']
#     heading = params['heading']
#     steering_angle = params['steering_angle']
#     next_point = waypoints[closest_waypoints[1]]
#     prev_point = waypoints[closest_waypoints[0]]
#     track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
#     track_direction = math.degrees(track_direction)
#     direction_diff = abs(track_direction - heading)
#     if direction_diff > 180:
#         direction_diff = 360 - direction_diff
#     direction_threshold = 5
#     reward = reward * (1/(1+abs(steering_angle-direction_diff)));
#     opt_speed = 4*(18/(18+abs(steering_angle)))
#     reward = reward * ((4-abs(params['speed']-opt_speed))/4);
#     steps = params['steps'];
#     on_track = on_track_reward(params);
#     return (1+(((1+steps/10000)**4+(1+reward)**2+on_track**2)/24))**3;

# def on_track_reward(params):
#     track_width = params['track_width']/2
#     distance_from_center = params['distance_from_center']
#     reward = track_width/track_width+0.5*distance_from_center

#     return 1+float(reward)

def reward_function(params):
    all_wheels_on_track = params['all_wheels_on_track']         
    distance_from_center = params['distance_from_center']   
    progress = params['progress']                           
    steps = params['steps']                                 
    speed = params['speed']                                 
    steering_angle = abs(params['steering_angle'])              
    track_width = params['track_width']                        

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

    # speed
    
    
    ABS_STEERING_THRESHOLD = 22
    if steering_angle > ABS_STEERING_THRESHOLD:
        reward *= 0.8
    
    return float(reward)
