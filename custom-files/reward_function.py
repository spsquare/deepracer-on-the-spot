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
    angle1 = abs(math.degrees(math.atan2(next_point_2[1]-next_point_1[1],next_point_2[0]-next_point_1[0])))
    angle2 = abs(math.degrees(math.atan2(next_point_4[1]-next_point_3[1],next_point_4[0]-next_point_3[0])))
    reward = 1e-9
    angle = angle2 - angle1
    steering_reward = 64/(1+abs(2*params['steering_angle']-angle))
    if abs(angle) <=2:
        angle = 0
    if abs(angle) >30:
        optimal_speed=1.5
    else:
        optimal_speed = 12*(1/(3+10*abs(math.sin(abs(angle)))))
    if params['steps'] > 0:
        progress_reward =(params['progress'])/(params['steps']*20)
        speed_penalty = (4 - abs(params['speed']- optimal_speed))
        reward += speed_penalty**2
        reward += progress_reward
    else:
        return 1e-9
    reward=reward+steering_reward
    return float(reward)