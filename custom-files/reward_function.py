import math

def angle_between_lines(x1, y1, x2, y2, x3, y3, x4, y4):
    dx1 = x2 - x1
    dy1 = y2 - y1
    dx2 = x4 - x3
    dy2 = y4 - y3
    angle = math.atan2(dy2, dx2) - math.atan2(dy1, dx1)
    return math.degrees(angle)
def reward_function(params):
    if params['is_offtrack'] or params['is_crashed']:
        return 1e-9
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    # Calculate the direction of the center line based on the closest waypoints
    waypoints_length= len(waypoints)
    next_point_1 = waypoints[int(closest_waypoints[1])]
    next_point_2 = waypoints[int((closest_waypoints[1]+1)%waypoints_length)]
    next_point_3 = waypoints[int((closest_waypoints[1]+2+waypoints_length)%waypoints_length)]
    next_point_4 = waypoints[int((closest_waypoints[1]+3+waypoints_length)%waypoints_length)]
    prev_point = waypoints[int(closest_waypoints[0])]
    prev_point_2 = waypoints[int((closest_waypoints[0]-1+waypoints_length)%waypoints_length)]

    # Penalize the reward if the difference is too large
    angle_f= angle_between_lines(next_point_1[0],next_point_1[1],next_point_2[0],next_point_2[1],next_point_3[0],next_point_3[1],next_point_4[0],next_point_4[1])
    angle_b= angle_between_lines(prev_point_2[0],prev_point_2[1],prev_point[0],prev_point[1],next_point_1[0],next_point_1[1],next_point_2[0],next_point_2[1])
    reward = 1e-9
    total_angle = (angle_f+angle_b)/2
    if total_angle >90:
        total_angle-=180
    elif total_angle <-90:
        total_angle+=180
    if all(prev_point_2 == prev_point) or all(prev_point==next_point_1) or all(next_point_1==next_point_2) or all(next_point_2==next_point_3) or all(next_point_3==next_point_4):
        total_angle =0
    if abs(total_angle)<=8:
        total_angle= 0
    total_angle *=0.75
    steering_reward =  1000*(1/(1+abs(params['steering_angle']-total_angle)))
    if abs(total_angle) >30 and abs(params['steering_angle'])>25 and total_angle*params['steering_angle']>=0:
        steering_reward=1000
    if params['steps'] > 0:
        progress_reward = 500*((10*params['progress'])/(params['steps']))**2
    else:
        return 1e-9
    reward=reward+ steering_reward+progress_reward

    opt_speed= 5*math.tanh(8/(1+abs(total_angle)))
    opt_speed=max(1.2,opt_speed)
    reward+=5*(5-abs(params['speed']-opt_speed))**2
    if abs(total_angle)<=12 and abs(params['steering_angle'])>=25:
        reward*=0.25
    return float(reward)