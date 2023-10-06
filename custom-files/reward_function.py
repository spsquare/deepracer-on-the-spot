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
    waypoints_length= len(waypoints)
    next_point_1 = waypoints[closest_waypoints[1]]
    next_point_2 = waypoints[(closest_waypoints[1]+1)%waypoints_length]
    next_point_3 = waypoints[(closest_waypoints[1]+2+waypoints_length)%waypoints_length]
    next_point_4 = waypoints[(closest_waypoints[1]+3+waypoints_length)%waypoints_length]
    prev_point = waypoints[closest_waypoints[0]]
    prev_point_2 = waypoints[(closest_waypoints[0]-1+waypoints_length)%waypoints_length]

    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    track_direction = math.atan2(next_point_1[1] - prev_point[1], next_point_1[0] - prev_point[0])
    # Convert to degree
    track_direction = math.degrees(track_direction)

    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = abs(track_direction - params['heading'])
    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    # Penalize the reward if the difference is too large
    angle_f= angle_between_lines(next_point_1[0],next_point_1[1],next_point_2[0],next_point_2[1],next_point_3[0],next_point_3[1],next_point_4[0],next_point_4[1])
    angle_b= angle_between_lines(prev_point_2[0],prev_point_2[1],prev_point[0],prev_point[1],next_point_1[0],next_point_1[1],next_point_2[0],next_point_2[1])
    reward = 1e-9
    total_angle = (angle_f+angle_b)/2
    if total_angle >90:
        total_angle-=180
    elif total_angle <-90:
        total_angle+=180
    if abs(total_angle)<=5:
        total_angle=0
    total_angle =total_angle*0.75
    steering_reward = 100/(1+abs(params['steering_angle']-total_angle))
    if params['steps'] > 0:
        progress_reward =(1.2*params['progress'])/(params['steps'])+ params['progress']//4
        reward += progress_reward
    else:
        return 1e-9
    reward=reward+ steering_reward
    if direction_diff <=10.0:
        reward+=10.0
    if abs(total_angle)<=5:
        if params['speed'] >=3:
            reward+=30
        if params['speed'] >=3.4:
            reward+=30
        if params['speed'] >=3.8:
            reward+=30
        if params['speed'] >=4:
            reward+=30
        if params['speed'] >=4.2:
            reward+=30
        if params['speed'] >=4.4:
            reward+=50
    else:
        opt_speed= 5*math.tanh(8/(1+abs(total_angle)))
        opt_speed=max(1.2,opt_speed)
        reward+=(5-abs(params['speed']-opt_speed))**2
    return float(reward)