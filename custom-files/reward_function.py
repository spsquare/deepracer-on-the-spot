import math

def angle_between_lines(x1, y1, x2, y2, x3, y3, x4, y4):
    dx1 = x2 - x1
    dy1 = y2 - y1
    dx2 = x4 - x3
    dy2 = y4 - y3
    angle = math.atan2(dy2, dx2) - math.atan2(dy1, dx1)
    deg= math.degrees(angle)
    if deg>180:
        deg=deg-360
    if deg <-180:
        deg= deg+360
    return deg
def reward_function(params):
    if params['is_offtrack'] or params['is_crashed']:
        return 1e-9
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    # Calculate the direction of the center line based on the closest waypoints
    waypoints_length= len(waypoints)
    steering_angle=params['steering_angle']
    prev = int(closest_waypoints[0])
    next = int(closest_waypoints[1])
    next_point_1 = waypoints[next]
    next_point_2 = waypoints[(next+1)%waypoints_length]
    next_point_3 = waypoints[(next+2)%waypoints_length]
    next_point_4 = waypoints[(next+3)%waypoints_length]
    next_point_5 = waypoints[(next+4)%waypoints_length]
    next_point_6 = waypoints[(next+5)%waypoints_length]
    prev_point = waypoints[prev]
    prev_point_2 = waypoints[(prev-1+waypoints_length)%waypoints_length]

    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    track_direction = math.atan2(next_point_1[1] - prev_point[1], next_point_1[0] - prev_point[0])
    # Convert to degree
    track_direction = math.degrees(track_direction)
    straight_points = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,158,159,160,161,162,163,164,165,166,167,168,169,170,171,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,224,225,181,182,183,184]
    easy_curve_left = [20,21,22,23,24,25,44,45,46,47,48,49,50,67,68,69,70,71,72,73,74,75,76,77,78,79,87,88,89,90,91,92,93,153,154,155,172,173,174,175,176,189,190,191,192,193]
    easy_curve_right = [56,57,58,59,60,61,62,199,200,201,202]
    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = abs(track_direction - params['heading'])
    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    # Penalize the reward if the difference is too large
    angle_f= angle_between_lines(next_point_1[0],next_point_1[1],next_point_2[0],next_point_2[1],next_point_3[0],next_point_3[1],next_point_4[0],next_point_4[1])
    angle_f2= angle_between_lines(next_point_3[0],next_point_3[1],next_point_4[0],next_point_4[1],next_point_5[0],next_point_5[1],next_point_6[0],next_point_6[1])
    #angle_f3= angle_between_lines(next_point_5[0],next_point_5[1],next_point_6[0],next_point_6[1],next_point_7[0],next_point_7[1],next_point_8[0],next_point_8[1])
    angle_b= angle_between_lines(prev_point_2[0],prev_point_2[1],prev_point[0],prev_point[1],next_point_1[0],next_point_1[1],next_point_2[0],next_point_2[1])
    reward = 1e-9
    #total_angle = (angle_f+angle_b+angle_f2+angle_f3)/4
    total_angle = (angle_f+angle_b+angle_f2)/3
    if total_angle >90:
        total_angle-=180
    elif total_angle <-90:
        total_angle+=180
    if total_angle >30:
        total_angle =30
    if total_angle <-30:
        total_angle=-30
    if next ==1 or prev==1 or (next+1)%waypoints_length ==1 or (next+2)%waypoints_length ==1 or (next+3)%waypoints_length ==1 or (next+4)%waypoints_length ==1 or (next+5)%waypoints_length ==1 or (next+6)%waypoints_length ==1 or (next+7)%waypoints_length ==1 or (prev -1 +waypoints_length)%waypoints_length ==1:
        total_angle =0
    if abs(total_angle)<=5:
        if abs(steering_angle)>=3:
            return 1e-3
    steering_reward = 100/(1+abs(params['steering_angle']-total_angle))
    if params['steps'] > 0:
        progress_reward =(params['progress'])/(params['steps'])+ params['progress']//2
        reward += progress_reward
    else:
        return 1e-9
    reward=reward+ steering_reward
    if direction_diff <=10.0:
        reward+=10.0
    opt_speed= 5*math.tanh(8/(1+abs(total_angle)))
    opt_speed=max(1.2,opt_speed)
    reward+=(5-abs(params['speed']-opt_speed))**2
    if abs(params['steering_angle'])<10 and abs(total_angle)>20:
        return 1e-3
    if abs(params['steering_angle'])>=25 and abs(total_angle)>=25 and total_angle*params['steering_angle']>=0:
        reward+=100.0
    if (total_angle <=22 and total_angle>5) and params['is_left_of_center']:
        reward+=1000
    if (total_angle >=-22 and total_angle<-5) and not params['is_left_of_center']:
        reward+=1000    
    return float(reward)