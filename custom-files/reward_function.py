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

    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = abs(track_direction - params['heading'])
    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    # Penalize the reward if the difference is too large
    angle_f= angle_between_lines(next_point_1[0],next_point_1[1],next_point_2[0],next_point_2[1],next_point_3[0],next_point_3[1],next_point_4[0],next_point_4[1])
    angle_f2= angle_between_lines(next_point_3[0],next_point_3[1],next_point_4[0],next_point_4[1],next_point_5[0],next_point_5[1],next_point_6[0],next_point_6[1])
    angle_b= angle_between_lines(prev_point_2[0],prev_point_2[1],prev_point[0],prev_point[1],next_point_1[0],next_point_1[1],next_point_2[0],next_point_2[1])
    reward = 1e-9
    total_angle = (angle_f+angle_b+angle_f2)/3
    if total_angle >90:
        total_angle-=180
    elif total_angle <-90:
        total_angle+=180
    if next ==1 or prev==1 or (next+1)%waypoints_length ==1 or (next+2)%waypoints_length ==1 or (next+3)%waypoints_length ==1 or (next+4)%waypoints_length ==1 or (next+5)%waypoints_length ==1 or (next+6)%waypoints_length ==1 or (next+7)%waypoints_length ==1 or (prev -1 +waypoints_length)%waypoints_length ==1:
        total_angle =0
    if abs(total_angle)<=15:
        reward = params['speed']
    elif total_angle >0 and params['is_left_of_center']:
        if abs(total_angle)>22 and params['speed']>=2:
            return 0.001
        reward = 4/(1+5*abs(params['distance_from_center']-abs(total_angle)*params['track_width']/60)/params['track_width'])
    elif total_angle<0 and not params['is_left_of_center']:
        if abs(total_angle)>22 and params['speed']>=2:
            return 0.001
        reward = 4/(1+5*abs(params['distance_from_center']-abs(total_angle)*params['track_width']/60)/params['track_width'])
    return float(reward)