import math;

def reward_function(params):
    if params['is_offtrack'] or params['is_crashed']:
        return 1e-9;
    progress = progress_reward(params);
    heading = params['heading'];
    steering_angle = params['steering_angle'];
    if(heading<0):
        heading = heading+360;
    waypoints = params['waypoints'];
    closest_waypoints = params['closest_waypoints']
    next_point = waypoints[closest_waypoints[1]];
    prev_point = waypoints[closest_waypoints[0]];
    track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
    track_direction = math.degrees(track_direction)
    if(track_direction<0):
        track_direction=track_direction+360;
    all_straight = True;
    all_increasing = True;
    all_decreasing = True;
    index = 5;
    headings = [];
    reward = 1.0;
    max_heading = abs(heading-track_direction);
    speed_thresh = 2;
    if(abs(track_direction-heading)>3):
        reward = reward*0.5;
    headings.append(track_direction);
    for i in range(1,5):
        next_point = waypoints[(closest_waypoints[1]+i)%len(closest_waypoints)]
        prev_point = waypoints[(closest_waypoints[1]+i-1)%len(closest_waypoints)]
        track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0]);
        track_direction = math.degrees(track_direction)
        if(track_direction<0):
            track_direction=track_direction+360;
        if(track_direction!=headings[-1]):
            if(abs(heading-track_direction)>max_heading):
                max_heading = abs(heading-track_direction);
                index = i;
            all_straight = False;
        if(track_direction>headings[-1]):
            all_decreasing = False
        if(track_direction<headings[-1]):
            all_increasing = False;
        headings.append(track_direction)
    if(all_straight):
        if(params['speed']<speed_thresh):
            reward = reward*0.2;
        else:
            reward = reward*(1+params['speed']/4)
        if not params['all_wheels_on_track']:
            reward = reward * 0.4;    
        if (abs(steering_angle)>2):
            reward = reward * 0.3;
    else:
        opt_speed = 4;
        if(index!=5):
            opt_speed = 72/(18+max_heading)
        interval = (speed_thresh-opt_speed)/4;
        if(interval<=0):
            interval = 0.1;
        if(params['speed']<opt_speed):
            reward = reward * 0.1;
        if(index==1):
            if(abs(params['speed']-opt_speed)>0.2):
                reward = reward*0.2;
            if(abs(headings[1]-headings[0]-params['steering_angle'])>3):
                reward = reward*0.3;
            if(all_decreasing and not params['is_left_of_center']):
                reward = reward * 0.5;
            if(all_increasing and params['is_left_of_center']):
                reward = reward * 0.5;
        elif(index==2):
            if(abs(params['speed']-opt_speed-0.1)>0.2):
                reward = reward*0.2;
        elif(index==3):
            if(abs(params['speed']-opt_speed-0.3)>0.2):
                reward = reward*0.2
        elif(index==4):
            if(abs(params['speed']-opt_speed-0.5)>0.2):
                reward = reward*0.2;
        
    return float(1+reward)*progress;
    
    
    
def progress_reward(params):
    progress = params['progress']
    steps = params['steps']+1
    track_length = params['track_length']
    reward = (progress)/(100*steps)
    return 1+reward;