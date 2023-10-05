import math
def reward_function(params):
    if params['is_offtrack'] or params['is_crashed']:
        return 1e-9;
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints'];
    cwi = closest_waypoints[1];
    degrees = [];
    for i in range(0,6):
        nxt = waypoints[(cwi+i)%len(waypoints)];
        prev = waypoints[(len(waypoints)+cwi+i-1)%len(waypoints)];
        if(nxt[1]==prev[1] and nxt[0]==prev[0]):
            continue;
        angle = math.atan2(nxt[1]-prev[1],nxt[0]-prev[0]);
        degrees.append(math.degrees(angle));
    maxdiff = 0.0;
    max_steering_diff = degrees[1]-degrees[0];
    index = 0;
    for i in range(1,len(degrees)):
        diff = degrees[i]-degrees[0];
        if(diff>180):
            diff = diff-360;
        elif(diff<-180):
            diff = diff+360
        if(abs(diff)>abs(maxdiff)):
            maxdiff = diff;
            index = i;
    maxdiff = maxdiff/index;
    if(max_steering_diff>180):
        max_steering_diff = max_steering_diff-360;
    elif(max_steering_diff<-180):
        max_steering_diff = max_steering_diff+360;
    req_steer = max_steering_diff;
    if(abs(maxdiff)>abs(max_steering_diff)):
        req_steer = maxdiff;
    req_speed = round(get_abs_speed(abs(req_steer)),1);
    reward = 1.0;
    speed_diff = abs(params['speed']-req_speed);
    reward = 10/(1+10*abs(speed_diff));
    
    direction_diff = abs(round(req_steer)-params['steering_angle']);
    if(direction_diff>180):
        direction_diff = 360-direction_diff;
        
    print("Speed Diff : {}, Direction Diff : {}".format(speed_diff,direction_diff))
    reward*= 50/(10+(direction_diff//2))
    len_wp = len(waypoints);
    heading = params['heading']
    next_wp = waypoints[closest_waypoints[1]];
    prev_wp = waypoints[closest_waypoints[0]];
    if(prev_wp[0]==next_wp[0] and prev_wp[1]==next_wp[1]):
        prev_wp = waypoints[(len_wp+closest_waypoints[0]-1)%len_wp];
    prev_angle = math.degrees(math.atan2(next_wp[1]-prev_wp[1],next_wp[0]-prev_wp[0]));
    heading = params['heading']
    heading_diff = abs(prev_angle-heading-params['steering_angle']);
    if(heading_diff>180):
        heading_diff = 360-heading_diff;
    reward*= 10/(1+5*(heading_diff//3));
    return float(progress_reward(params)*reward);


def progress_reward(params):
    progress = params['progress']
    steps = params['steps']+1
    reward = (progress)/(100*steps)
    return 1+reward;


def get_abs_speed(diff):
    return min(4,54/(12+diff));