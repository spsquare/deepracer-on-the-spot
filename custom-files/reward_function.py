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
        diff = abs(degrees[i]-degrees[0]);
        if(diff>180):
            diff = 360-diff;
        if(diff>maxdiff):
            maxdiff = max(abs(diff),maxdiff);
            index = i;
    maxdiff = maxdiff/index;
    k = max_steering_diff;
    if(max_steering_diff>180):
        max_steering_diff = max_steering_diff-360;
    elif(max_steering_diff<180):
        max_steering_diff = max_steering_diff+360;


    abs_speed = min(4.0,round(get_abs_speed(maxdiff),1));
    if(maxdiff<5):
        abs_speed = 4;
    steering_speed = min(4.0,round(get_abs_speed(abs(max_steering_diff)),1));
    if(abs(max_steering_diff)<5):
        steering_speed = 4;

    speed_diff = abs(params['speed']-min(abs_speed,steering_speed));

    reward =  200/(1+50*speed_diff);


    direction_diff = abs(k-params['steering_angle']);
    if(direction_diff>180):
        direction_diff = 360-direction_diff;
    reward = reward * (300/(50*direction_diff+1));

    return float(progress_reward(params)*reward);


def progress_reward(params):
    progress = params['progress']
    steps = params['steps']+1
    reward = (progress)/(100*steps)
    return 1+reward;

def get_abs_speed(diff):
    return max(1.3,54/(12+abs(diff)));