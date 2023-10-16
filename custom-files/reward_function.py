import math;

def get_diff(angle1,angle2):
    diff = angle1-angle2;
    if(diff>180):
        diff-=360;
    elif(diff<-180):
        diff+=360;
    return diff;


def gen_angle(nxt,prev):
    deg = math.degrees(math.atan2(nxt[1]-prev[1],nxt[0]-prev[0]));
    return deg

def get_abs_speed(diff):
    if(diff<=5):
        return 4;
    if(diff<=15):
        return 200/(35+3*diff);
    return max(1.4,43/(diff+2));

def progress_reward_func(params):
    progress = params['progress']+1
    steps = params['steps']+1
    reward = (progress)/(steps)
    return reward;
    
    
def reward_function(params):
    if params['is_offtrack'] or params['is_crashed']:
        return 1e-9
    waypoints = params['waypoints'];
    is_left_of_center = params['is_left_of_center'];
    start = int(params['closest_waypoints'][0]);
    track_width = params['track_width'];
    end = int(params['closest_waypoints'][1]);
    cs = params['speed'];
    steering = params['steering_angle'];
    distance_from_center = params['distance_from_center']
    len_wp = len(waypoints);
    
    curr = gen_angle(waypoints[end%len_wp],waypoints[start%len_wp])

    angles = [];
    angles.append(curr);
    for k in range(end+1,end+4):
        angle = gen_angle(waypoints[(k)%len_wp],waypoints[(k-1)%len_wp])
        if(angle==0):
            continue;
        angles.append(angle);
    angle = 0;
    for k in range(1,len(angles)):
        angle+=get_diff(angles[k],angles[k-1])/k
    alt_steer = round(angle);
    angles = [];
    angles.append(curr);
    for k in range(start-1,start-4):
        angle = gen_angle(waypoints[(k+1)%len_wp],waypoints[(k)%len_wp])
        if(angle==0):
            continue;
        angles.append(angle);
    angle = 0;
    for k in range(1,len(angles)):
        angle+=get_diff(angles[k],angles[k-1])/k;
    
    req_steer = (alt_steer+angle);
    req_speed = round(get_abs_speed(abs(req_steer)),1);
    reward = 1;
    
    
    steer_diff = abs(req_steer-steering);
    re_steer = 100/(1+steer_diff);
    if(steer_diff<5):
        re_steer+=20;
    if(steer_diff<4):
        re_steer+=20;
    if(steer_diff<3):
        re_steer+=20;
    if(steer_diff<2):
        re_steer+=20;
    if(steer_diff<1):
        re_steer+=20;
    
    speed_diff = 10*abs(cs-req_speed);
    
    re_speed = 50/(1+speed_diff);
    if(speed_diff<5):
        re_speed+=20;
    if(speed_diff<4):
        re_speed+=20;
    if(speed_diff<3):
        re_speed+=20;
    if(speed_diff<2):
        re_speed+=20;
    if(speed_diff<1):
        re_speed+=20;
        
    if(abs(req_steer)<=5):
        data = 2*distance_from_center/track_width
        if(data<0.5):
            reward+=10;
        if(data<0.4):
            reward+=10;
        if(data<0.3):
            reward+=10;
        if(data<0.2):
            reward+=10;
        dir_diff = abs(curr-params['heading']);
        if(dir_diff>180):
            dir_diff = 360-dir_diff;
        reward+= 50/(1+10*dir_diff);    
    else:
        if(req_steer<0 and not is_left_of_center):
            reward+=30;
        if(req_steer>0 and is_left_of_center):
            reward+=30;
    reward+=re_speed+re_steer+progress_reward_func(params);
    
    return reward;