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
    if(diff<10):
        return 4;
    return max(1.3, 40/diff)

def progress_reward_func(params):
    progress = params['progress']
    steps = params['steps']+1
    reward = (progress)/(steps)
    return reward;
    
    
def reward_function(params):
    if params['is_offtrack'] or params['is_crashed']:
        return 1e-9
    waypoints = params['waypoints'];
    left_of_center = params['is_left_of_center'];
    start = int(params['closest_waypoints'][0]);
    end = int(params['closest_waypoints'][1]);
    cs = params['speed'];
    steering = params['steering_angle'];
    track_width = params['track_width'];
    distance_from_center = params['distance_from_center']
    len_wp = len(waypoints);
    curr = gen_angle(waypoints[end%len_wp],waypoints[(start-1+len_wp)%len_wp])

    angles = []
    for k in range(end+1,end+2):
        nxt = waypoints[k%len_wp];
        prev = waypoints[(k-1+len_wp)%len_wp];
        if(nxt[1]==prev[1] and nxt[0]==prev[0]):
            continue;
        angles.append(gen_angle(nxt,prev));
    prev_angles = [];
    for k in range(start-2,start-1):
        nxt = waypoints[(k+len_wp)%len_wp];
        prev = waypoints[(k-1+len_wp)%len_wp];
        if(nxt[1]==prev[1] and nxt[0]==prev[0]):
            continue;
        prev_angles.append(gen_angle(nxt,prev));
    prev_angles.append(curr);
    total = 0;
    window = 1;
    prev_total = 0;
    for k in range(window,len(angles)):
        total=total+(get_diff(angles[k],angles[k-window])/k);

    for k in range(window,len(prev_angles)):
        prev_total=prev_total+(get_diff(prev_angles[k],prev_angles[k-window])/(len(prev_angles)-k));

    req_steer = round((total+prev_total)/2)
    req_speed = round(get_abs_speed(abs(req_steer)),1);
    speed_diff = abs(cs-req_speed);
    if(abs(req_steer)<10):
        reward = 0;
        re_speed = 100/(1+10*speed_diff);
        re_steer = 100/(1+10*abs(steering-req_steer));
        if(cs>3.0):
            re_speed+=10;
        if(cs>3.2):
            re_speed+=10;
        if(cs>3.4):
            re_speed+=10;
        if(cs>3.6):
            re_speed+=10;
        if(cs>3.8):
            re_speed+=10;
        if(cs>4.0):
            re_speed+=40;

        re_distance = 0;
        if(distance_from_center<0.5*track_width):
            re_distance+=10;
        if(distance_from_center<0.4*track_width):
            re_distance+=10;
        if(distance_from_center<0.3*track_width):
            re_distance+=10;
        if(distance_from_center<0.2*track_width):
            re_distance+=10;
        if(distance_from_center<0.1*track_width):
            re_distance+=10;
        reward+=re_distance;
        

        reward+=re_speed;
        reward+=re_steer;
        return reward;
    else:
        reward = 0;
        re_speed = 50/(1+10*speed_diff);
        re_steer = 50/(1+10*abs(steering-req_steer));
        if(req_steer*steering<0):
            return 1e-9;
        
        if(req_steer<0 and not left_of_center):
            re_steer+=40;
        elif(req_steer>0 and left_of_center):
            re_steer+=40;
        re_heading = 50/(1+10*abs(curr-params['heading']));
    
        reward+=re_heading+ re_speed + re_steer + progress_reward_func(params);
        return reward;