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
    if(diff<5):
        return 4;
    return max(1.4, 4*math.cos(math.pi*diff/50))

def progress_reward_func(params):
    progress = params['progress']
    steps = params['steps']+1
    reward = (progress)/(steps)
    return 1+reward;
    
    
def reward_function(params):
    if params['is_offtrack'] or params['is_crashed']:
        return 1e-9
    waypoints = params['waypoints'];
    start = int(params['closest_waypoints'][0]);
    end = int(params['closest_waypoints'][1]);
    len_wp = len(waypoints);
    
    while(waypoints[start%len_wp][0]==waypoints[end%len_wp][0] and waypoints[start%len_wp][1]==waypoints[end%len_wp][1]):
        end+=1;
        end = end%len_wp;
    
    curr = gen_angle(waypoints[end%len_wp],waypoints[start%len_wp])
    nxt = gen_angle(waypoints[(end+2)%len_wp],waypoints[end%len_wp]);
    prev = gen_angle(waypoints[(start)%len_wp],waypoints[(start-2+len_wp)%len_wp])

    diff1 = get_diff(curr,prev);
    diff2 = get_diff(nxt,curr);
    
    total = (diff1+diff2)/2
        
    req_steer = round(total)
    req_speed = round(get_abs_speed(abs(req_steer)),1);
    
    direction_diff = abs(curr-params['heading']);
    steer_diff = abs(params['steering_angle']-req_steer);
    if(steer_diff<3 and (params['steering_angle']*req_steer>0 or params['steering_angle']-req_steer==0)):
        steer_diff = 0;
    steering_reward = 100/(1+(steer_diff));
    direction_reward = 10/(1+10*direction_diff);
    speed_reward = 10/(1+abs(params['speed']-req_speed));
    cs = params['speed'];
    if(abs(req_steer)<5):
        cs = 0;
        if(cs>3.0):
            speed_reward+=10;
        if(cs>3.2):
            speed_reward+=10;
        if(cs>3.4):
            speed_reward+=10;
        if(cs>3.6):
            speed_reward+=10;
        if(cs>3.8):
            speed_reward+=10;
        if(cs>4):
            speed_reward+=20;
    progress_reward = progress_reward_func(params);
    
    reward = steering_reward+direction_reward+speed_reward+progress_reward
    
    track_width = params['track_width'];
    distance_from_center = params['distance_from_center']
    left_of_center = params['is_left_of_center'];
    marker_1 = 0.1*track_width;
    if(req_steer<5):
        marker_1 = 0.1*track_width;
    elif(req_steer<10):
        marker_1 = track_width/6;
    elif(req_steer<20):
        marker_1 = track_width/3;
    
    
    if(distance_from_center<=marker_1):
        reward += 50;
    else:
        reward -=30;
    if(req_steer<-3 and not left_of_center):
        reward += 50;
    elif(req_steer>3 and left_of_center):
        reward += 50;
    
    
    return reward;