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
    return max(1.3, 4*math.cos(math.pi*diff/50))

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
    cs = params['speed'];
    len_wp = len(waypoints);
    
    while(waypoints[start%len_wp][0]==waypoints[end%len_wp][0] and waypoints[start%len_wp][1]==waypoints[end%len_wp][1]):
        end+=1;
        end = end%len_wp;
    
    curr = gen_angle(waypoints[end%len_wp],waypoints[start%len_wp])
    nxt = gen_angle(waypoints[(end+2)%len_wp],waypoints[end%len_wp]);
    prev = gen_angle(waypoints[(start)%len_wp],waypoints[(start-2+len_wp)%len_wp])

    diff1 = get_diff(curr,prev);
    diff2 = get_diff(nxt,curr);
    
    total = (diff1+diff2)/2;
    
    req_steer = round(total)
    req_speed = round(get_abs_speed(abs(req_steer)),1);
    
    direction_diff = abs(curr-params['heading']);
    steer_diff = abs(params['steering_angle']-req_steer);
    direction_reward = 100/(1+(direction_diff))  
    steering_reward = 100/(1+(steer_diff));
    speed_diff = 10*(abs(cs-req_speed));
    if(abs(req_steer)>=5 and params['steering_angle']*req_steer<0):
        return 1e-9;
    progress_reward = progress_reward_func(params);

    speed_reward = 100/(1+speed_diff);
    if(abs(req_steer)<5):
        if(cs>3):
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
    
    track_width = params['track_width'];
    distance_from_center = params['distance_from_center']
    left_of_center = params['is_left_of_center'];
    marker_1 = round(min(track_width/2.2,max(track_width/20,(req_steer/30)*(track_width/2))),2);
    reward = direction_reward+speed_reward+progress_reward
    if(abs(req_steer)>=5):
        marker_1 = min(1,(abs(req_steer)+5)/30)*track_width/2;
        distance_from_center_reward = 0;
        if(distance_from_center>0.1*marker_1):
            distance_from_center_reward += 20;
        if(distance_from_center>0.3*marker_1):
            distance_from_center_reward += 20;
        if(distance_from_center>0.5*marker_1):
            distance_from_center_reward += 20;
        if(distance_from_center>0.7*marker_1):
            distance_from_center_reward += 20;
            
        if(distance_from_center>marker_1):
            distance_from_center_reward = 0;
        if(req_steer>5 and left_of_center):
            distance_from_center_reward+=30;
        elif(req_steer<-5 and not left_of_center):
            distance_from_center_reward+=30;
        

            
        reward+=distance_from_center_reward;
    return reward;
    