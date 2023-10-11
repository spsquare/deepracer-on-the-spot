import math


def get_diff(angle1,angle2):
    diff = angle1-angle2;
    if(diff>180):
        diff-=360;
    elif(diff<-180):
        diff+=360;
    return diff;

def gen_angle(nxt,prev):
    return math.degrees(math.atan2(nxt[1]-prev[1],nxt[0]-prev[0]));

def progress_reward(params):
    progress = params['progress']
    steps = params['steps']+1
    reward = (progress)/(10*steps)
    return 1+reward;

def get_abs_speed(diff):
    return max(1.3, 4*math.cos(math.pi*diff/45))

def reward_function(params):
    
    waypoints = params['waypoints'];
    start = int(params['closest_waypoints'][0]);
    end = int(params['closest_waypoints'][1]);
    len_wp = len(waypoints);
    
    
    while(waypoints[start%len_wp][0]==waypoints[end%len_wp][0] and waypoints[start%len_wp][1]==waypoints[end%len_wp][1]):
        end+=1;
        end = end%len_wp;
    
    curr_angle = gen_angle(waypoints[(end+len_wp)%len_wp],waypoints[(start+len_wp)%len_wp]);

    nxt = waypoints[(start+len_wp)%len_wp];
    prev = waypoints[(start-2+len_wp)%len_wp]
    prev_angle = gen_angle(nxt,prev);

    nxt = waypoints[(end+2+len_wp)%len_wp];
    prev = waypoints[(end+len_wp)%len_wp]
    new_angle_1 = gen_angle(nxt,prev);
    nxt = waypoints[(start+len_wp)%len_wp];


    avg_steer = get_diff(new_angle_1,curr_angle)+get_diff(curr_angle,prev_angle)
    avg_steer/=2;
    
    
    req_steer = round(avg_steer);
    req_speed = round(get_abs_speed(abs(req_steer)),1);
    
    curr_steer = params['steering_angle'];
    curr_speed = params['speed'];
    
    speed_reward = 50/(1+5*abs(req_speed-curr_speed));
    steering_reward = 50/(1+5*abs(req_steer-curr_steer));
    
    reward = float(steering_reward+speed_reward)+progress_reward(params);
    
    track_width = params['track_width']/2;
    distance_from_center = params['distance_from_center']
    left_of_center = params['is_left_of_center']
    
    marker_1 = (track_width*(4-req_speed)/4)+(track_width/4);
    
    if(distance_from_center < marker_1):
        if(req_speed<3.5):
            if(req_steer<0 and not left_of_center):
                reward+=10;
            elif(req_steer>0 and left_of_center):
                reward+=10;
        else:
            reward*=10;
    return reward;