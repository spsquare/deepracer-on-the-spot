import math;


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
    reward = (progress)/(100*steps)
    return 1+reward;

def get_abs_speed(diff):
    if(diff<5):
        return 4 - 0.2* (diff/2);
    return max(1.3,39.3/(diff+6.23))
    
def reward_function(params):
    waypoints = params['waypoints'];
    start = int(params['closest_waypoints'][0]);
    end = int(params['closest_waypoints'][1]);
    len_wp = len(waypoints);
    while(waypoints[start%len_wp][0]==waypoints[end%len_wp][0] and waypoints[start%len_wp][1]==waypoints[end%len_wp][1]):
        end+=1;
        end = end%len_wp;
    
    curr_angle = gen_angle(waypoints[(end+len_wp)%len_wp],waypoints[(start+len_wp)%len_wp]);
    angles = [];
    angles.append(curr_angle);
    max_angle = 0;
    count = 0;
    index = end;

    nxt = waypoints[(end+1+len_wp)%len_wp];
    prev = waypoints[(start+len_wp)%len_wp]
    prev_angle = gen_angle(nxt,prev);
    nxt = waypoints[(end+len_wp)%len_wp];
    prev = waypoints[(start-1+len_wp)%len_wp]
    new_angle = gen_angle(nxt,prev);
    alt_speed = round(get_abs_speed(abs(get_diff(new_angle,prev_angle))),1);    
    for k in range(end,end+6):
        nxt = waypoints[(len_wp+k)%len_wp];
        prev = waypoints[(len_wp+k-1)%len_wp];
        if(nxt[1]==prev[1] and nxt[0]==prev[0]):
            continue;
        angle = gen_angle(waypoints[(len_wp+k)%len_wp],waypoints[(len_wp+k-1)%len_wp]);
        diff = get_diff(angle,curr_angle);
        angles.append(angle);

        if(abs(diff)>abs(max_angle)):
            max_angle = diff;
            index = k-end;
        count+=1;

    future_speed = round(get_abs_speed(abs(max_angle/(index+1))),1);

    alt_speed_1 = future_speed;
    req_steer = round(get_diff(angles[2],curr_angle));
    req_speed = min(alt_speed_1,alt_speed)
    
    curr_speed = params['speed'];
    curr_steer = params['steering_angle'];
    curr_heading = params['heading'];
    
    speed_reward = 50/(1+10*abs(curr_speed-req_speed));
    steer_reward = 50/(1+abs(curr_steer-req_steer));
    heading_reward = 50/(1+abs(round(get_diff(curr_angle,curr_heading))));
    
    
    reward = float(speed_reward+steer_reward+heading_reward+progress_reward(params));
    
    
    
    track_width = params['track_width']/2;
    distance_from_center = params['distance_from_center']
    left_of_center = params['is_left_of_center']
    
    marker_1 = (track_width*(4-req_speed)/4)+(track_width/4);
    
    if(distance_from_center < marker_1):
        if(req_speed<3.5):
            if(req_steer<0 and not left_of_center):
                reward+=25;
            elif(req_steer>0 and left_of_center):
                reward+=25;
        else:
            reward+=25;
    
    
    return reward;