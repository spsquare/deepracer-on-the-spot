import math;

def get_angle(angle1,angle2):
    diff = angle1 - angle2;
    if(diff>180):
        diff-=360;
    elif(diff<-180):
        diff+=360;
    return diff;


def get_abs_speed(diff):
    d = diff
    if(diff<10):
        d = diff//3;
        return 4 - d*0.2
    return max(1.3,round(224/(5.3*d+13),1));

def gen_angle(nxt,prev):
    return math.degrees(math.atan2(nxt[1]-prev[1],nxt[0]-prev[0]));

def reward_function(params):
    if params['is_offtrack'] or params['is_crashed']:
        return 1e-9;
    closest_waypoints = params['closest_waypoints'];
    speed = params['speed'];
    steering_angle = params['steering_angle'];
    waypoints = params['waypoints'];
    
    len_wp = len(waypoints);
    
    cwi = int(closest_waypoints[0]);
    cwj = int(closest_waypoints[1]);
    
    angles = [];
    for i in range(cwi-2,cwj+5):
        prev = waypoints[(len_wp+i)%len_wp];
        nxt = waypoints[(len_wp+i+1)%len_wp];
        if(nxt[1]==prev[1] and nxt[0]==prev[0]):
            angles.append(0);
        else:
            angles.append(gen_angle(nxt,prev));
    
    diff1 = get_angle(angles[2],angles[0]);
    diff2 = get_angle(angles[3],angles[1]);
    
    avg = (diff2+diff1)/2;
    if(avg>180):
        avg-=360;
    elif(avg<-180):
        avg+=360;
    avg_speed = round(get_abs_speed(abs(avg)),1);   
    max_diff = 0;
    count = 0;
    index = 0;
    for i in range(2,len(angles)):
        diff = get_angle(angles[i],angles[i-1]);
        if(abs(diff)>abs(max_diff)):
            max_diff = diff;
            index = i-2;
        count+=1;
    desired_speed = get_abs_speed(abs(max_diff));
    safe_speed = round(desired_speed + index*((4-desired_speed)/count),1);
    req_speed = min(desired_speed,avg_speed);
    
    
    nxt_angle = gen_angle(waypoints[(cwj+1)%len_wp],waypoints[cwj]);
    curr_angle = gen_angle(waypoints[cwj],waypoints[cwi]);
    req_steer = get_angle(nxt_angle,curr_angle);
    
    steering_diff = abs(steering_angle-req_steer);
    if(steering_diff>180):
        steering_diff = 360-steering_diff;
    speed_diff = abs(speed-req_speed);
    
    reward = 1.0;
    
    reward*= (10/(10+speed_diff));
    reward*= (10/1+10*(steering_diff));
    return float(reward)