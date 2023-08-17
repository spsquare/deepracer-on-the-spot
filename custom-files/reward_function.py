def reward_function(params):
    if params['is_offtrack'] or params['is_crashed']:
        return 1e-9
    if params['steps'] > 0:
        reward =(params['progress'])/(params['steps']*100)+(params['speed']**2)
    else:
        reward = 1e-9

    return float(reward)