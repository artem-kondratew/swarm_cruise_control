import numpy as np

from .swarm import Agent


def get_peers_vecs(agents : list[Agent], current_agent : Agent, pacemaker : Agent, logger, R_vis=np.inf) -> list:
    peers_vecs = []
    
    # logger.info(f"{pacemaker.pt, current_agent.pt, pacemaker.id, current_agent.id}")
    
    # vec = pacemaker.pt - current_agent.pt
    # if np.linalg.norm(vec) <= R_vis:
    #     peers_vecs.append(vec)

    for agent in agents:
        if agent.id == current_agent.id:
            continue
        vec = agent.pt - current_agent.pt
        if np.linalg.norm(vec) <= R_vis:
            peers_vecs.append(vec)
            
    # logger.info(f'peers: {peers_vecs}')

    return peers_vecs


def select_peers_and_azimuth(peers_vecs : list, pacemaker : Agent, current_agent : Agent, logger) -> tuple:
    '''
    Selects 4 best peers and calculates an azimuth of current target direction for given agent

    Params:
        peers : list
            list of peers (2D-vectors)
        pacemaker : Pacemaker
            pacemaker object
        agent : Agent, AgentReal
            current agent object

    Returns:
        nearest_peer_forward : np.array(2)
            the best peer in forward side semiplane 
        nearest_peer_back : np.array(2)
             : the best peer in backward side semiplane  
        nearest_peer_left : np.array(2)
             : the best peer in left side semiplane 
        nearest_peer_right : np.array(2)
             : the best peer in right side semiplane 
        azimuth : float
            azimuth of the target direction
    '''
    vec_to_pacemaker = pacemaker.pt - current_agent.pt
    nearest_peer_to_pacemaker_vec = None
    nearest_dist_to_pacemaker = np.inf

    if len(peers_vecs) > 0:
        for peer_vec in peers_vecs:
            dist_to_pacemaker = np.linalg.norm(vec_to_pacemaker - peer_vec)
            if dist_to_pacemaker < nearest_dist_to_pacemaker:
                nearest_dist_to_pacemaker = dist_to_pacemaker
                nearest_peer_to_pacemaker_vec = peer_vec
    else:
        nearest_peer_to_pacemaker_vec = vec_to_pacemaker

    R_90 = np.array([
        [0, -1],
        [1, 0],
    ])
    
    side_dir = R_90 @ nearest_peer_to_pacemaker_vec
    
    nearest_peer_forward, nearest_peer_back = None, None
    nearest_peer_left, nearest_peer_right = None, None
    nearest_dist_forward, nearest_dist_back = np.inf, np.inf
    nearest_dist_left, nearest_dist_right = np.inf, np.inf
    
    azimuth = np.arctan2(nearest_peer_to_pacemaker_vec[1], nearest_peer_to_pacemaker_vec[0])
        
    assert side_dir.shape == (2, 1)

    for peer_vec in peers_vecs:
        dist_fore = nearest_peer_to_pacemaker_vec.T @ peer_vec
        dist_side = side_dir.T @ peer_vec
        
        flag_forward = dist_fore >= 0
        flag_left = dist_side >= 0
        
        if flag_forward:
            if dist_fore < nearest_dist_forward:
                nearest_dist_forward = dist_fore
                nearest_peer_forward = peer_vec
        else:
            if -dist_fore < nearest_dist_back:
                nearest_dist_back = -dist_fore
                nearest_peer_back = peer_vec
        if flag_left:
            if dist_side < nearest_dist_left:
                nearest_dist_left = dist_side
                nearest_peer_left = peer_vec
        else:
            if -dist_side < nearest_dist_right:
                nearest_dist_right = -dist_side
                nearest_peer_right = peer_vec

    return (nearest_peer_forward, nearest_peer_back, nearest_peer_left, nearest_peer_right), azimuth


def g(min_dist, gap, has_peer) -> float:
    '''
    Auxiliary function

    Params:
        min_dist : float
            distance to the peer
        gap : float
            desired spatial gap
        has_peer : bool
            does the appropriate peer exist

    Return:
        float value
    '''
    return min_dist if has_peer else gap


def control_cmd(best_peers_vecs, azimuth, agent : Agent, w, acc_max, kp_theta, logger, xi=lambda g: np.tanh(g)) -> tuple:
    '''
    Calculates control force for real agent

    Params:
        peers_and_azimuth : tuple
            best peers and azimuth to the target direction
        agent : Agent, AgentReal
            agent object
        w : float
            desired spatial gap
        acc_max : float
            agent control acceleration bound
        xi : func
            auxiliary lambda function

    Return:
        float
            linear acceleration vector
        u_theta
            angular control velocity
    '''
    peer_forward_vec, peer_back_vec, peer_left_vec, peer_right_vec = best_peers_vecs
    axis_fore = np.array([np.cos(azimuth), np.sin(azimuth)]).reshape(-1, 1)
    
    # logger.info(f'{best_peers_vecs}')

    T = np.array([
        [0, -1],
        [1, 0]
    ])
    axis_norm = T @ axis_fore

    has_peer_forward = peer_forward_vec is not None
    has_peer_back = peer_back_vec is not None
    has_peer_left = peer_left_vec is not None
    has_peer_right = peer_right_vec is not None

    d_plus_fore = axis_fore.T @ peer_forward_vec if has_peer_forward else np.inf
    d_minus_fore = -axis_fore.T @ peer_back_vec if has_peer_back else np.inf
    d_plus_side = axis_norm.T @ peer_left_vec if has_peer_left else np.inf
    d_minus_side = -axis_norm.T @ peer_right_vec if has_peer_right else np.inf

    v = agent.v
    theta = agent.theta
    
    v_vec = np.array([
        [v * np.cos(theta)],
        [v * np.sin(theta)],
    ])
    
    v = axis_fore.T @ v_vec
    sigma = v - xi(g(d_plus_fore, w, has_peer_forward)) + xi(g(d_minus_fore, w, has_peer_back))
    acc = -acc_max * np.sign(sigma)

    v_side = axis_norm.T @ v_vec
    sigma_side = v_side - xi(g(d_plus_side, 0, has_peer_left)) + xi(g(d_minus_side, 0, has_peer_right))
    u_theta = -kp_theta * np.sign(sigma_side)
    
    # logger.info(f'{sigma_side, np.sign(sigma_side)} {has_peer_left, has_peer_right}')

    return float(acc), float(u_theta)