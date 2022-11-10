import numpy as np
import transforms3d


def get_distance(initial_state, final_state, l1=True):
    """Get the L1 distance between the initial and final states

    Args:
        initial_state (list): Current position, yaw and velocity
        final_state (list): Reference position, yaw and velocity
        l1 (bool, optional): Whether to use distance measure as L1 or L2 norm. Defaults to True.

    Returns:
        _type_: _description_
    """
    x1, y1, theta1, _ = initial_state
    x2, y2, theta2, _ = final_state

    if l1:
        return np.abs(x2-x1) + np.abs(y2-y1)
    else:
        return np.sqrt(np.abs(x1-x2)**2 + np.abs(y1-y2)**2)


def get_ground_truth_state(data, frame):
    """Given the frame number, return the ground truth state

    Args:
        data (SimulatorData): Ground truth data collected from simulator
        frame (int): Current frame number

    Returns:
        list: Current state based on the provided frame
    """
    psi = data.cams[0].initial_state[frame][2]
    psi = np.radians((psi+360)%360)
    state =  [data.cams[0].initial_state[frame][0], data.cams[0].initial_state[frame][1], psi, data.cams[0].initial_state[frame][3]]
    return state


def search_ground_truth(initial_state, c_frame, data, horizon=10):
    """Find the closest frame in ground truth to the current vehicle position

    Args:
        initial_state (list): Initial state with position, yaw and velocity
        c_frame (int): Current frame value
        data (SimulatorData): Data containing state values for cameras
        horizon (int, optional): No of steps to look into the future. Defaults to 10.

    Returns:
        int: Frame no of reference state
    """
    d = np.inf
    frame = c_frame
    for f in range(c_frame, c_frame+40):
        state = get_ground_truth_state(data, f)
        curr_d = get_distance(initial_state, state, l1=False)

        if curr_d < d:
            d = curr_d
            frame = f

    return frame


def get_final_state(initial_state, c_frame, data, horizon=10):
    """Get reference / final state based on the initial state

    Args:
        initial_state (list): Current position, yaw and velocity 
        c_frame (int): Current frame
        data (SimulatorData): Simulator data as sequence
        horizon (int, optional): No of steps in the future. Defaults to 10.

    Returns:
        tuple: Reference state, future frame no, current frame no, distance
    """
    c_frame = search_ground_truth(initial_state, c_frame, data, horizon=horizon)
    c_state = get_ground_truth_state(data, c_frame)
    f_frame = c_frame + 1
    final_state = get_ground_truth_state(data, f_frame)
    d = get_distance(initial_state, final_state, l1=False)
    
    if d >= horizon:
        return (final_state, f_frame, c_frame, d)
    else:
        while d < horizon:
            f_frame += 1
            prev_state = final_state
            final_state = get_ground_truth_state(data, f_frame)
            d += get_distance(prev_state, final_state, l1=False)
        return (final_state, f_frame, c_frame, d)


def get_relative_cam_states(initial_state, final_state, y=0., yaw=0.):
    r_initial_state = initial_state.copy()
    r_initial_state[2] += np.radians((yaw+360)%360)

    R = transforms3d.euler.euler2mat(0,0,yaw).T
    vehicle_loc_relative = np.dot(R, np.array([y,0.,0.]))
    r_initial_state[0] += vehicle_loc_relative[1]
    r_initial_state[1] += vehicle_loc_relative[0]
    r_initial_state[2] += np.radians((yaw+360)%360)

    return r_initial_state, final_state
