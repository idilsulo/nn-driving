"""
mpc.py

* Adapted from:
     https://github.com/WuStangDan/mpc-course-assignments/blob/master/assignment2.py
"""

# import sys
# sys.path.append('../')

import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
from scipy.optimize import minimize, LinearConstraint, Bounds
import time
import pickle

import arguments
from config import SimulatorParams, MPCParams, CamParams
from data.simulator_data import *
from utils.data_utils import *

sim_params = SimulatorParams()
mpc_params = MPCParams()
cam_params = CamParams()


class ModelPredictiveControl:
    def __init__(self, initial=[0,0,0,0], reference=[14,7,-3.14/4]):
        self.horizon = 20 # The number of time stamps ahead to predict
        self.dt = 0.2     # Time difference between to states

        # Initial state 
        self.initial = initial

        # Reference or set point the controller will achieve.
        # Do not use math.pi as it will cause precision errors
        self.reference1 = reference
        self.reference2 =  None 

    def plant_model(self, prev_state, dt, pedal, steering):
        """Plant the model at the previous state and delta t 

        Args:
            prev_state (list): _description_
            dt (float): _description_
            pedal (float): _description_
            steering (float): _description_

        Returns:
            list: [x_t, y_t, psi_t, v_t]
        """
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t = prev_state[3]

        x_t = x_t + v_t * np.cos(psi_t) * dt
        y_t = y_t + v_t * np.sin(psi_t) * dt
        psi_t = psi_t + v_t * np.tan(steering) / 2.5 * dt
        v_t = v_t * 0.95 + pedal * dt
        # v_t = v_t + pedal * dt

        return [x_t, y_t, psi_t, v_t]

    def cost_function(self, u, *args):
        """Compute cost for the provided horizon 

        Args:
            u (list): pair of pedal and steering values
            state: the state at timestep t (x_t, y_t, psi_t, v_t)
            ref: reference values (x_t, y_t, psi_t, v_t) the controller will achieve
        Returns:
            cost: Total cost computed through the horizon
        """

        state = args[0]
        ref = args[1]

        interim_ref = None
        interim_ref2 = None

        if len(args) > 2:
            interim_ref = args[2]
        if len(args) > 3:
            interim_ref2 = args[3]

        cost = 0.0

        for i in range(self.horizon):
            state = self.plant_model(state, self.dt, u[i*2], u[2*i+1])

            psi_dist = np.abs(ref[2] - state[2])
            if psi_dist > np.pi:
                psi_dist = np.abs(psi_dist - 2*np.pi)

            cost += (np.abs(ref[0] - state[0])**2 + np.abs(ref[1] - state[1])**2)
            cost +=  (psi_dist**2)

            if interim_ref:
                psi_dist = np.abs(interim_ref[2] - state[2])
                if psi_dist > np.pi:
                    psi_dist = np.abs(psi_dist - 2*np.pi)

                cost += (np.abs(interim_ref[0] - state[0])**2 + np.abs(interim_ref[1] - state[1])**2) + (psi_dist**2)

            if interim_ref2:
                psi_dist = np.abs(interim_ref2[2] - state[2])
                if psi_dist > np.pi:
                    psi_dist = np.abs(psi_dist - 2*np.pi)

                cost += (np.abs(interim_ref2[0] - state[0])**2 + np.abs(interim_ref2[1] - state[1])**2) + (psi_dist**2)

        return cost


def calculate_u(mpc, initial, reference, sim_total=250, show=False, interim_state=None, interim_state2=None):
    """Calculate the expected steering value

    Args:
        mpc (ModelPredictiveControl): 
        initial (list): Initial state
        reference (list): Reference state
        sim_total (int, optional): Total simulation steps. Defaults to 250.
        show (bool, optional): Whether to visualize optimization steps. Defaults to False.
        interim_state (list, optional): Intermediate state calculation. Defaults to None.
        interim_state2 (list, optional): Additional intermediate state calculation.. Defaults to None.

    Returns:
        list: Optimized steering comand sequence
    """
    start = time.process_time()
    mpc.initial = initial
    mpc.reference1 = reference

    num_inputs = 2
    u = np.zeros(mpc.horizon*num_inputs) # A flat numpy array with steering and pedal values
                                         # for each frame in the horizon
    bounds = []

    # Set bounds for inputs bounded optimization.
    for i in range(mpc.horizon):
        # The car should not go backwards
        # The problem is that throttle < 0 is also used for slowing the vehicle down 
        # (i.e break) instead of going backwards
        bounds += [[0, 1]]          # Bounds for throttle / pedal
        bounds += [[-1.222, 1.222]] # Bounds for steering angle

    ref_1 = mpc.reference1
    ref_2 = mpc.reference2
    ref = ref_1

     # Initial state
    state_i = np.array([mpc.initial])

    u_i = np.array([[0,0]])
    # sim_total = 250
    predict_info = [state_i]


    for i in range(1,sim_total+1):
        u = np.delete(u,0)
        u = np.delete(u,0)
        u = np.append(u, u[-2])
        u = np.append(u, u[-2])
        start_time = time.time()

        # Non-linear optimization.
        u_solution = minimize(mpc.cost_function, u, (state_i[-1], ref, interim_state, interim_state2),
                                method='SLSQP',
                                bounds=bounds,
                                # constraints=constraint,
                                tol = 1e-5)
        print('Step ' + str(i) + ' of ' + str(sim_total) + '   Time ' + str(round(time.time() - start_time,5)),end='\r')
        u = u_solution.x
        y = mpc.plant_model(state_i[-1], mpc.dt, u[0], u[1])
        if (i > 130 and ref_2 != None):
            ref = ref_2
        predicted_state = np.array([y])
        for j in range(1, mpc.horizon):
            predicted = mpc.plant_model(predicted_state[-1], mpc.dt, u[2*j], u[2*j+1])
            predicted_state = np.append(predicted_state, np.array([predicted]), axis=0)
        predict_info += [predicted_state]
        state_i = np.append(state_i, np.array([y]), axis=0)
        u_i = np.append(u_i, np.array([(u[0], u[1])]), axis=0)
    
    if show:
        ###################
        # SIMULATOR DISPLAY

        # Total Figure
        fig = plt.figure(figsize=(8,8))
        gs = gridspec.GridSpec(8,8)

        # Elevator plot settings.
        ax = fig.add_subplot(gs[:8, :8])

        plt.xlim(initial[0]-30, initial[0]+30)
        ax.set_ylim([initial[1]-17, initial[1]+17])
        plt.xticks(np.arange(initial[0]-27,initial[0]+57, step=2))
        plt.yticks(np.arange(initial[1]-27,initial[1]+57, step=2))
        plt.title('MPC 2D')

        # Time display.
        time_text = ax.text(6, 0.5, '', fontsize=15)

        # Main plot info.
        car_width = 1.0
        patch_car = mpatches.Rectangle((0, 0), car_width, 2.5, fc='k', fill=False)
        patch_goal = mpatches.Rectangle((0, 0), car_width, 2.5, fc='b',
                                        ls='dashdot', fill=False)

        ax.add_patch(patch_car)
        ax.add_patch(patch_goal)
        predict, = ax.plot([], [], 'r--', linewidth = 1)

        # Car steering and throttle position.
        telem = [3,14]
        patch_wheel = mpatches.Circle((telem[0]-3, telem[1]), 2.2)
        ax.add_patch(patch_wheel)
        wheel_1, = ax.plot([], [], 'k', linewidth = 3)
        wheel_2, = ax.plot([], [], 'k', linewidth = 3)
        wheel_3, = ax.plot([], [], 'k', linewidth = 3)
        throttle_outline, = ax.plot([telem[0], telem[0]], [telem[1]-2, telem[1]+2],
                                    'b', linewidth = 20, alpha = 0.4)
        throttle, = ax.plot([], [], 'k', linewidth = 20)
        brake_outline, = ax.plot([telem[0]+3, telem[0]+3], [telem[1]-2, telem[1]+2],
                                'b', linewidth = 20, alpha = 0.2)
        brake, = ax.plot([], [], 'k', linewidth = 20)
        throttle_text = ax.text(telem[0], telem[1]-3, 'Forward', fontsize = 15,
                            horizontalalignment='center')
        brake_text = ax.text(telem[0]+3, telem[1]-3, 'Reverse', fontsize = 15,
                            horizontalalignment='center')

        # Shift xy, centered on rear of car to rear left corner of car.
        def car_patch_pos(x, y, psi):
            x_new = x - np.sin(psi)*(car_width/2)
            y_new = y + np.cos(psi)*(car_width/2)
            return [x_new, y_new]

        def steering_wheel(wheel_angle):
            wheel_1.set_data([telem[0]-3, telem[0]-3+np.cos(wheel_angle)*2],
                             [telem[1], telem[1]+np.sin(wheel_angle)*2])
            wheel_2.set_data([telem[0]-3, telem[0]-3-np.cos(wheel_angle)*2],
                             [telem[1], telem[1]-np.sin(wheel_angle)*2])
            wheel_3.set_data([telem[0]-3, telem[0]-3+np.sin(wheel_angle)*2],
                             [telem[1], telem[1]-np.cos(wheel_angle)*2])

        def update_plot(num):
            # Car.
            patch_car.set_xy(car_patch_pos(state_i[num,0], state_i[num,1], state_i[num,2]))
            patch_car.angle = np.rad2deg(state_i[num,2])-90
            # Car wheels
            np.rad2deg(state_i[num,2])
            steering_wheel(u_i[num,1]*2)
            throttle.set_data([telem[0],telem[0]],
                            [telem[1]-2, telem[1]-2+max(0,u_i[num,0]/5*4)])
            brake.set_data([telem[0]+3, telem[0]+3],
                            [telem[1]-2, telem[1]-2+max(0,-u_i[num,0]/5*4)])

            # Goal.
            if (num <= 130 or ref_2 == None):
                patch_goal.set_xy(car_patch_pos(ref_1[0],ref_1[1],ref_1[2]))
                patch_goal.angle = np.rad2deg(ref_1[2])-90
            else:
                patch_goal.set_xy(car_patch_pos(ref_2[0],ref_2[1],ref_2[2]))
                patch_goal.angle = np.rad2deg(ref_2[2])-90

            predict.set_data(predict_info[num][:,0],predict_info[num][:,1])
            
            return patch_car, time_text


        print("Compute Time: ", round(time.process_time() - start, 3), "seconds.")
        # Animation.
        car_ani = animation.FuncAnimation(fig, update_plot, frames=range(1,len(state_i)), interval=100, repeat=True, blit=False)
        plt.show()

    return u_i.reshape(len(u_i)*2), u, predict_info


def apply_mpc(args, data, mpc_data, horizon=15, dt=0.05, interim_cost=False, relative_states=False):
    """Apply MPC over the provided sequence 

    Args:
        args: Command line arguments
        data (SimulatorData): Data containing reference trajectory
        mpc_data (SimulatorData): Data to save resulting MPC states
        horizon (int, optional): How many steps into the future to optimize at each step. Defaults to 15.
        dt (float, optional): Time difference between each step. Defaults to 0.05.
        interim_cost (bool, optional): Whether to use intermediate step. Defaults to False.
        relative_states (bool, optional): Whether to take relative states during MPC optimization. Defaults to False.

    Returns:
        SimulatorData: Data containing MPC results
    """
    mpc = ModelPredictiveControl()
    mpc.horizon = horizon
    mpc.dt = dt

    # Get center camera
    cam = data.cams[0]

    c_frame = sim_params.NO_SKIPPED_FRAMES + 1

    try:

        for frame in range(sim_params.NO_SKIPPED_FRAMES+1, sim_params.NO_FRAMES+1):
            print("FRAME #{}".format(frame))

            # Use fixed speed instead of ground-truth speed
            v = 6

            # initial_state_center = [x, y, psi, v]
            initial_state_center = data.cams[0].initial_state[frame]
            psi = data.cams[0].initial_state[frame][2]
            initial_state_center[2] = np.radians((psi+360)%360)
            initial_state_center[3] = v
            
            final_state, f_frame, c_frame_updated, _ = get_final_state(initial_state_center, c_frame, data, horizon=mpc_params.GT_SEARCH)
            
            interim_state = None
            interim_state2 = None
            if interim_cost:
                interim_state, _, _, _ = get_final_state(initial_state_center, c_frame, data, horizon=3)
            
            c_frame = c_frame_updated

            # In order to check turns, use the initial state from center camera
            psi_dist = np.abs(final_state[2] - initial_state_center[2])
            if psi_dist > np.pi:
                psi_dist = np.abs(psi_dist - 2*np.pi)
        
            # The current state is considered as a turn if the angle difference between the initial and the reference 
            #   state is greater than 5 degrees
            if psi_dist > np.radians(5):
                final_state = interim_state
                interim_state = None
                mpc.horizon = 2
            else:
                interim_state = None
                mpc.horizon = horizon
            

            for i, cam in enumerate(mpc_data.cams):

                # Add camera transform to the initial state
                if not relative_states:
                    if i == 0:
                        initial_state = initial_state_center
                    else:
                        initial_state = data.cams[i].initial_state[frame]
                        initial_state[2] = np.radians((initial_state[2]+360)%360)
                        initial_state[3] = v
                else:
                    r_initial_state, r_final_state = get_relative_cam_states(initial_state_center, final_state, y=cam.y, yaw=cam.yaw)

                    print("INITIAL STATE    : ", initial_state_center)
                    print("FINAL STATE      : ", final_state)
                    print("R. INITIAL STATE : ", r_initial_state)
                    print("R. FINAL STATE   : ", r_final_state)
           
                try:
                    if not relative_states:
                        u, u_2, predict_info = calculate_u(mpc, initial_state, final_state, sim_total=1, show=False, interim_state=interim_state, interim_state2=interim_state2)    
                    else:
                        u, u_2, predict_info = calculate_u(mpc, r_initial_state, r_final_state, sim_total=1, show=False, interim_state=interim_state, interim_state2=interim_state2)

                    steering = np.mean(u_2.reshape(len(u_2)//2,2)[mpc_params.STEER,1])

                except:
                    # Horizon and ground-truth search values should be chosen such that this case does not occur
                    print("Bound error, use previous steering value.")
                    if len(cam.steering) > 0:
                        steering = cam.steering[-1]
                    else:
                        steering = 0.

                throttle =  0.6

                print(initial_state)
                
                if not relative_states:
                    cam.initial_state[frame] = initial_state
                    cam.final_state.append(final_state)
                else:
                    cam.initial_state[frame] = r_initial_state
                    cam.final_state.append(r_final_state)
                
                cam.horizon.append(predict_info)
                cam.steering.append(steering)
                cam.throttle.append(throttle)

            if f_frame >= sim_params.NO_FRAMES:
                break


    except Exception as e:
        print(e)
        
    return mpc_data

if __name__ == '__main__':

    ap = arguments.make_parser()
    args = ap.parse_args()

    town_no = sim_params.TOWN[4:]

    # TEST CAMERA PARAMETER ARGS
    assert(cam_params.NO_CAMS * 2 == len(cam_params.CAM_PARAMS))
    assert(cam_params.NO_CAMS == len(cam_params.CAM_NAMES))

    dirs = os.listdir(args.OUT_PATH + args.EPISODE)
    print(dirs)

    for pos in dirs:
        print("{}/{}{}/gt/".format(args.OUT_PATH, args.EPISODE, pos))
        if os.path.exists("{}/{}{}/gt/".format(args.OUT_PATH, args.EPISODE, pos)):

            if args.GT:
                data = pickle.load(open(args.OUT_PATH + args.EPISODE + pos + '/gt/data.pkl', 'rb'))
            else:
                data = pickle.load(open(args.OUT_PATH + args.EPISODE + pos + '/gt/data_slam.pkl', 'rb'))
            
            mpc_data = SimulatorData(no_cameras=cam_params.NO_CAMS, cam_params=cam_params.CAM_PARAMS, cam_names=cam_params.CAM_NAMES)
            mpc_data = apply_mpc(args, data, mpc_data, horizon=mpc_params.HORIZON, dt=mpc_params.MPC_DT, interim_cost=mpc_params.INTERIM_COST,
                                relative_states=False)

            os.makedirs(args.OUT_PATH + args.EPISODE + pos + '/mpc', exist_ok=True)

            if args.GT:
                pickle.dump(mpc_data, open(args.OUT_PATH + args.EPISODE + pos + '/mpc/mpc_gt_data.pkl', 'wb'))
            else:
                pickle.dump(mpc_data, open(args.OUT_PATH + args.EPISODE + pos + '/mpc/mpc_slam_data.pkl', 'wb'))