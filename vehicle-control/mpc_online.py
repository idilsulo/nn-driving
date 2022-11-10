"""
mpc_online.py

* Control vehicle on CARLA simulator solely via MPC 
* Used only for sanity-check purposes
"""

import arguments
from config import CamParams, SimulatorParams, MPCParams
from data.simulator_data import *
from mpc.mpc import *
from utils.sim_utils import *
from utils.data_utils import *

cam_params = CamParams()
sim_params = SimulatorParams()
mpc_params = MPCParams()


def moving_target(args, data, horizon=10, dt=0.12, optim_center_only=True, interim_cost=True,
                    test_robustness=False, perturb_percent=0, perturb_array=None, perturb_frame=0, skip_frame=0):

    optim_center_only = True
    if optim_center_only:
        cam_params.NO_CAMS = 1
        cam_params.CAM_PARAMS = [0,0]
        cam_params.CAM_NAMES = ["center"]
    else:
        cam_params.NO_CAMS = 3
        cam_params.CAM_PARAMS = [0,0,-1.5,0,1.5,0]
        cam_params.CAM_NAMES = ["center","left7","right5"]

    cam_params.CAM_PARAMS += [0,0]
    cam_params.CAM_NAMES += ["rear"]
    cam_params.NO_CAMS += 1

    scores = []
    score = 0

    if test_robustness:
        if perturb_array is None:
            perturb_array = np.random.rand(sim_params.NO_FRAMES)

        if perturb_frame > 0:
            # Determine the perturb sequence
            perturb_seq = ([False] * perturb_frame + [True] * skip_frame) * ((sim_params.NO_FRAMES // (perturb_frame + skip_frame) + 1))
            # Random value 0.5 correponds to 0 perturbation
            perturb_seq = perturb_seq[:len(perturb_array)]
            perturb_array[perturb_seq] = 0.5

    actor_list = []
    
    mpc = ModelPredictiveControl()
    mpc.horizon = horizon
    mpc.dt = dt

    mpc_data = SimulatorData(no_cameras=cam_params.NO_CAMS, cam_params=cam_params.CAM_PARAMS, cam_names=cam_params.CAM_NAMES)
    
    try:

        client = carla.Client('localhost', args.PORT)
        client.set_timeout(12.0)

        world = client.load_world("/Game/Carla/Maps/{}".format(sim_params.TOWN))
        print(world)

        original_settings = world.get_settings()
        settings = world.get_settings()

        settings.fixed_delta_seconds = sim_params.DT # 1 / FPS
        settings.synchronous_mode =  True
        world.apply_settings(settings)

        if sim_params.WEATHER != 'Default':
            weather_obj = get_preset_weather(sim_params.WEATHER)
            world.set_weather(weather_obj)
       
        port = 8312
        tm = client.get_trafficmanager(port)
        tm.set_random_device_seed(20)
        tm.set_synchronous_mode(True)    

        m = world.get_map()
        # We create the sensor queue in which we keep track of the information
        # already received. This structure is thread safe and can be
        # accessed by all the sensors callback concurrently without problem.
        sensor_queue = Queue()   

        # Blueprints that can be used for adding new actors into the simulation.
        blueprint_library = world.get_blueprint_library()
        bp = blueprint_library.filter('mustang')[0] if sim_params.DEFAULT_VEHICLE else random.choice(blueprint_library.filter('vehicle'))

        # Fix the starting position of the vehicle
        transform = world.get_map().get_spawn_points()[args.START_POS]

        # Spawn the vehicle
        vehicle = world.spawn_actor(bp, transform)
        actor_list.append(vehicle)
        print('created %s' % vehicle.type_id)

        ext_x = vehicle.bounding_box.extent.x
        ext_y = vehicle.bounding_box.extent.y
        width = ext_x * 2 if ext_x * 2 < 2.0 else ext_y * 2
        
        for cam in mpc_data.cams:
            if cam.name != "rear":
                camera_rgb, _, _ = set_camera(blueprint_library, world, vehicle, 
                                        x = cam_params.CAM_X, z = cam_params.CAM_Z,
                                        y=cam.y, yaw=cam.yaw,
                                        pitch=cam_params.CAM_PITCH, roll=0.,
                                        img_width=cam_params.IMG_WIDTH, img_height=cam_params.IMG_HEIGHT, fov=cam_params.IMG_FOV,
                                        sensor_type=cam_params.SENSOR_TYPE)

            else:
                # Rear camera
                camera_rgb, _, _ = set_camera(blueprint_library, world, vehicle, 
                                        x = cam_params.CAM_X-6.5, z = cam_params.CAM_Z+1.2,
                                        y=0, yaw=0,
                                        pitch=-30, roll=0.,
                                        img_width=cam_params.IMG_WIDTH, img_height=cam_params.IMG_HEIGHT, fov=cam_params.IMG_FOV,
                                        sensor_type=cam_params.SENSOR_TYPE)
            cam.obj = camera_rgb
            actor_list.append(camera_rgb)
            print('created %s' % camera_rgb.type_id)


        initial_frame = int(world.get_snapshot().frame)
        print("\nWorld's frame: %d" % initial_frame)
        
        mpc_data.cams[0].obj.listen(lambda d: sensor_callback(mpc_data, d, sensor_queue, mpc_data.cams[0].name, 0, args.OUT_PATH, args.EPISODE, initial_frame, sub_dir='mpc_online/', skip=sim_params.NO_SKIPPED_FRAMES))
        
        if cam_params.NO_CAMS >= 3:
            mpc_data.cams[1].obj.listen(lambda d: sensor_callback(mpc_data, d, sensor_queue, mpc_data.cams[1].name, 1, args.OUT_PATH, args.EPISODE, initial_frame, sub_dir='mpc_online/', skip=sim_params.NO_SKIPPED_FRAMES))
            mpc_data.cams[2].obj.listen(lambda d: sensor_callback(mpc_data, d, sensor_queue, mpc_data.cams[2].name, 2, args.OUT_PATH, args.EPISODE, initial_frame, sub_dir='mpc_online/', skip=sim_params.NO_SKIPPED_FRAMES))

        mpc_data.cams[cam_params.NO_CAMS-1].obj.listen(lambda d: sensor_callback(mpc_data, d, sensor_queue, mpc_data.cams[cam_params.NO_CAMS-1].name, cam_params.NO_CAMS-1, args.OUT_PATH, args.EPISODE, initial_frame, sub_dir='mpc_online/', skip=sim_params.NO_SKIPPED_FRAMES))

        x, y, z = 1.5, 0., 2.
        pitch, yaw, roll = 0., 0., 0.
        location = carla.Location(x=x, y=y, z=z)
        rotation = carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)
        sensor_transform = carla.Transform(location, rotation)

        collusion_bp = blueprint_library.find('sensor.other.collision')
        collusion_detector = world.spawn_actor(collusion_bp, sensor_transform, attach_to=vehicle)
        actor_list.append(collusion_detector)

        collusion = []
        collusion_detector.listen(lambda out: collusion.append(out))

        invasion_bp = blueprint_library.find('sensor.other.lane_invasion')
        invasion_detector = world.spawn_actor(invasion_bp, sensor_transform, attach_to=vehicle)
        actor_list.append(invasion_detector)

        invasion = []
        invasion_detector.listen(lambda out: invasion.append(out))


        i = 0
        f_frame = 1
        c_frame = 1
        steering = 0

        try:
            for frame in range(1, sim_params.NO_FRAMES+1):
                
                # Do tick
                world.tick()

                try:
                    for _ in range(cam_params.NO_CAMS):
                        s_frame = sensor_queue.get(True, 10.0)
                        print("    Frame: %d   Sensor: %s" % (s_frame[0], s_frame[1]))

                except Empty:
                    print("    Some of the sensor information is missed")

                # Get center camera
                cam = data.cams[0]

                initial_state = list(mpc_data.cams[0].initial_state.values())[-1]
                psi = initial_state[2]
                psi = np.radians((psi+360)%360)
                initial_state[2] = psi

                # Note: MPC configuration is being tested based on fixed velocity
                # v = 3.1 m/s
                initial_state[3] = 6
                initial_state_center = initial_state

                final_state, _, c_frame_updated, d = get_final_state(initial_state, c_frame, data, horizon=mpc_params.GT_SEARCH)

                interim_state = None
                interim_state2 = None

                final_state_ref = final_state
    
                if interim_cost:
                    interim_state, _, _, _ = get_final_state(initial_state, c_frame, data, horizon=3)

                psi_dist = np.abs(final_state[2] - initial_state[2])
                if psi_dist > np.pi:
                    psi_dist = np.abs(psi_dist - 2*np.pi)

                if psi_dist > np.radians(5):
                    interim_state, _, _, _ = get_final_state(initial_state, c_frame, data, horizon=3)
                    final_state = interim_state
                    interim_state = None
                    mpc.horizon = 2
                else:
                    interim_state = None
                    mpc.horizon = horizon
                
                c_frame = c_frame_updated

                for i, cam in enumerate(mpc_data.cams):
                    
                    # Apply mpc at online mode only for the center camera
                    if i != 0 and optim_center_only:
                        continue
                    elif cam.name == "rear":
                        continue

                    if i > 0:
                        initial_state = list(mpc_data.cams[i].initial_state.values())[-1]
                        psi = initial_state[2]
                        psi = np.radians((psi+360)%360)
                        initial_state[2] = psi
                        initial_state[3] = 6 # v
                    
                    
                    u, u_2, predict_info = calculate_u(mpc, initial_state, final_state, sim_total=1, show=False, interim_state=interim_state, interim_state2=interim_state2)
                          
                    throttle = 0.6 # 0.5
                    steering = np.mean(u_2.reshape(len(u_2)//2,2)[mpc_params.STEER,1])
                    
                    cam.initial_state[frame] = initial_state
                    cam.interim_state.append(final_state)
                    cam.final_state.append(final_state_ref)
                    cam.horizon.append(predict_info)
                    cam.steering.append(steering)
                    cam.throttle.append(throttle)

                initial_state = initial_state_center
                throttle = mpc_data.cams[0].throttle[-1]
                steering = mpc_data.cams[0].steering[-1]
                steering = 2 * steering / 2.444

                if test_robustness:

                    if perturb_array[frame] < 0.5 + 1e-4 and perturb_array[frame] > 0.5 - 1e-4:
                        print("SKIPPED")
                    # Range of perturbation, for 5% -> (-0.05,0.05) -> 0.1
                    perturb_range = perturb_percent * 0.01 * 2
                    
                    # Minimum perturbation in range -> -0.05
                    min_perturb   = - perturb_percent * 0.01
                    
                    # Random float between 0 and 1
                    perturbation = perturb_array[frame]
                    perturbation = (perturbation * perturb_range) + min_perturb

                    steering += perturbation


                # Apply control optimized with respect to center camera
                vehicle.apply_control(carla.VehicleControl(throttle=throttle, steer=steering))
                v = math.sqrt(vehicle.get_velocity().x**2 + vehicle.get_velocity().y**2 + vehicle.get_velocity().z**2)

                # Always have the traffic light on green
                if vehicle.is_at_traffic_light():
                    traffic_light = vehicle.get_traffic_light()
                    if traffic_light.get_state() == carla.TrafficLightState.Red:
                            traffic_light.set_state(carla.TrafficLightState.Green)

                gt_frame = search_ground_truth(initial_state, c_frame, data, horizon=10)
                gt_state = get_ground_truth_state(data, gt_frame)

                # Update current frame
                c_frame = gt_frame

                location = carla.Location()
                location.x = gt_state[0]
                location.y = gt_state[1]
                location.z = 2.0
                
                w = m.get_waypoint(location)
                _ = w.transform.location
                w_rot = w.transform.rotation

                psi = w_rot.yaw
                psi = np.radians((psi+360)%360)

                dist = get_distance(initial_state, gt_state, l1=False)
                print("Initial state:      ", initial_state)
                print("Ground-truth state: ", gt_state)
                print("Distance:           ", dist)

                lane_invasion_flag = False if dist <= (((w.lane_width - width) / 2.0)+0.15) else True
                frame_score = 0
                
                if len(collusion) > 0:
                    _ = collusion.pop()
                    if len(invasion) > 0:
                        _ = invasion.pop()
                    frame_score = 0
                elif len(invasion) > 0:
                    _ = invasion.pop()
                    frame_score = 0
                elif lane_invasion_flag:
                    frame_score = 0
                else:
                    frame_score = 1

                scores.append(frame_score)
                print("Frame score:        ", frame_score)
                
                mpc_data.pos.append((vehicle.get_location().x,vehicle.get_location().y, vehicle.get_location().z))
                mpc_data.psi.append((vehicle.get_transform().rotation.pitch, vehicle.get_transform().rotation.yaw, vehicle.get_transform().rotation.roll)) 
                
                mpc_data.speed.append(math.sqrt(vehicle.get_velocity().x**2 + vehicle.get_velocity().y**2 + vehicle.get_velocity().z**2))
                acc = vehicle.get_acceleration()
                mpc_data.acceleration.append((acc.x, acc.y, acc.z))
                mpc_data.manual_gear_shift.append(vehicle.get_control().manual_gear_shift)
                mpc_data.gear.append(vehicle.get_control().gear)
                mpc_data.hand_brake.append(vehicle.get_control().hand_brake)                

                print('Frame              : {}'.format(frame))
                print("Frame score        : {}\n".format(frame_score))

                print("Vehicle - Throttle: {}, Brake: {}, Steering: {} Gear: {}".format(vehicle.get_control().throttle, vehicle.get_control().brake, vehicle.get_control().steer, vehicle.get_control().gear))            
                print("Vehicle location  : (x,y,z): ({},{},{})".format(vehicle.get_location().x,vehicle.get_location().y, vehicle.get_location().z))
                print("Speed             : {}".format(v))


        except Exception as e:
            print(e)
            print("No more future states exist, exiting.")

    finally:
        score = np.mean(scores)
        print("SCORE: ", score)
        print('destroying actors')
        for cam in mpc_data.cams:
            cam.obj.destroy()
            cam.obj = None
            cam.images = []

        invasion_detector.destroy()
        collusion_detector.destroy()

        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])

        # Disable the sync mode always, before the script ends
        settings.synchronous_mode = False
        tm.set_synchronous_mode(False)
        world.apply_settings(original_settings)
        print('done.')

        pickle.dump(mpc_data, open(args.OUT_PATH + args.EPISODE + 'mpc_online/mpc_data.pkl', 'wb'))
        return score


if __name__ == '__main__':
    
    ap = arguments.make_parser()
    args = ap.parse_args()

    town_no = sim_params.TOWN[4:]

    cam_params.NO_CAMS = 3
    cam_params.CAM_NAMES =["center", "left", "right"]
    cam_params.CAM_PARAMS = [0, 0, -0.5, 0, 0.5, 0]

    # TEST CAMERA PARAMETER ARGS
    assert(cam_params.NO_CAMS * 2 == len(cam_params.CAM_PARAMS))
    assert(cam_params.NO_CAMS == len(cam_params.CAM_NAMES))

    args.EPISODE += "pos{}/".format(args.START_POS)

    os.system("rm -rf {}mpc_online/*".format(args.OUT_PATH + args.EPISODE))
    os.makedirs("{}mpc_online/".format(args.OUT_PATH + args.EPISODE), exist_ok=True)
    data = pickle.load(open(args.OUT_PATH + args.EPISODE + 'gt/data.pkl', 'rb'))
    
    score = moving_target(args, data, horizon=mpc_params.HORIZON, dt=mpc_params.MPC_DT, interim_cost=mpc_params.INTERIM_COST)