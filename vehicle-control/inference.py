"""
inference.py
"""

import numpy as np
import torch

from utils.sim_utils import *
from utils.data_utils import *
from config import CamParams, SimulatorParams
from data.simulator_data import *
from train.model import CNN
import arguments

cam_params = CamParams()
sim_params = SimulatorParams()


def apply_inference(args, data, inference_dir="inference/", provided_throttle=0.6):

    actor_list = []
    scores = []
    velocities = []
    score = 0

    inference_data = SimulatorData(no_cameras=2, cam_params=[0,0,0,0], cam_names=['center','rear'])
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    # Load the model
    model = CNN().double()
    model.load_state_dict(torch.load(args.MODEL_PATH))
    model.to(device)
    print("Model has been loaded.")

    model.eval()
    print(model)

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
        tm_port = tm.get_port()
        tm.set_random_device_seed(20)
        tm.set_synchronous_mode(True)  

        map = world.get_map()
        sensor_queue = Queue()

        # Blueprints that can be used for adding new actors into the simulation.
        blueprint_library = world.get_blueprint_library()
        bp = blueprint_library.filter('mustang')[0] if sim_params.DEFAULT_VEHICLE else random.choice(blueprint_library.filter('vehicle'))

        # Fix the starting position of the vehicle
        transform = map.get_spawn_points()[args.START_POS]

        # Spawn the vehicle
        vehicle = world.spawn_actor(bp, transform)
        actor_list.append(vehicle)
        print('created %s' % vehicle.type_id)

        vehicle.set_autopilot(False, port)

        ext_x = vehicle.bounding_box.extent.x
        ext_y = vehicle.bounding_box.extent.y
        width = ext_x * 2 if ext_x * 2 < 2.0 else ext_y * 2
        
        # Add RGB camera attached to the vehicle
        camera_rgb, _, _ = set_camera(blueprint_library, world, vehicle, 
                                    x = cam_params.CAM_X+0.5, z = cam_params.CAM_Z,
                                    y=0, yaw=0,
                                    pitch=cam_params.CAM_PITCH, roll=0.,
                                    img_width=cam_params.IMG_WIDTH, img_height=cam_params.IMG_HEIGHT, fov=cam_params.IMG_FOV,
                                    sensor_type=cam_params.SENSOR_TYPE)
        
        actor_list.append(camera_rgb)
        print('created %s' % camera_rgb.type_id)

        # Add rear camera
        camera_rear, _, _ = set_camera(blueprint_library, world, vehicle, 
                                    x = cam_params.CAM_X-6.5, z = cam_params.CAM_Z+1.2,
                                    y=0, yaw=0,
                                    pitch=-30, roll=0.,
                                    img_width=cam_params.IMG_WIDTH, img_height=cam_params.IMG_HEIGHT, fov=cam_params.IMG_FOV,
                                    sensor_type=cam_params.SENSOR_TYPE)
        
        actor_list.append(camera_rear)
        print('created %s' % camera_rear.type_id)

        initial_frame = int(world.get_snapshot().frame)
        print("\nWorld's frame: %d" % initial_frame)

        camera_rgb.listen(lambda d: sensor_callback(inference_data, d, sensor_queue, inference_data.cams[0].name, 0, args.OUT_PATH, args.EPISODE, initial_frame, sub_dir=inference_dir, skip=sim_params.NO_SKIPPED_FRAMES))
        camera_rear.listen(lambda d: sensor_callback(inference_data, d, sensor_queue, inference_data.cams[1].name, 1, args.OUT_PATH, args.EPISODE, initial_frame, sub_dir=inference_dir, skip=sim_params.NO_SKIPPED_FRAMES))
        
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

        throttle = provided_throttle
        steering = 0.0
        brake = 0.0
        c_frame = 1

        for frame in range(sim_params.NO_FRAMES):
            
            # Do tick
            world.tick()

            try:
                print("\nReceived frames    :")
                for _ in range(2):
                    s_frame = sensor_queue.get(True, 10.0)
                    print("    Frame: %d   Sensor: %s" % (s_frame[0], s_frame[1]))

            except Empty:
                print("    Some of the sensor information is missed")
            
            print()

            if frame>sim_params.NO_SKIPPED_FRAMES:
                
                key = s_frame[0] - initial_frame
                img_file = "{}{}{}center/{:06d}.png".format(args.OUT_PATH, args.EPISODE, inference_dir, key)
                img = cv2.imread(img_file)
                array = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

                x=50
                w=1000
                array = array[:, x:x+w] # Crop image

                dim = (128, 128)
                array = cv2.resize(array, dim) # Resize image
                
                array = np.transpose(array, (2,0,1))
                array = array.astype(np.float32)
                array = torch.from_numpy(array)
                array /= 255.0

                array = array.unsqueeze(0)
                array = array.double()
                array = array.to(device)

                w = map.get_waypoint(camera_rgb.get_location())

                pred = model(array)
                pred = pred.squeeze()
                pred = pred.detach().cpu().item()
                
                throttle = provided_throttle
                steering = pred
                steering = 2 * steering / 2.444

            else:
                steering = 0
                throttle = 0
            
            vehicle.apply_control(carla.VehicleControl(throttle=throttle, steer=steering, brake=brake))
            v = math.sqrt(vehicle.get_velocity().x**2 + vehicle.get_velocity().y**2 + vehicle.get_velocity().z**2)
            velocities.append(v)

            # Always have the traffic light on green
            if vehicle.is_at_traffic_light():
                traffic_light = vehicle.get_traffic_light()
                if traffic_light.get_state() == carla.TrafficLightState.Red:
                        traffic_light.set_state(carla.TrafficLightState.Green)

            initial_state = list(inference_data.cams[0].initial_state.values())[-1]
            psi = initial_state[2]
            psi = np.radians((psi+360)%360)
            initial_state[2] = psi

            gt_frame = search_ground_truth(initial_state, c_frame, data, horizon=10)
            gt_state = get_ground_truth_state(data, gt_frame)

            # Update current frame
            c_frame = gt_frame

            location = carla.Location()
            location.x = gt_state[0]
            location.y = gt_state[1]
            location.z = 2.0
            
            w = map.get_waypoint(location)
            w_loc = w.transform.location
            w_rot = w.transform.rotation

            psi = w_rot.yaw
            psi = np.radians((psi+360)%360)

            dist = get_distance(initial_state, gt_state, l1=False)
            print("Initial state      :", initial_state)
            print("Ground-truth state :", gt_state)
            print("Distance           :", dist)

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

            print('Frame              : {}'.format(frame))
            print("Frame score        : {}\n".format(frame_score))

            print("Vehicle - Throttle: {}, Brake: {}, Steering: {} Gear: {}".format(vehicle.get_control().throttle, vehicle.get_control().brake, vehicle.get_control().steer, vehicle.get_control().gear))            
            print("Vehicle location  : (x,y,z): ({},{},{})".format(vehicle.get_location().x,vehicle.get_location().y, vehicle.get_location().z))
            print("Speed             : {}".format(v))
    
    except Exception as e:
        print(e)

    finally:
        score = np.mean(scores)
        v = np.max(velocities) / 2.0
        
        print("SCORE: ", score)
        print('destroying actors')
        camera_rgb.destroy()
        camera_rear.destroy()
        invasion_detector.destroy()
        collusion_detector.destroy()

        # Disable the sync mode always, before the script ends
        settings.synchronous_mode = False
        tm.set_synchronous_mode(False)
        world.apply_settings(original_settings)
        
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        print('done.')
        return (score, v)


if __name__ == '__main__':

    ap = arguments.make_parser()
    args = ap.parse_args()

    town_no = sim_params.TOWN[4:]

    args.TRAJECTORY_NAME = "pos{}".format(args.START_POS)
    args.EPISODE += "{}/".format(args.TRAJECTORY_NAME)
    
    cmd = "rm -rf {}inference/*".format(args.OUT_PATH + args.EPISODE)
    print(cmd)
    os.system(cmd)

    # Test the model at inference time
    data = pickle.load(open(args.OUT_PATH + args.EPISODE + 'gt/data.pkl', 'rb'))
    _ = apply_inference(args, data, inference_dir="inference/")
