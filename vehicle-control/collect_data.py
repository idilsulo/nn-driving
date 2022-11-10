import arguments
from config import CamParams, SimulatorParams
from utils.sim_utils import *
from data.simulator_data import *

cam_params = CamParams()
sim_params = SimulatorParams()


def collect_ground_truth(args):
    """Collects images, steering, throttle and position values for the ground truth trajectory
    
    Args:
        args: commandline arguments

    Returns:
        data (SimulatorData): collected simulator data
    """

    actor_list = []
    data = SimulatorData(no_cameras=cam_params.NO_CAMS, cam_params=cam_params.CAM_PARAMS, cam_names=cam_params.CAM_NAMES)
    N = 0

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
        
        port = 8012
        tm = client.get_trafficmanager(port)
        tm.set_random_device_seed(20)
        tm.set_synchronous_mode(True)

        # We create the sensor queue in which we keep track of the information
        # already received. This structure is thread safe and can be
        # accessed by all the sensors callback concurrently without problem.
        sensor_queue = Queue()

        # Blueprints that can be used for adding new actors into the simulation.
        blueprint_library = world.get_blueprint_library()
        bp = blueprint_library.filter('mustang')[0] if sim_params.DEFAULT_VEHICLE else random.choice(blueprint_library.filter('vehicle'))

        # Fix the starting position of the vehicle
        map = world.get_map()
        transform = map.get_spawn_points()[args.START_POS]

        # Spawn the vehicle
        vehicle = world.spawn_actor(bp, transform)
        actor_list.append(vehicle)
        print('created %s' % vehicle.type_id)

        # Let's put the vehicle to drive around.
        vehicle.set_autopilot(True, port)

        frame = 0

        # Add RGB cameras attached to the vehicle
        # Camera positioned at the center represents ground truth trajectory
        for cam in data.cams:

            if not cam_params.GAN:
                camera_rgb, _, _ = set_camera(blueprint_library, world, vehicle, 
                                        x = cam_params.CAM_X, z = cam_params.CAM_Z,
                                        y=cam.y, yaw=cam.yaw,
                                        pitch=cam_params.CAM_PITCH, roll=0.,
                                        img_width=cam_params.IMG_WIDTH, img_height=cam_params.IMG_HEIGHT, fov=cam_params.IMG_FOV,
                                        sensor_type=cam_params.SENSOR_TYPE)
            else:
                camera_rgb, _, _ = set_camera(blueprint_library, world, vehicle, 
                                        x = cam_params.CAM_X, z = cam_params.CAM_Z,
                                        y=cam.y, yaw=cam.yaw,
                                        pitch=cam_params.CAM_PITCH, roll=0.,
                                        img_width=cam_params.IMG_WIDTH-200, img_height=cam_params.IMG_HEIGHT, fov=99.922751242647,
                                        sensor_type=cam_params.SENSOR_TYPE)

            cam.obj = camera_rgb
            actor_list.append(camera_rgb)
            print('created %s' % camera_rgb.type_id)

        initial_frame = int(world.get_snapshot().frame)
        print("\nWorld's frame: %d" % initial_frame)

        data.cams[0].obj.listen(lambda d: sensor_callback(data, d, sensor_queue, data.cams[0].name, 0, args.OUT_PATH, args.EPISODE, initial_frame, skip=sim_params.NO_SKIPPED_FRAMES, gan=cam_params.GAN))
        
        if cam_params.NO_CAMS >= 2:
            data.cams[1].obj.listen(lambda d: sensor_callback(data, d, sensor_queue, data.cams[1].name, 1, args.OUT_PATH, args.EPISODE, initial_frame, skip=sim_params.NO_SKIPPED_FRAMES, gan=cam_params.GAN))
        
        if cam_params.NO_CAMS >= 3:
            data.cams[2].obj.listen(lambda d: sensor_callback(data, d, sensor_queue, data.cams[2].name, 2, args.OUT_PATH, args.EPISODE, initial_frame, skip=sim_params.NO_SKIPPED_FRAMES, gan=cam_params.GAN))

        if cam_params.NO_CAMS >= 5:
            data.cams[3].obj.listen(lambda d: sensor_callback(data, d, sensor_queue, data.cams[3].name, 3, args.OUT_PATH, args.EPISODE, initial_frame, skip=sim_params.NO_SKIPPED_FRAMES, gan=cam_params.GAN))
            data.cams[4].obj.listen(lambda d: sensor_callback(data, d, sensor_queue, data.cams[4].name, 4, args.OUT_PATH, args.EPISODE, initial_frame, skip=sim_params.NO_SKIPPED_FRAMES, gan=cam_params.GAN))
        
        if cam_params.NO_CAMS >= 7:
            data.cams[5].obj.listen(lambda d: sensor_callback(data, d, sensor_queue, data.cams[5].name, 5, args.OUT_PATH, args.EPISODE, initial_frame, skip=sim_params.NO_SKIPPED_FRAMES, gan=cam_params.GAN))
            data.cams[6].obj.listen(lambda d: sensor_callback(data, d, sensor_queue, data.cams[6].name, 6, args.OUT_PATH, args.EPISODE, initial_frame, skip=sim_params.NO_SKIPPED_FRAMES, gan=cam_params.GAN))

        if cam_params.NO_CAMS >= 9:
            data.cams[7].obj.listen(lambda d: sensor_callback(data, d, sensor_queue, data.cams[7].name, 7, args.OUT_PATH, args.EPISODE, initial_frame, skip=sim_params.NO_SKIPPED_FRAMES, gan=cam_params.GAN))
            data.cams[8].obj.listen(lambda d: sensor_callback(data, d, sensor_queue, data.cams[8].name, 8, args.OUT_PATH, args.EPISODE, initial_frame, skip=sim_params.NO_SKIPPED_FRAMES, gan=cam_params.GAN))

        if cam_params.NO_CAMS >= 11:
            data.cams[9].obj.listen(lambda d: sensor_callback(data, d, sensor_queue, data.cams[9].name, 9, args.OUT_PATH, args.EPISODE, initial_frame, skip=sim_params.NO_SKIPPED_FRAMES, gan=cam_params.GAN))
            data.cams[10].obj.listen(lambda d: sensor_callback(data, d, sensor_queue, data.cams[10].name, 10, args.OUT_PATH, args.EPISODE, initial_frame, skip=sim_params.NO_SKIPPED_FRAMES, gan=cam_params.GAN))

        if cam_params.NO_CAMS == 12:
            data.cams[11].obj.listen(lambda d: sensor_callback(data, d, sensor_queue, data.cams[11].name, 11, args.OUT_PATH, args.EPISODE, initial_frame, skip=sim_params.NO_SKIPPED_FRAMES, gan=cam_params.GAN))

        t = 0

        for frame in range(sim_params.NO_FRAMES):

            # Do tick
            world.tick()
            
            # Now, we wait to the sensors data to be received.
            # As the queue is blocking, we will wait in the queue.get() methods
            # until all the information is processed and we continue with the next frame.
            # We include a timeout of 1.0 s (in the get method) and if some information is
            # not received in this time we continue.
            try:
                for _ in range(cam_params.NO_CAMS):
                    s_frame = sensor_queue.get(True, 10.0)
                    print("    Frame: %d   Sensor: %s" % (s_frame[0], s_frame[1]))

            except Empty:
                print("    Some of the sensor information is missed")

            # Always have the traffic light on green
            if vehicle.is_at_traffic_light():
                traffic_light = vehicle.get_traffic_light()
                if traffic_light.get_state() == carla.TrafficLightState.Red:
                        traffic_light.set_state(carla.TrafficLightState.Green)
            
            print('Frame: %s' % frame)
            w_frame = int(world.get_snapshot().frame)
            print("\nWorld's frame: %d" % w_frame)

            # Center camera at index 0
            data.cams[0].throttle.append(vehicle.get_control().throttle)
            data.cams[0].brake.append(vehicle.get_control().brake)
            data.cams[0].steering.append(vehicle.get_control().steer)
            
            data.pos.append((vehicle.get_location().x,vehicle.get_location().y, vehicle.get_location().z))
            data.psi.append((vehicle.get_transform().rotation.pitch, vehicle.get_transform().rotation.yaw, vehicle.get_transform().rotation.roll)) 
                        
            # Approach below is used in https://github.com/carla-simulator/carla/blob/master/PythonAPI/examples/automatic_control.py
            v = math.sqrt(vehicle.get_velocity().x**2 + vehicle.get_velocity().y**2 + vehicle.get_velocity().z**2)
            data.speed.append(v)

            acc = vehicle.get_acceleration()
            data.acceleration.append((acc.x, acc.y, acc.z))
            data.manual_gear_shift.append(vehicle.get_control().manual_gear_shift)
            data.gear.append(vehicle.get_control().gear)
            data.hand_brake.append(vehicle.get_control().hand_brake)

            print("Throttle: {}, Steering: {}, Brake: {}".format(data.cams[0].throttle[-1], data.cams[0].steering[-1], data.cams[0].brake[-1]))
            print("Vehicle location: (x,y,z): ({},{},{})".format(data.pos[-1][0], data.pos[-1][1], data.pos[-1][2]))
            print("Acceleration: {}, Speed: {}".format(data.acceleration[-1], data.speed[-1]))

    except Exception as e:
        print(e)

    finally:
        print('destroying actors')
        for cam in data.cams:
            cam.obj.destroy()
            cam.obj = None
            cam.images = []

        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])

        # Disable the sync mode always, before the script ends
        settings.synchronous_mode = False
        tm.set_synchronous_mode(False)
        world.apply_settings(original_settings)
        print('done.')
        
        pickle.dump(data, open(args.OUT_PATH + args.EPISODE + 'gt/data.pkl', 'wb'))
        return data


if __name__ == '__main__':
    
    ap = arguments.make_parser()
    args = ap.parse_args()

    town_no = sim_params.TOWN[4:]

    # TEST CAMERA PARAMETER ARGS
    assert(cam_params.NO_CAMS * 2 == len(cam_params.CAM_PARAMS))
    assert(cam_params.NO_CAMS == len(cam_params.CAM_NAMES))

    # Collect data from the simulator
    args.EPISODE += "pos{}/".format(args.START_POS)

    os.makedirs("{}/{}".format(args.OUT_PATH, args.EPISODE), exist_ok=True)
    
    data = collect_ground_truth(args)
