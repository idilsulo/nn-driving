"""
utils.py

* Utilities for manipulating CARLA simulator server
"""

import glob
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import cv2
from torchvision import transforms
from queue import Queue
from queue import Empty
import random
import json
import re
import torch
import time
import math
import pickle

try:
    sys.path.append(glob.glob(os.environ['CARLA'])[0])
except IndexError:
    pass

import carla


from data.simulator_data import *

torch.manual_seed(42)

def visualize_image(image):
    data = np.array(image.raw_data) # shape is (image.height * image.width * 4,) 
    data_reshaped = np.reshape(data, (image.height, image.width,4))
    rgb_3channels = data_reshaped[:,:,:3] # first 3 channels
    
    cv2.imshow("image",rgb_3channels )
    cv2.waitKey(10)

def set_camera(blueprint_library, world, vehicle, x=1.5, y=0., z=2,
                    pitch=0., yaw=0., roll=0.,
                    # img_width=512, img_height=512, fov=90)
                    img_width=1200, img_height=600, fov=110, sensor_type='sensor.camera.rgb'):
    """
    * Sets the camera based on provided rotation and location values
    * x, y, z (in meters)
    * pitch, yaw, roll (in degrees)
    """
    camera_bp_rgb = blueprint_library.find(sensor_type)
    camera_bp_rgb.set_attribute('image_size_x',  str(img_width))
    camera_bp_rgb.set_attribute('image_size_y',  str(img_height))
    camera_bp_rgb.set_attribute('fov',  str(fov))

    # Warning: The declaration order is different in CARLA (pitch,yaw,roll), 
    # and in the Unreal Engine Editor (roll,pitch,yaw). 
    # When working in a build from source, don't mix up the axes' rotations. 
    # One may consider the field of view while setting rotation as well.
    location = carla.Location(x=x, y=y, z=z)
    rotation = carla.Rotation(pitch=pitch, yaw=yaw, roll=roll)
    
    # Since version 0.8.0 the positions of the sensors are specified in meters instead of centimeters. 
    # Always relative to the vehicle.
    # Therefore, transform is relative to the vehicle
    camera_transform_rgb = carla.Transform(location, rotation)
    camera_rgb = world.spawn_actor(camera_bp_rgb, camera_transform_rgb, attach_to=vehicle)
    return camera_rgb, location, rotation


def record_camera_intrinsic(image_w=512.0, image_h=512.0, fov=90.0):

    # Build the K projection matrix:
    # K = [[Fx,  0, image_w/2],
    #      [ 0, Fy, image_h/2],
    #      [ 0,  0,         1]]

    focal = image_w / (2.0 * np.tan(fov * np.pi / 360.0))

    # In this case Fx and Fy are the same since the pixel aspect
    # ratio is 1
    K = np.identity(3)
    K[0, 0] = K[1, 1] = focal
    K[0, 2] = image_w / 2.0
    K[1, 2] = image_h / 2.0

    json_str = json.dumps(K.tolist(), indent=4)
    json_file = open("camera_intrinsic.json", "w")
    json_file.write(json_str)
    json_file.close()


# Sensor callback, called when new sensor data is present
def sensor_callback(data, sensor_data, sensor_queue, sensor_name, cam_id, out_path, episode, 
                    initial_frame, sub_dir='gt/', skip=10, gan=False):
    
    key = sensor_data.frame - initial_frame
    if key > skip: sensor_data.save_to_disk(out_path + episode + sub_dir + sensor_name + '/' + '%06d.png' % (key))

    if key > skip and gan: 
        img = cv2.imread(out_path + episode + sub_dir + sensor_name + '/' + '%06d.png' % (key))
        rgb_img = cv2.resize(img, (128,128))
        cv2.imwrite(out_path + episode + sub_dir + sensor_name + '/' + '%06d.png' % (key), cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR))

    x = sensor_data.transform.location.x
    y = sensor_data.transform.location.y
    psi = sensor_data.transform.rotation.yaw
    v = 6 # Keep the velocity constant, it will be overwritten by mpc

    data.cams[cam_id].initial_state[key] = [x, y, psi, v] 
    sensor_queue.put((sensor_data.frame, sensor_name))


def find_weather_presets():
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), x) for x in presets]


def get_preset_weather(weather_name):
    # Print presets to display available options
    presets = find_weather_presets()
    print(presets)
    for w_obj, w_name in presets:
        if w_name == weather_name:
            return w_obj


def consecutive(data, stepsize=0):
    return np.split(data, np.where(np.diff(data) != stepsize)[0]+1)


def mean_velocity(scores, velocities):
    final_group = consecutive(scores,0)[-1]
    if final_group[0] == 0:
        v = np.mean(velocities[:-len(final_group)])
    else:
        v = np.mean(velocities)
    return v

