"""
simulator_data.py

* Data class to collect simulator data
"""


class CameraData:
    def __init__(self, y=0, yaw=0, name="center"):
        self.throttle = []
        self.steering = []
        self.brake = []
        self.name = name
        self.y = y
        self.yaw = yaw
        self.obj = None
        self.images= []

        # Needed for visualization purposes
        self.initial_state = {}     # Initial state of the camera used for computing MPC : [x, y, psi, v]
        self.interim_state = []
        self.final_state = []       # Reference state used for computing MPC : [x, y, psi, v] 
        self.horizon = []           # Planted states through the horizon: [x, y, psi, v]                    


class SimulatorData:
    def __init__(self, no_cameras=3, cam_params=[-1.5, -45, 0, 0, 1.5, 45], cam_names=["left", "center", "right"]):
        
        self.pos = []                 # Position of the vehicle    : (x, y, z)
        self.psi = []                 # Orientation of the vehicle : (pitch, yaw, roll)
        self.speed = []               # Speed of the vehicle
        self.acceleration = []        # Acceleration of the vehicle    : (x, y, z)
        self.manual_gear_shift = []   # Determines whether the vehicle will be controlled by changing gears manually. Default is False.
        self.gear = []                # States which gear is the vehicle running on. 
        self.hand_brake = []          # Determines whether hand brake will be used. Default is False. 

        self.cams = []                # Holds the CameraData objects for each camera attached to the vehicle
        self.no_cameras = no_cameras  # Number of cameras attached to the vehicle
        self.cam_params = cam_params  # Parameters for each camera: [y_0, yaw_0, y_1, yaw_1, ...]


        for i in range(self.no_cameras):
            cam = CameraData(y=cam_params[2*i], yaw=cam_params[2*i+1], name=cam_names[i])
            self.cams.append(cam)

    def __len__(self):
        return len(self.cams[0].steering)

