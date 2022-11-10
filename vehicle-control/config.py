"""
config.py
"""

class CamParams:
    # CAM CONFIG
    NO_CAMS = 12 # Number of cameras to place on vehicle.
    CAM_NAMES = ["center", "left", "right", "left2", "right2", "left3", "right3", 
                 "left4", "right4", "left5", "right5", "right6"]
    CAM_PARAMS = [0, 0, -0.15, 0, 0.15, 0, -0.30, 0, 
                  0.30, 0, -0.45, 0, 0.45, 0, -0.60, 
                  0, 0.60, 0, -0.75, 0, 0.75, 0, 0.5, 0] # y and yaw values for the cameras.

    CAM_PITCH = 0 # Pitch value of camera, in degrees. Same for all cameras.
    CAM_X = 1.5   # X value of camera, relative to the vehicle. Same for all cameras.
    CAM_Z = 2     # Z value of camera, relative to the vehicle. Same for all cameras.

    # IMAGE CONFIG
    IMG_WIDTH = 1200 # Width of the image frames.
    IMG_HEIGHT = 600 # Height of the image frames.
    IMG_FOV = 110    # Field of view of the image frames.
    SENSOR_TYPE = "sensor.camera.rgb" # Type of the sensors used for data collection.
    GAN = False # Whether to collect data for GAN


class SimulatorParams:
    # FRAME / WORLD TICK CONFIG
    DT = 0.2        # Time difference of the simulator, 1/FPS
    NO_FRAMES = 200 # Number of frames to collect in total.
    NO_SKIPPED_FRAMES = 25 # Number of frames to skip.
    NO_FRAMES += NO_SKIPPED_FRAMES

    # WEATHER / VEHICLE CONFIG
    WEATHER = "Default" # Weather choice for the simulator.
    DEFAULT_VEHICLE = True # Whether to use the default vehicle or a random one.

    # TOWN / START POS CONFIG
    TOWN = "Town01" # Town choice of the simulator
    EPISODE_POS = ['pos108', 'pos152', 'pos52', 'pos47', 'pos178', 'pos187', 'pos208', 'pos214']


class InferenceParams:
    PERTURBATION = False # Whether to perform perturbation test for inference step.
    PERTURB_PERCENT = 0  # Percentage of perturbation to apply at inference step.


class MPCParams:
    INTERIM_COST = True # Whether to compute the cost together with an intermediate state 
                        # that is located half the distance to the reference state.
    GT_SEARCH = 13      # Ground truth search value of the MPC (in meters).
						# This will ensure that there is always a fixed distance between the 
                        # initial and the final state.
    HORIZON = 10        # Horizon value of the MPC.
    MPC_DT = 0.2        # Time difference of the MPC.
    STEER = [0]         # Indices for steering angle values to consider from the predicted 
                        # steering command sequence.


class TrainParams:
    # IMG / TRAJECTORY TYPE (GT / SYNTHESIZED)
    GT_TRAJECTORY = True    # Whether to use ground-truth trajectory (with MPC executed).
    GT_IMG = True           # Whether to use ground-truth images.
    NO_CAMS = 11            # Number of cameras at each step for gt + synthesized 
    EPISODE_POS = ['pos47'] # Episode positions to consider for training

    # TRAIN
    LEARNING_RATE = 1e-4 # Learning rate.
    WS = True            # Whether to use weighted sampling during training
    NO_EPOCHS = 100      # Number of epochs.
    BATCH_SIZE = 300     # Size of each mini-batch.
    LOSS = 'L1'          # Type of loss function to use for training: MSE or L1
    

    # GPU ACCELARATED TRAINING
    DEVICE = None # Device to perform the training on:
			      #  - If not specified, GPU will be used if exists.
				  #  - Otherwise, specified option will be used.
