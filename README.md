# A Framework for Learning Vision Based Autonomous Lateral Vehicle Control without Explicit Supervision

This repository contains the code for the paper **A Framework for Learning Vision Based Autonomous
Lateral Vehicle Control without Explicit Supervision**.

![Overview of framework](/assets/images/overview-diagram.png)

<div align="center">
<a href="./assets/supplementary/7752_supplementary.pdf">Supplementary</a> | <a href="./assets/supplementary/video.mp4">Video</a>
</div>


## Installation

This repository requires Python 3.7 for interacting with CARLA 9.10, and uses `pip` for managing the packages. To create a `pip` virtual environment and load the necessary packages in order to execute this repository, please use the following command:

```
python3 -m venv env                 # Create virtual environment
source env/bin/activate             # Activate virtual environment
pip install -r requirements.txt     # Load packages from requirements.txt
```

Installation details regarding other related components are provided in their respective subdirectories.

## Running CARLA Simulator
We use CARLA simulator version 9.10, for data collection and training. For installation of CARLA 9.10, you can refer [here](https://github.com/carla-simulator/carla).

The simulator can be run by providing the following set of commands:

```
cd /PATH/TO/SIMULATOR/CARLA/CARLA_9.10/
DISPLAY= ./CarlaUE4.sh -opengl -carla-rpc-port=PORT_NO 
```

**Note:** If you are using a different `PORT_NO` rather than the default (i.e. 2000), make sure to provide this argument while running the scripts that require a connection to simulator under `vehicle-control/`.

### CARLA 9.10 Python API Binaries
The data collection and inference scripts in this repository requires a connection to the CARLA simulator. For this purpose, it is required to provide the path to the CARLA Python binaries. Therefore, please make sure to set an environment variable named `CARLA` as:

```
export CARLA="/PATH/TO/CARLA/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg
```

## Configuration

* We provide `config.py` for the configuration parameters that are used for data collection via CARLA simulator, MPC and training under `vehicle-control`.

* Additional parameters such as dataset or model paths are provided to the respective scripts as arguments using `arguments.py`.

* We use the following structure to record our dataset / mpc / inference images and trajectories:

```
└── data/                          <- Top dataset directory
    └── episode01/                 <- Corresponding CARLA episode sub-directory for each data collection
        └── pos47/
            ├── gt/                <- Sub-directory for ground truth data
            │   ├── center/        <- Frames collected from the center camera
            │   ├── left/          <- Frames collected from the left camera
            │   ├── right/         <- Frames collected from the right camera
            │   ├── ...     
            │   └── data.pkl       <- Pickled data.simulator_data class object, 
            │                             contains information such as vehicle position, orientation etc.
            ├── synthesized/       <- Sub-directory for synthesized image data
            │   ├── center/        <- Frames collected from the center camera
            │   ├── left/          <- Frames collected from the left camera
            │   ├── right/         <- Frames collected from the right camera
            │   ├── ...  
            │   └── data_slam.pkl     
            │           
            ├── mpc_online/        <- Sub-directory for online MPC data, used only for sanity-check
            │
            ├── mpc/
            │   ├── mpc_gt_data.pkl   <- Pickled data.simulator_data class object, 
            │   │                        contains trajectories with MPC applied to each camera position 
            │   │                        in ground-truth
            │   └── mpc_slam_data.pkl   <- Pickled data.simulator_data class object, 
            │                            trajectories with MPC applied to each camera position 
            │                            after SLAM / pose estimation
            │
            └── inference/         <- Recorded frames at test / inference time 
```

For the following steps, we mainly refer to the scripts under `vehicle-control/` directory, and refer to the additional components for the steps where they might be relevant.

```
cd vehicle-control/
```

## Data Collection

For collecting ground truth data with the default arguments, simply use the following command. `PORT_NO` is the number of port on which CARLA simulator is running, and is not required if the default port for CARLA simulator is being used.

```
python3 collect_data.py --START_POS 214 --PORT PORT_NO
```

**Optional Arguments:**

```
--START_POS     # Starting position in CARLA Town01
                # We consider 108, 152, 52, 47, 178, 187, 208, 214
--EPISODE       # Subfolder under dataset to save trajectories & images
--OUT_PATH      # Output directory, where to construct the dataset
```

By default, our configuration collects data to enable all configurations of our pipeline. Therefore, one can select to train the neural network with `GT Trajectory` or `VO (Visual Odometry)`, as well as `GT Images` or `Synthesized`.

## Model Predictive Control (MPC)

MPC helps us to retrieve steering angles with respect to all camera positions. If `GT Trajectory` will be used for training, then one can directly execute MPC across the data collected from the simulator. 

If it is desired to train our `VO + Synthesized` method, then it is required to first execute the visual odometry component. For this, please refer to [here](orb-slam2/README.md). This will output the trajectories for each camera position. In order to bring them into the required data format to execute MPC, we provide [this](orb-slam2/pose_processing.py) script.

### Model Predictive Control - MPC (Offline mode / without running CARLA Simulator)

For applying MPC on the collected data in offline mode in order to obtain the final dataset for training, simply use:

```
python3 mpc/mpc.py --START_POS 47
```

**Optional Arguments:**

```
--GT True    # Whether to execute MPC directly on the data collected 
             # from the CARLA simulator (i.e. ground-truth trajectories), 
             # or on the estimated / SLAM trajectories
```

This script will create a subfolder named `mpc/` under the desired dataset path that contains the estimated trajectory for each camera position.

**Note:** It is required to have mpc executed for all trajectories (i.e. starting positions) that will be used upon training.

### Online MPC (while running CARLA Simulator)

In order to test the current parameter configuration of MPC on the CARLA Simulator, we additionally provide the `mpc_online.py` script which directly interacts with the simulator using Model Predictive Control (MPC). Note that this script is only used for sanity-check purposes.

To execute online MPC, the following command can be used:

```
python3 mpc_online.py --START_POS 47
```



## Training

For training the model, we require a sequence of RGB images as input and the steering labels estimated by MPC as their target. The training can be performed with the `GT images` collected directly from the simulator. 

If it is desired to train our `VO + Synthesized` method, then one should first execute the view synthesis component. This component will provide the `Synthesized images` across multiple camera positions lateral to the vehicle. The details regarding the use of this component are provided [here](view-synthesis/README.md).

Note that for our qualitative evaluation on KITTI dataset, we also make use of this view synthesis component in order to obtain the `Synthesized images`.

For displaying the model architecture with `torchsummary`, use the following command:

```
cd train/
python3 model.py
```

For training the model, the following command can be provided:

```
python3 train.py
```

**Optional Arguments:**

```
--DATASET_PATH                      # Path to dataset, e.g. out/
--WEIGHTED_SAMPLING True            # Whether to use weighted sampling method for oversampling the dataset

--MODEL_SAVE_DIR train/models       # The directory to save models in
--EXPERIMENT_DIR train/experiments  # The directory to save experiments in

--HYPERPARAM_EXP_NAME default_exp   # Name of the hyperparameter experiment

```

Training results are recorded on `tensorboard` and can be investigated by using the following command:

```
tensorboard --logdir=train/experiments/
```

The `log_dir` argument should be changed to the corresponding directory, if another directory is provided with `--EXPERIMENT_DIR` rather than the default.
 
## Inference

For testing the model trained by using the commands provided above at inference time, one can use:

```
python3 inference.py
```

The inference script calculates a score for the ratio of time the car remains in driving lane. In order to calculate this score, we require the ground-truth trajectory for the corresponding starting position to be collected and present under the respective directory structure. We provide `pos47` from CARLA Town 1 as a sample trajectory under the top-level `data/episode01/` directory.

**Optional Arguments:**

```
--START_POS     # Starting position in CARLA Town01
                # We consider 108, 152, 52, 47, 178, 187, 208, 214
--MODEL_PATH    # Path to model, e.g. train/models/ours.pth
```

**Note:** CARLA simulator should be up and running in order to test the models using inference script. The port number of the simulator should match with `PORT_NO` argument, if another port number is being used rather than the default one.

## Citation

If you find the repository useful, please cite us as
```bibtex
@article{khan2022vehiclecontrol,
 title  = {A Framework for Learning Vision Based Autonomous Lateral Vehicle Control without Explicit Supervision},
 author = {Qadeer Khan, Idil Sülo, Melis Ocal and Daniel Cremers},
 year   = {2022},
}
```
