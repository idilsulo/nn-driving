## Installation

* This repository requires Python 3.7 for interacting with CARLA 9.10, and uses `pip` for managing the packages. To create a `pip` virtual environment and load the necessary packages in order to execute this repository, please use the following command:

```
python3 -m venv env                 # Create virtual environment
source env/bin/activate             # Activate virtual environment
pip install -r requirements.txt     # Load packages from requirements.txt
```

* There are several visualiztion scripts that are being used in this project. If you are connecting to the remote machines via SSH connection and your local machine is using OS X, one suggestion is to use **XQuartz**  which forwards the GUI windows that are opened on the remote machine to the local machine. For further information: https://www.xquartz.org/index.html

## CARLA 9.10 Python API Binaries
In order to collect data and test the trained models, we use CARLA simulator with version 9.10. For the scripts in this repository to connect to the simulator, please make sure to set an environment variable named `CARLA` as:

```
export CARLA="/PATH/TO/CARLA/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg
```


## Configuration

* We provide `config.py` for the configuration parameters that are used for data collection via CARLA simulator, MPC and training.

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
            │   └── ...     
            │           
            ├── mpc_online/        <- Sub-directory for online MPC data, used only for sanity-check
            │
            ├── mpc/
            │   ├── mpc_data.pkl   <- Pickled data.simulator_data class object, 
            │   │                        contains trajectories with MPC applied to each camera position 
            │   │                        in ground-truth
            │   └── mpc_data.pkl   <- Pickled data.simulator_data class object, 
            │                            trajectories with MPC applied to each camera position 
            │                            after SLAM / pose estimation
            │
            └── inference/         <- Recorded frames at test / inference time 
```

## Data Collection

* For collecting ground truth data with the default arguments, simply use the following command. `PORT_NO` is the number of port on which CARLA simulator is running, and is not required if the default port for CARLA simulator (2000) is being used.

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


## Model Predictive Control (MPC)


### Model Predictive Control - MPC (Offline MPC / without running CARLA Simulator)

* For applying MPC on the collected data in offline mode in order to obtain the final dataset for training, simply use:

```
python3 mpc/mpc.py --START_POS 47
```

**Optional Arguments:**

```
--GT True    # MPC can either be executed directly on the data collected from the CARLA simulator (i.e. ground-truth data), or on the estimated / SLAM trajectories
```


* This script will create a subfolder named `mpc/` under the desired dataset path that contains the estimated trajectory for each camera position.
* **Note:** It is required to have mpc executed for all trajectories that will be used upon training.

### MPC on CARLA Simulator (while running CARLA Simulator / online mode)

* In order to test the current parameter configuration of MPC on the CARLA Simulator, we additionally provide the `mpc_online` script which directly interacts with the simulator using Model Predictive Control (MPC). For this, following command can be provided:

```
python3 mpc_online.py --START_POS 47
```

<!-- 
### MPC Online Mode Visualization

![MPC Online Mode Visualization](/assets/mpc-online-view.png)

* It is possible to visualize the online MPC results by providing the following command. This will display the bird's eye-view plot and the image frames collected from the rear camera:

```
cd vis/
python3 vis.py --MPC True --ONLINE True
``` -->

## Training

* For displaying the recent model architecture with `torchsummary`, use the following command:

```
cd train/
python3 model.py
```

* For training the model, use the following command:

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

* Training results are recorded on `tensorboard` and can be investigated by providing the following command:

```
tensorboard --logdir=train/experiments/
```

* Note that the `log_dir` should be changed to the corresponding directory, if another directory is provided with `--EXPERIMENT_DIR` rather than the default.

* After providing the desired hyperparameters, the experiments will be run with all combinations of the provided hyperparameters. 

## Inference

* For testing the model trained by using the commands provided above at inference time, use the following command:

```
python3 inference.py
```

* CARLA simulator should be up and running in order to test the models using inference script. The port number of the simulator should match with `PORT_NO` argument, if another port number is being used rather than the default one.

* **Note:** The inference script calculates a score for the ratio of time the car remains in driving lane. For this, we require the ground-truth trajectory for the corresponding starting position. We provide `pos47` as a sample trajectory under the top-level `data/` directory.

**Optional Arguments:**

```
--START_POS     # Starting position in CARLA Town01
                # We consider 108, 152, 52, 47, 178, 187, 208, 214
--MODEL_PATH       # Path to model, e.g. train/models/ours.pth
```
