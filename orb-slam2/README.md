# ORB-SLAM2
**Authors:** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))

Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *IEEE Transactions on Robotics,* vol. 33, no. 5, pp. 1255-1262, 2017. **[PDF](https://128.84.21.199/pdf/1610.06475.pdf)**.

# 1. Prerequisites
The library has been tested in **Ubuntu 12.04**, **14.04** and **16.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

## C++11 or C++0x Compiler
New thread and chrono functionalities of C++11 are used.

## Pangolin
[Pangolin](https://github.com/stevenlovegrove/Pangolin) is used for visualization and user interface. Download and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
[OpenCV](http://opencv.org) is used to manipulate images and features. Download and install instructions can be found at: http://opencv.org. **Required at least 2.4.3. Tested with OpenCV 2.4.11 and OpenCV 3.2**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## DBoW2 and g2o (Included in Thirdparty folder)
Modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations are used. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

## Evaluation Script

Requires matplotlib, numpy, math, copy libraries.

# 2. Building ORB-SLAM2 library and examples

Clone the repository:
```
git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2
```

A script `build.sh` to build the *Thirdparty* libraries and *ORB-SLAM2* is provided. Please make sure you have installed all required dependencies (see section 1). Execute:
```
cd ORB_SLAM2
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM2.so** at *lib* folder and the executable **stereo_mpc** in *Examples* folder.


# 3. Stereo Examples

## MPC

Execute the following command. Change `mpc.yaml`to your yaml file. Change `PATH_TO_DATASET_FOLDER` to the uncompressed dataset folder. Change `SEQUENCE_NUMBER` to your sequence name. The sequence to run the SLAM for requires a `times.txt` in the same `PATH_TO_DATASET_FOLDER/Examples/Stereo/SEQUENCE_NUMBER` directory which contains the timestamps of the corresponding frames on each line.
```
./Examples/Stereo/stereo_mpc ./Vocabulary/ORBvoc.txt Examples/Stereo/mpc.yaml PATH_TO_DATASET_FOLDER/Examples/Stereo/SEQUENCE_NUMBER
```

## Evaluate SLAM predictions

Change `gt_poses_path` and `pred_poses_path` to your pose files. Execute the following command.

```
python eval_slam.py
```

# 4. License

ORB-SLAM2 is released under a [GPLv3 license](https://github.com/raulmur/ORB_SLAM2/blob/master/License-gpl.txt). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/raulmur/ORB_SLAM2/blob/master/Dependencies.md).


If you use ORB-SLAM2 (Monocular) in an academic work, please cite:

    @article{murTRO2015,
      title={{ORB-SLAM}: a Versatile and Accurate Monocular {SLAM} System},
      author={Mur-Artal, Ra\'ul, Montiel, J. M. M. and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={31},
      number={5},
      pages={1147--1163},
      doi = {10.1109/TRO.2015.2463671},
      year={2015}
     }

if you use ORB-SLAM2 (Stereo or RGB-D) in an academic work, please cite:

    @article{murORB2,
      title={{ORB-SLAM2}: an Open-Source {SLAM} System for Monocular, Stereo and {RGB-D} Cameras},
      author={Mur-Artal, Ra\'ul and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={33},
      number={5},
      pages={1255--1262},
      doi = {10.1109/TRO.2017.2705103},
      year={2017}
     }
