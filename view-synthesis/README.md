# Monocular Depth Estimation

The following method is used for monocular depth estimation required for our view synthesis component:

> **Digging into Self-Supervised Monocular Depth Prediction**
>
> [Clément Godard](http://www0.cs.ucl.ac.uk/staff/C.Godard/), [Oisin Mac Aodha](http://vision.caltech.edu/~macaodha/), [Michael Firman](http://www.michaelfirman.co.uk) and [Gabriel J. Brostow](http://www0.cs.ucl.ac.uk/staff/g.brostow/)
>
> [ICCV 2019 (arXiv pdf)](https://arxiv.org/abs/1806.01260)

## Modifications

Please see the patch file for all the modifications applied.

Please cite the following paper:

```
@article{monodepth2,
  title     = {Digging into Self-Supervised Monocular Depth Prediction},
  author    = {Cl{\'{e}}ment Godard and
               Oisin {Mac Aodha} and
               Michael Firman and
               Gabriel J. Brostow},
  booktitle = {The International Conference on Computer Vision (ICCV)},
  month = {October},
year = {2019}
}
```


## License
Copyright © Niantic, Inc. 2019. Patent Pending.
All rights reserved.
Please see the [license file](LICENSE) for terms.


# Image Rendering Script

## Prerequisites

The code is tested with Python 3.7.7.

### PyTorch3D

Download and install instructions can be found at: https://github.com/facebookresearch/pytorch3d/blob/master/INSTALL.md **Current version is 0.4.0.**

### Other Dependencies

Information regarding current pytorch version and other Python libraries can be found in the requirements.txt.

## Rendering

    1. Change the following paths to your own directories: FOLDER_mpc_rgb, FOLDER_mpc_disp, and FOLDER_out.
        FOLDER_mpc_rgb: RGB image folder
        FOLDER_mpc_disp: Disparity map folder
        FOLDER_out: Output folder

    2. The following variables can be adjusted for your own image synthesis:
        `id_list_pos`: List of folders to be processed by the script
        `cam_rgb` and `cam_disp`: As the current script uses just the center camera for synthesis, these variables can be changed to other cameras to synthesize from other point of views.
        `translation_arr`: The translation distances in the horizontal direction
        `save_path`: Path to save the rendered images

    3. Run the jupyter notebook after executing the following command if you are using commandline:
        jupyter notebook


## Directory Structure

Currently only the center camera is used for image rendering.

### Input Directory

```
├── FOLDER_mpc_disp
│   ├── episode_name  
│       ├── disp_center
│       ├── disp_left
│       ├── disp_right
│       ├── ...
│       ├── disp_leftX
│       ├── disp_rightX
│           ├── 00000.npy                     
│           ├── 00001.npy 
│           └── ...    
├── FOLDER_mpc_rgb
│   ├── episode_name    
│       ├── ...          
│       ├── gt    
│           ├── center
│           ├── left
│           ├── right
│           ├── ...
│           ├── leftX
│           ├── rightX   
│                ├── 00000.png                      
│                ├── 00001.png 
└──              └── ...           
```

### Output Directory

```
├── FOLDER_out
│   ├── episode_name           
│       ├── left
│       ├── right 
│       ├── ...
│       ├── leftX
│       ├── rightX   
│            ├── 00000.png                      
│            ├── 00001.png   
└──          └── ...              
```
