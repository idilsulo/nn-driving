"""
dataloader.py
"""

import cv2
import numpy as np
import torch
from torch.utils.data import Dataset, DataLoader, WeightedRandomSampler
import glob
import pickle
import matplotlib.pyplot as plt


torch.manual_seed(42)

def select_device(args):
    """
    Sets the device to perform the training on
    Args:
        device: If set, specified device will be used.
                Otherwise, GPU will be used if available.
    """
    if not args.DEVICE:
        args.DEVICE = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        print("Device: ", args.DEVICE)

    return args

def plot_histogram(steering, name):
    _ = plt.hist(steering, bins="auto")  # arguments are passed to np.histogram
    plt.title("Histogram of steering values with respect to {} camera".format(name))
    plt.show()


def get_histogram_weights(steering, no_cams, center_steering=None, center_edges=None, hist=None):
    # Oversampling method to overcome class imbalance

    counts = hist.tolist()
    counts = [no_cams * c for c in counts]

    # Calculate class weights
    class_weights = [sum(counts) / c for c in counts]

    # Assign weights to each input sample
    input_weights = []
    for s in center_steering:
        for i, edge in enumerate(center_edges):
            if s > edge:
                continue
            else:
                input_weights.append(class_weights[i-1])
                break

    input_weights *= no_cams
    return input_weights



class DatasetLoader(Dataset):
    """"For loading RGB images and corresponding depth"""
    
    def __init__(self, dataset_directory, episode_positions, no_cams=7, skipped=10, 
                       gt_trajectory=True, gt_img=True):
        
        self.dataset_directory = dataset_directory
        self.rgb_images = []
        self.steering = []
        self.weights = []
        self.transform = None
        self.cam_nos = []

        # Create the list of indices that would ideally be collected from an episode
        indices = list(range(1200))
        center_steering = []

        print(episode_positions)

        plot_flag = False
        for pos in episode_positions:

            try:
                # Only use positions in the dataset where mpc data exists
                if gt_trajectory:
                    data = pickle.load(open(self.dataset_directory + pos + '/mpc/mpc_gt_data.pkl', 'rb'))
                else:
                    data = pickle.load(open(self.dataset_directory + pos + '/mpc/mpc_slam_data.pkl', 'rb'))
                print(pos, len(data.cams[0].steering))
            except:
                continue
            
            for i, cam in enumerate(data.cams):
                if i == no_cams:
                    break
                
                if gt_img:
                    path = "{}/{}/gt/{}/*.png".format(self.dataset_directory, pos, cam.name)
                else:
                    path = "{}/{}/synthesized/{}/*.png".format(self.dataset_directory, pos, cam.name)

                images = sorted(glob.glob(path, recursive=True))

                # Get the indices for recorded images
                # Image recording starts with frame #11, frames use 1-based indexing
                # Take steering indices until the point where recording stops while applying mpc
                # Steering values are 0-based, images are 1-based index, hence equality
                img_idx = [int(img.split('/')[-1][:-4]) for img in images if int(img.split('/')[-1][:-4]) < (len(cam.steering)+skipped+1)]

                # Get the intersection of indices that are collected from each camera
                indices = list(set(indices).intersection(img_idx))

            for i, cam in enumerate(data.cams):

                if i == no_cams:
                    break

                if gt_img:
                    path = "{}/{}/gt/{}/*.png".format(self.dataset_directory, pos, cam.name)
                else:
                    path = "{}/{}/synthesized/{}/*.png".format(self.dataset_directory, pos, cam.name)

                images = sorted(glob.glob(path, recursive=True))

                # Only get the steering and image pairs that are present for other cameras
                images = [img for img in images if int(img.split('/')[-1][:-4]) in indices]
                images = images[1:]
                # print(len(images))

                # Steering commands use 0-based index
                steering = cam.steering.copy() # [cam.steering[i-1] for i in indices]
                steering = steering[1:]

                if cam.name == 'center':
                    center_steering += steering

                self.steering += steering
                self.cam_nos += [i] * len(steering)
                self.rgb_images += images

        plot_flag = False

        print(len(self.rgb_images))
        print(len(self.steering))

        assert(len(self.rgb_images) == len(self.steering))

        print("# of RGB images, in total: ", len(self.rgb_images))

        if plot_flag:
            plot_histogram(self.steering, "all")

        hist, edges = np.histogram(center_steering,  bins=10)
        print("Edges: ", edges)

        self.weights = get_histogram_weights(self.steering, no_cams, 
            center_steering=center_steering, center_edges=edges, hist=hist)


    def __len__(self):
        return len(self.rgb_images)
    
    def __getitem__(self, idx):
        
        data = dict()

        img_file = self.rgb_images[idx]
        img = cv2.imread(img_file)
        rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # BGR to RGB

        x=50
        w=1000
        rgb_img = rgb_img[:, x:x+w] # Crop image

        dim = (128, 128)
        rgb_img = cv2.resize(rgb_img, dim) # Resize image / downsample

        rgb_img = rgb_img / 255.0 # Scale color values 
        rgb_img = np.transpose(rgb_img, (2,0,1))  # Since Pytorch models take tensors 

        if self.transform:
            rgb_img = self.transform(torch.from_numpy(rgb_img))
            data['rgb'] = rgb_img.clone()
        else:
            data['rgb'] = rgb_img.copy()

        data['steering'] = self.steering[idx]
        data['filenames'] = img_file
        data['cam_no'] = self.cam_nos[idx]

        return data


def create_dataloader(dd_object, weighted_sampling=True, batch_size=100):
    train_split = (len(dd_object)*8) // 10
    lengths = [train_split, len(dd_object)-train_split]
    trainset, validset = torch.utils.data.random_split(dd_object, lengths, generator=torch.Generator().manual_seed(42))


    if not weighted_sampling:
        trainloader = DataLoader(trainset, batch_size=batch_size, shuffle=True, num_workers=1)
        validloader = DataLoader(validset, batch_size=batch_size, shuffle=True, num_workers=1)
    else:
        train_weights = np.take(np.array(dd_object.weights), trainset.indices)
        train_sampler = WeightedRandomSampler(train_weights, len(trainset))

        valid_weights = np.take(np.array(dd_object.weights), validset.indices)
        valid_sampler = WeightedRandomSampler(valid_weights, len(validset))
        trainloader = DataLoader(trainset, sampler=train_sampler, batch_size=batch_size, num_workers=1)
        validloader = DataLoader(validset, sampler=valid_sampler, batch_size=batch_size, num_workers=1)

    return trainloader, validloader
