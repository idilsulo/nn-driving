"""
train.py
"""
import sys
from turtle import pos, position
sys.path.append('../')

import os
import numpy as np
import torch.nn as nn
import torch.optim
from torch.utils.tensorboard import SummaryWriter
from itertools import product


from config import CamParams, SimulatorParams, TrainParams
from model import *
from dataloader import *
import arguments

torch.manual_seed(42)

cam_params = CamParams()
sim_params = SimulatorParams()
train_params = TrainParams()


def select_device(args):
	"""
	Sets the device to perform the training on

	Args:
		device: If set, specified device will be used.
				Otherwise, GPU will be used if available.
	"""
	if not train_params.DEVICE:
		train_params.DEVICE = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
		print("Device: ", train_params.DEVICE)

	return args


def get_criterion(loss_type):
	if loss_type == 'MSE':
		criterion = nn.MSELoss()
	elif loss_type == 'L1':
		criterion = nn.L1Loss()

	return criterion


def train(trainloader, model, optimizer, criterion, device):

	model.train()

	train_loss = 0.0
	
	for i, minibatch in enumerate(trainloader):
		
		optimizer.zero_grad()
		
		img = minibatch['rgb'].double()
		label = minibatch['steering'].double()

		img = img.to(device)
		label = label.to(device)

		pred = model(img)
		pred = pred.squeeze()
		
		loss = criterion(pred, label)
		
		loss.backward()
		optimizer.step()

		train_loss += loss.item()
	
	train_loss /= len(trainloader)
	return train_loss

def test(validloader, model, criterion, device):
	model.eval()

	valid_loss = 0.0

	for i, minibatch in enumerate(validloader):
			
		img = minibatch['rgb'].double()
		label = minibatch['steering'].double()

		img = img.to(device)
		label = label.to(device)

		pred = model(img)
		pred = pred.squeeze()
		
		loss = criterion(pred, label)
		valid_loss += loss.item()

	valid_loss /= len(validloader)
	return valid_loss


def train_and_test(trainloader, validloader, device, num_epochs=100, lr=0.2, 
	loss_type='MSE', model_dir="models", exp_dir="experiments", exp_name="default_exp",
	mean=None, std=None):

	os.makedirs(model_dir, exist_ok=True)
	os.makedirs(exp_dir, exist_ok=True)
	
	tensorboard_dir = "{}/{}".format(exp_dir, exp_name)
	os.makedirs(tensorboard_dir, exist_ok=True)
	writer = SummaryWriter(tensorboard_dir)

	writer.add_text("mean", str(mean), 0)
	writer.add_text("std", str(std), 0)

	criterion = get_criterion(loss_type)
	
	model = CNN().double()
	model = model.to(device)

	optimizer = torch.optim.Adam(model.parameters(), lr=lr, amsgrad=False)

	best_loss = np.inf
	patience = 15

	for epoch in range(1, num_epochs+1):

		if patience == 0:
			# Stop training 
			# At most 15 epochs after best validation loss
			break
		
		train_loss = train(trainloader, model, optimizer, criterion, device)
		writer.add_scalar('Loss/train', train_loss, epoch)

		valid_loss = test(validloader, model, criterion, device)
		writer.add_scalar('Loss/valid', valid_loss, epoch)

		# Save best model based on lowest loss value
		if valid_loss <= best_loss:
			best_loss = valid_loss
			model_path = "{}/{}_best_model.pth".format(model_dir, exp_name)
			torch.save(model.state_dict(), model_path)
			patience = 15

		elif epoch > 30:
			patience -= 1

		print("Epoch {:3d} / {:3d}  |  TRAIN  train loss ({}) : {:.3f} | TEST test loss : {:.3f}".format(
					epoch, num_epochs, loss_type, train_loss, valid_loss))

		model_path = "{}/{}_last_model.pth".format(model_dir, exp_name)
		torch.save(model.state_dict(), model_path)

		model.train()
		
	print()


if __name__ == '__main__':

	ap = arguments.make_parser()
	args = ap.parse_args()

	town_no = sim_params.TOWN[4:]

	# Set device as CPU / GPU
	args = select_device(args)

	hyperparam_exp_name = "LOSS[{}]--WS[{}]--LR[{}]--NO_CAMS[{}]".format(
		train_params.LOSS, train_params.WS, train_params.LEARNING_RATE, train_params.NO_CAMS)

	print(hyperparam_exp_name)
	
	episode_positions = train_params.EPISODE_POS

	dd_object = DatasetLoader(dataset_directory=args.DATASET_PATH, episode_positions=episode_positions,
								no_cams=train_params.NO_CAMS, skipped=sim_params.NO_SKIPPED_FRAMES,
								gt_trajectory=train_params.GT_TRAJECTORY, gt_img=train_params.GT_IMG)

	trainloader, validloader = create_dataloader(dd_object, weighted_sampling=train_params.WS, batch_size=train_params.BATCH_SIZE)
	
	train_and_test(trainloader=trainloader, validloader=validloader, num_epochs=train_params.NO_EPOCHS, 
			loss_type=train_params.LOSS, device=train_params.DEVICE,  lr=train_params.LEARNING_RATE, 
			model_dir=args.MODEL_SAVE_DIR, exp_dir=args.EXPERIMENT_DIR, exp_name=hyperparam_exp_name,
			mean=None, std=None)
