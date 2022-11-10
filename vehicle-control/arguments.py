"""
arguments.py
"""
import argparse
import distutils.util


def make_parser(jupyter_notebook=False):
	"""
	Add arguments to parser object

	Args:
		jupyter_notebook: Flag to use the argument parser where there is no main
	Returns:
		ap: Argument parser object
	"""
	if jupyter_notebook:
		# Argument parser cannot be called when there is no main.
		# Executing the line below overwrites sys.argv
		import sys;
		sys.argv = [''];
		del sys

	ap = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)

	ap.add_argument("--PORT", type=int, required=False, default=2000, 
					help="Port number to use while connecting to CARLA simulator.")

	ap.add_argument("--OUT_PATH", type=str, required=False, default="../data/", 
					help="Path to save the output images.")

	ap.add_argument("--EPISODE", type=str, required=False, default="episode01/", 
					help="Path to save the output images.")

	ap.add_argument("--START_POS", type=int, required=False, default="47", 
					help="Starting position of the vehicle on the map.")

	# MPC
	ap.add_argument("--GT", type=lambda x: bool(distutils.util.strtobool(x)), required=False, default=True, 
					help="Whether to execute mpc on ground truth or SLAM trajectory.")

	# INFERENCE
	ap.add_argument("--MODEL_PATH", type=str, required=False, default="train/models/ours.pth", 
					help="Path to the pre-trained model to be used.")

	# TRAIN / DATASET / DATALOADER
	ap.add_argument("--DATASET_PATH", type=str, required=False, default="../../data/episode01/",
					help="Path to the dataset folder.")

	ap.add_argument("--WEIGHTED_SAMPLING", type=lambda x: bool(distutils.util.strtobool(x)), required=False, default=True, 
					help="Whether to apply weighted sampling while constructing the dataloader / batches.")

	# SAVING / PLOTTING
	ap.add_argument("--MODEL_SAVE_DIR", type=str, required=False, default="train/models",
					help="The directory to save models in.")
	
	ap.add_argument("--EXPERIMENT_DIR", type=str, required=False, default="train/experiments",
					help="The directory to save experiments in.")

	# HYPER PARAMETER EXPERIMENT
	ap.add_argument("--HYPERPARAM_EXP_NAME", type=str, required=False, default="default_exp",
					help="Name of the hyper parameter experiment. If empty string "
						 "it is interpreted as a single experiment.")
	return ap
