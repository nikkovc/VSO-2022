# from scipy.interpolate import RectBivariateSpline, interp2d
from evalBezierFuncs_3P import *
import numpy as np





class TorqueProfile():

	def __init__(self, model_filepath='TorqueProfile/torqueProfileCoeffs_dataport3P.csv',phase_order=3, stride_length_order=1, incline_order=1):
		self.phaseDelins = [0.1,0.5,0.65,1]
		self.best_fit_params_torque = self.loadBezierCurves(model_filepath)
		self.phase_order = phase_order
		self.stride_length_order = stride_length_order
		self.incline_order = incline_order
		self.numFuncs = (incline_order+1) * (stride_length_order+1) * (phase_order+1)


	def loadBezierCurves(self,filename):
	    data = np.loadtxt(filename,delimiter=',')

	    best_fit_params_torque = data[:]

	    return best_fit_params_torque

	def evalTorqueProfile(self,phase_estimate, stepLength_estimate, incline_estimate):

		return self.evalBiologicalProfile(phase_estimate, stepLength_estimate, incline_estimate)

	def evalBiologicalProfile(self,phase_estimate, stepLength_estimate, incline_estimate):
		torque = 0
		if phase_estimate <= 0.65:
			torque = returnPiecewiseBezier3P(phase_estimate,stepLength_estimate,incline_estimate, self.best_fit_params_torque, self.phaseDelins,
											self.numFuncs, self.phase_order, self.stride_length_order, self.incline_order)/5

		if torque < 0:
			torque = 0

		return torque


