import numpy as np
from time import time
from evalBezierFuncs_3P import *
from evalFourierFuncs_3P import *


class GaitModel():

	def __init__(self, model_filepath ,phase_order=20):
		self.best_fit_params_shankAngle = self.loadCoefficients(model_filepath)
		self.phase_order = phase_order


	def loadCoefficients(self,filename):
		data = np.loadtxt(filename,delimiter=',')

		best_fit_params_shankAngle = data[0,:]

		return best_fit_params_shankAngle

	#KINEMATICS
	def returnShankAngle(self, phase):
		return self.best_fit_params_shankAngle @ returnFourier(phase, self.phase_order)


	#FIRST DERIVATIVES
	def returnShankAngleDeriv_dphase(self, phase):
		return self.best_fit_params_shankAngle @ returnFourierDeriv(phase, self.phase_order)

	def returnShankAngleDeriv_dsL(self, phase,stepLength,incline):
		return self.best_fit_params_shankAngle @ returnFourierBasis_DerivEval_dsL(phase,stepLength,incline, self.phase_order, self.stride_length_order, self.incline_order)

	def returnShankAngleDeriv_dincline(self, phase,stepLength,incline):
		return self.best_fit_params_shankAngle @ returnFourierBasis_DerivEval_dincline(phase,stepLength,incline, self.phase_order, self.stride_length_order, self.incline_order)

	#SECOND DERIVATIVES
	def returnShankAngle2ndDeriv_dphase2(self, phase,stepLength,incline):
		return self.best_fit_params_shankAngle @ returnFourier2ndDeriv(phase, self.phase_order)

	def returnShankAngle2ndDeriv_dphasedsL(self, phase,stepLength,incline):
		return self.best_fit_params_shankAngle @ returnFourierBasis_2ndDerivEval_dphasedsL(phase,stepLength,incline, self.phase_order, self.stride_length_order, self.incline_order)

	def returnShankAngle2ndDeriv_dphasedincline(self, phase,stepLength,incline):
		return self.best_fit_params_shankAngle @ returnFourierBasis_2ndDerivEval_dphasedincline(phase,stepLength,incline, self.phase_order, self.stride_length_order, self.incline_order)





