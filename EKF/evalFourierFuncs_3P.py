import numpy as np
from evalBezierFuncs_3P import *


#returns the evaluation of the three basis functions

def _returnKroneckerBasisEvald(inclineFuncs, stepLengthFuncs, phaseFuncs):

	numInclineFuncs = len(inclineFuncs)
	numStepLengthFuncs = len(stepLengthFuncs)
	numPhaseFuncs = len(phaseFuncs)
	
	basisEvald = np.kron(inclineFuncs, np.kron(stepLengthFuncs, phaseFuncs))

	return basisEvald


# evaluation functions

def returnFourierBasis_Eval(phase,stepLength,incline, phase_order=50, stride_length_order=1, incline_order=1):
	inclineFuncs = selectOrderBezier(incline_order, incline)
	stepLengthFuncs = selectOrderBezier(stride_length_order, stepLength)
	phaseFuncs = returnFourier(phase, phase_order)

	fourierCoeffs3P = _returnKroneckerBasisEvald(inclineFuncs, stepLengthFuncs, phaseFuncs)


	return fourierCoeffs3P

# derivatives
def returnFourierBasis_DerivEval_dphase(phase,stepLength,incline, phase_order, stride_length_order, incline_order):

	inclineFuncs = selectOrderBezier(incline_order, incline)
	stepLengthFuncs = selectOrderBezier(stride_length_order, stepLength)
	phaseFuncs = returnFourierDeriv(phase, phase_order)

	fourierDerivCoeffs3P = _returnKroneckerBasisEvald(inclineFuncs, stepLengthFuncs, phaseFuncs)




	return fourierDerivCoeffs3P


def returnFourierBasis_DerivEval_dsL(phase,stepLength,incline, phase_order, stride_length_order, incline_order):
	inclineFuncs = selectOrderBezier(incline_order, incline)
	stepLengthFuncs = selectOrderBezierDeriv(stride_length_order, stepLength)
	phaseFuncs = returnFourier(phase, phase_order)


	fourierDerivCoeffs3P = _returnKroneckerBasisEvald(inclineFuncs, stepLengthFuncs, phaseFuncs)




	return fourierDerivCoeffs3P


def returnFourierBasis_DerivEval_dincline(phase,stepLength,incline, phase_order, stride_length_order, incline_order):

	inclineFuncs = selectOrderBezierDeriv(incline_order, incline)
	stepLengthFuncs = selectOrderBezier(stride_length_order, stepLength)
	phaseFuncs = returnFourier(phase, phase_order)


	fourierDerivCoeffs3P = _returnKroneckerBasisEvald(inclineFuncs, stepLengthFuncs, phaseFuncs)




	return fourierDerivCoeffs3P


# second derivatives

def returnFourierBasis_2ndDerivEval_dphase2(phase,stepLength,incline, phase_order, stride_length_order, incline_order):

	inclineFuncs = selectOrderBezier(incline_order, incline)
	stepLengthFuncs = selectOrderBezier(stride_length_order, stepLength)
	phaseFuncs = returnFourier2ndDeriv(phase, phase_order)

	fourier2ndDerivCoeffs3P = _returnKroneckerBasisEvald(inclineFuncs, stepLengthFuncs, phaseFuncs)




	return fourier2ndDerivCoeffs3P



def returnFourierBasis_2ndDerivEval_dphasedsL(phase,stepLength,incline, phase_order, stride_length_order, incline_order):


	inclineFuncs = selectOrderBezier(incline_order, incline)
	stepLengthFuncs = selectOrderBezierDeriv(stride_length_order, stepLength)
	phaseFuncs = returnFourierDeriv(phase, phase_order)


	fourier2ndDerivCoeffs3P = _returnKroneckerBasisEvald(inclineFuncs, stepLengthFuncs, phaseFuncs)


	return fourier2ndDerivCoeffs3P

def returnFourierBasis_2ndDerivEval_dphasedincline(phase,stepLength,incline, phase_order, stride_length_order, incline_order):


	inclineFuncs = selectOrderBezierDeriv(incline_order, incline)
	stepLengthFuncs = selectOrderBezier(stride_length_order, stepLength)
	phaseFuncs = returnFourierDeriv(phase, phase_order)


	
	fourier2ndDerivCoeffs3P = _returnKroneckerBasisEvald(inclineFuncs, stepLengthFuncs, phaseFuncs)



	return fourier2ndDerivCoeffs3P



#the most basic fourier functions

def returnFourier(t, N):

	ws = np.arange(1,N+1)
	fourierEvals = np.concatenate((np.array([1]), np.cos(ws * 2*np.pi * t), np.sin(ws* 2*np.pi * t)))

	return fourierEvals


def returnFourierDeriv(t, N):
	ws = np.arange(1,N+1)

	d1 = ws * 2*np.pi * -np.sin(ws * 2*np.pi * t)
	d2 = ws * 2*np.pi * np.cos(ws* 2*np.pi * t)
	fourierEvals = np.concatenate((np.array([0]), d1, d2))

	return fourierEvals

def returnFourier2ndDeriv(t, N):


	ws = np.arange(1,N+1)

	dd1 = ws * ws * 2*np.pi * 2*np.pi * -np.cos(ws * 2*np.pi * t)
	dd2 = ws * ws * 2*np.pi * 2*np.pi * -np.sin(ws * 2*np.pi * t)
	fourierEvals = np.concatenate((np.array([0]), dd1, dd2))

	return fourierEvals






