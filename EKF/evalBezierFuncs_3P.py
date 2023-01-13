import numpy as np



# piecewise evaluation functions

def returnPiecewiseBezier3P(phase,stepLength,incline,h_piecewise, phaseDelins, numFuncs, phase_order=3, stride_length_order=1, incline_order=1):


	if isinstance(phase, np.ndarray):

		phase[phase<0] = 0
		phase[phase>1] = 1


		offsets = np.zeros((phase.shape[0]))

		offsets[phase <= phaseDelins[0]] = 0
		offsets[np.logical_and(phase <= phaseDelins[1], phase > phaseDelins[0])] = 1
		offsets[np.logical_and(phase <= phaseDelins[2], phase > phaseDelins[1])] = 2
		offsets[np.logical_and(phase <= phaseDelins[3], phase > phaseDelins[2])] = 3

		# print(offsets)

		phaseIdxs = np.arange(0, numFuncs)
		# print(phaseIdxs)
		phaseIdxs = np.tile(phaseIdxs,(phase.shape[0],1))
		# print(phaseIdxs)
		offsets = (numFuncs*offsets).reshape(-1,1)
		# print(offsets)
		phaseIdxs = phaseIdxs + np.tile(offsets,(1,phaseIdxs.shape[1]))

		phaseIdxs = phaseIdxs.astype(int)
		# print('phaseIdxs')
		# print(phaseIdxs)
		# print(phaseIdxs.shape)
		# print(type(phaseIdxs))


		# print(h_piecewise[phaseIdxs])
		# print(h_piecewise)

		evald = returnBezierEval3P(phase,stepLength,incline, phase_order, stride_length_order, incline_order)
		# print('evald')
		# print(evald)
		# print(evald.shape)
		# input()

		# f = lambda m, x : x[m]

		# dd = np.apply_along_axis(f,1,phaseIdxs, h_piecewise)
		# print(dd)
		coeffs = h_piecewise[phaseIdxs]
		# print('coeffs')
		# print(coeffs)
		# print(coeffs.shape)
		# input()

		output = np.sum(coeffs * evald,1)
		# print('output')
		# print(output)
		# print(output.shape)
		# input()
			
	else:
		if phase < 0:
			phase = 0
			
		elif phase > 1:
			phase = 1
		
		if phase <= phaseDelins[0]:
			phaseIdxs = range(0 + 0*numFuncs,numFuncs + 0*numFuncs)
		elif phase <= phaseDelins[1]:
			phaseIdxs = range(0 + 1*numFuncs,numFuncs + 1*numFuncs)
		elif phase <= phaseDelins[2]:
			phaseIdxs = range(0 + 2*numFuncs,numFuncs + 2*numFuncs)
		else:
			phaseIdxs = range(0 + 3*numFuncs,numFuncs + 3*numFuncs)

		
			
		output = h_piecewise[phaseIdxs] @ returnBezierEval3P(phase,stepLength,incline, phase_order, stride_length_order, incline_order)
	

	
	return output






def returnPiecewiseBezier3PDeriv_dphase(phase,stepLength,incline,h_piecewise, phaseDelins, numFuncs, phase_order=3, stride_length_order=1, incline_order=1):

	if isinstance(phase, np.ndarray):

		phase[phase<0] = 0
		phase[phase>1] = 1


		offsets = np.zeros((phase.shape[0]))

		offsets[phase <= phaseDelins[0]] = 0
		offsets[np.logical_and(phase <= phaseDelins[1], phase > phaseDelins[0])] = 1
		offsets[np.logical_and(phase <= phaseDelins[2], phase > phaseDelins[1])] = 2
		offsets[np.logical_and(phase <= phaseDelins[3], phase > phaseDelins[2])] = 3

		phaseIdxs = np.arange(0, numFuncs)
		# print(phaseIdxs)
		phaseIdxs = np.tile(phaseIdxs,(phase.shape[0],1))
		# print(phaseIdxs)
		offsets = (numFuncs*offsets).reshape(-1,1)
		# print(offsets)
		phaseIdxs = phaseIdxs + np.tile(offsets,(1,phaseIdxs.shape[1]))

		phaseIdxs = phaseIdxs.astype(int)

		evald = returnBezier3PDerivEval_dphase(phase,stepLength,incline, phase_order, stride_length_order, incline_order)
		coeffs = h_piecewise[phaseIdxs]

		output = np.sum(coeffs * evald,1)

	else:

		if phase < 0:
			phase = 0
			
		elif phase > 1:
			phase = 1
		
		if phase <= phaseDelins[0]:
			phaseIdxs = range(0 + 0*numFuncs,numFuncs + 0*numFuncs)
		elif phase <= phaseDelins[1]:
			phaseIdxs = range(0 + 1*numFuncs,numFuncs + 1*numFuncs)
		elif phase <= phaseDelins[2]:
			phaseIdxs = range(0 + 2*numFuncs,numFuncs + 2*numFuncs)
		else:
			phaseIdxs = range(0 + 3*numFuncs,numFuncs + 3*numFuncs)

		
			
		output = h_piecewise[phaseIdxs] @ returnBezier3PDerivEval_dphase(phase,stepLength,incline, phase_order, stride_length_order, incline_order)
	
	return output



def returnPiecewiseBezier3PDeriv_dsL(phase,stepLength,incline,h_piecewise, phaseDelins, numFuncs, phase_order=3, stride_length_order=1, incline_order=1):

	if isinstance(phase, np.ndarray):

		phase[phase<0] = 0
		phase[phase>1] = 1


		offsets = np.zeros((phase.shape[0]))

		offsets[phase <= phaseDelins[0]] = 0
		offsets[np.logical_and(phase <= phaseDelins[1], phase > phaseDelins[0])] = 1
		offsets[np.logical_and(phase <= phaseDelins[2], phase > phaseDelins[1])] = 2
		offsets[np.logical_and(phase <= phaseDelins[3], phase > phaseDelins[2])] = 3

		phaseIdxs = np.arange(0, numFuncs)
		# print(phaseIdxs)
		phaseIdxs = np.tile(phaseIdxs,(phase.shape[0],1))
		# print(phaseIdxs)
		offsets = (numFuncs*offsets).reshape(-1,1)
		# print(offsets)
		phaseIdxs = phaseIdxs + np.tile(offsets,(1,phaseIdxs.shape[1]))

		phaseIdxs = phaseIdxs.astype(int)

		evald = returnBezier3PDerivEval_dsL(phase,stepLength,incline, phase_order, stride_length_order, incline_order)
		coeffs = h_piecewise[phaseIdxs]

		output = np.sum(coeffs * evald,1)
	else:

		if phase < 0:
			phase = 0
			
		elif phase > 1:
			phase = 1
		
		if phase <= phaseDelins[0]:
			phaseIdxs = range(0 + 0*numFuncs,numFuncs + 0*numFuncs)
		elif phase <= phaseDelins[1]:
			phaseIdxs = range(0 + 1*numFuncs,numFuncs + 1*numFuncs)
		elif phase <= phaseDelins[2]:
			phaseIdxs = range(0 + 2*numFuncs,numFuncs + 2*numFuncs)
		else:
			phaseIdxs = range(0 + 3*numFuncs,numFuncs + 3*numFuncs)

		
		
		output = h_piecewise[phaseIdxs] @ returnBezier3PDerivEval_dsL(phase,stepLength,incline, phase_order, stride_length_order, incline_order)
	
	return output


def returnPiecewiseBezier3PDeriv_dincline(phase,stepLength,incline,h_piecewise, phaseDelins, numFuncs, phase_order=3, stride_length_order=1, incline_order=1):

	if isinstance(phase, np.ndarray):

		phase[phase<0] = 0
		phase[phase>1] = 1


		offsets = np.zeros((phase.shape[0]))

		offsets[phase <= phaseDelins[0]] = 0
		offsets[np.logical_and(phase <= phaseDelins[1], phase > phaseDelins[0])] = 1
		offsets[np.logical_and(phase <= phaseDelins[2], phase > phaseDelins[1])] = 2
		offsets[np.logical_and(phase <= phaseDelins[3], phase > phaseDelins[2])] = 3

		phaseIdxs = np.arange(0, numFuncs)
		# print(phaseIdxs)
		phaseIdxs = np.tile(phaseIdxs,(phase.shape[0],1))
		# print(phaseIdxs)
		offsets = (numFuncs*offsets).reshape(-1,1)
		# print(offsets)
		phaseIdxs = phaseIdxs + np.tile(offsets,(1,phaseIdxs.shape[1]))

		phaseIdxs = phaseIdxs.astype(int)
		evald = returnBezier3PDerivEval_dincline(phase,stepLength,incline, phase_order, stride_length_order, incline_order)
		coeffs = h_piecewise[phaseIdxs]

		output = np.sum(coeffs * evald,1)

	else:


		if phase < 0:
			phase = 0
			
		elif phase > 1:
			phase = 1
		
		if phase <= phaseDelins[0]:
			phaseIdxs = range(0 + 0*numFuncs,numFuncs + 0*numFuncs)
		elif phase <= phaseDelins[1]:
			phaseIdxs = range(0 + 1*numFuncs,numFuncs + 1*numFuncs)
		elif phase <= phaseDelins[2]:
			phaseIdxs = range(0 + 2*numFuncs,numFuncs + 2*numFuncs)
		else:
			phaseIdxs = range(0 + 3*numFuncs,numFuncs + 3*numFuncs)

		
		
		output = h_piecewise[phaseIdxs] @ returnBezier3PDerivEval_dincline(phase,stepLength,incline, phase_order, stride_length_order, incline_order)
	
	return output


def returnPiecewiseBezier3P_2ndDeriv_dphase2(phase,stepLength,incline,h_piecewise, phaseDelins, numFuncs, phase_order=3, stride_length_order=1, incline_order=1):

	if isinstance(phase, np.ndarray):

		phase[phase<0] = 0
		phase[phase>1] = 1


		offsets = np.zeros((phase.shape[0]))

		offsets[phase <= phaseDelins[0]] = 0
		offsets[np.logical_and(phase <= phaseDelins[1], phase > phaseDelins[0])] = 1
		offsets[np.logical_and(phase <= phaseDelins[2], phase > phaseDelins[1])] = 2
		offsets[np.logical_and(phase <= phaseDelins[3], phase > phaseDelins[2])] = 3

		phaseIdxs = np.arange(0, numFuncs)
		# print(phaseIdxs)
		phaseIdxs = np.tile(phaseIdxs,(phase.shape[0],1))
		# print(phaseIdxs)
		offsets = (numFuncs*offsets).reshape(-1,1)
		# print(offsets)
		phaseIdxs = phaseIdxs + np.tile(offsets,(1,phaseIdxs.shape[1]))

		phaseIdxs = phaseIdxs.astype(int)
		evald = eturnBezier3P_2ndDerivEval_dphase2(phase,stepLength,incline, phase_order, stride_length_order, incline_order)
		coeffs = h_piecewise[phaseIdxs]

		output = np.sum(coeffs * evald,1)

	else:


		if phase < 0:
			phase = 0
			
		elif phase > 1:
			phase = 1
		
		if phase <= phaseDelins[0]:
			phaseIdxs = range(0 + 0*numFuncs,numFuncs + 0*numFuncs)
		elif phase <= phaseDelins[1]:
			phaseIdxs = range(0 + 1*numFuncs,numFuncs + 1*numFuncs)
		elif phase <= phaseDelins[2]:
			phaseIdxs = range(0 + 2*numFuncs,numFuncs + 2*numFuncs)
		else:
			phaseIdxs = range(0 + 3*numFuncs,numFuncs + 3*numFuncs)

		
		
		output = h_piecewise[phaseIdxs] @ returnBezier3P_2ndDerivEval_dphase2(phase,stepLength,incline, phase_order, stride_length_order, incline_order)
		
	return output



def returnPiecewiseBezier3P_2ndDeriv_dphasedsL(phase,stepLength,incline,h_piecewise, phaseDelins, numFuncs, phase_order=3, stride_length_order=1, incline_order=1):


	if isinstance(phase, np.ndarray):

		phase[phase<0] = 0
		phase[phase>1] = 1


		offsets = np.zeros((phase.shape[0]))

		offsets[phase <= phaseDelins[0]] = 0
		offsets[np.logical_and(phase <= phaseDelins[1], phase > phaseDelins[0])] = 1
		offsets[np.logical_and(phase <= phaseDelins[2], phase > phaseDelins[1])] = 2
		offsets[np.logical_and(phase <= phaseDelins[3], phase > phaseDelins[2])] = 3

		phaseIdxs = np.arange(0, numFuncs)
		# print(phaseIdxs)
		phaseIdxs = np.tile(phaseIdxs,(phase.shape[0],1))
		# print(phaseIdxs)
		offsets = (numFuncs*offsets).reshape(-1,1)
		# print(offsets)
		phaseIdxs = phaseIdxs + np.tile(offsets,(1,phaseIdxs.shape[1]))

		phaseIdxs = phaseIdxs.astype(int)
		evald = returnBezier3P_2ndDerivEval_dphasedsL(phase,stepLength,incline, phase_order, stride_length_order, incline_order)
		coeffs = h_piecewise[phaseIdxs]

		output = np.sum(coeffs * evald,1)

	else:

		if phase < 0:
			phase = 0
			
		elif phase > 1:
			phase = 1
		
		if phase <= phaseDelins[0]:
			phaseIdxs = range(0 + 0*numFuncs,numFuncs + 0*numFuncs)
		elif phase <= phaseDelins[1]:
			phaseIdxs = range(0 + 1*numFuncs,numFuncs + 1*numFuncs)
		elif phase <= phaseDelins[2]:
			phaseIdxs = range(0 + 2*numFuncs,numFuncs + 2*numFuncs)
		else:
			phaseIdxs = range(0 + 3*numFuncs,numFuncs + 3*numFuncs)

		
		
		output = h_piecewise[phaseIdxs] @ returnBezier3P_2ndDerivEval_dphasedsL(phase,stepLength,incline, phase_order, stride_length_order, incline_order)
	
	return output

def returnPiecewiseBezier3P_2ndDeriv_dphasedincline(phase,stepLength,incline,h_piecewise, phaseDelins, numFuncs, phase_order=3, stride_length_order=1, incline_order=1):

	if isinstance(phase, np.ndarray):

		phase[phase<0] = 0
		phase[phase>1] = 1


		offsets = np.zeros((phase.shape[0]))

		offsets[phase <= phaseDelins[0]] = 0
		offsets[np.logical_and(phase <= phaseDelins[1], phase > phaseDelins[0])] = 1
		offsets[np.logical_and(phase <= phaseDelins[2], phase > phaseDelins[1])] = 2
		offsets[np.logical_and(phase <= phaseDelins[3], phase > phaseDelins[2])] = 3

		phaseIdxs = np.arange(0, numFuncs)
		# print(phaseIdxs)
		phaseIdxs = np.tile(phaseIdxs,(phase.shape[0],1))
		# print(phaseIdxs)
		offsets = (numFuncs*offsets).reshape(-1,1)
		# print(offsets)
		phaseIdxs = phaseIdxs + np.tile(offsets,(1,phaseIdxs.shape[1]))

		phaseIdxs = phaseIdxs.astype(int)
		evald = returnBezier3P_2ndDerivEval_dphasedincline(phase,stepLength,incline, phase_order, stride_length_order, incline_order)
		coeffs = h_piecewise[phaseIdxs]

		output = np.sum(coeffs * evald,1)

	else:


		if phase < 0:
			phase = 0
			
		elif phase > 1:
			phase = 1
		
		if phase <= phaseDelins[0]:
			phaseIdxs = range(0 + 0*numFuncs,numFuncs + 0*numFuncs)
		elif phase <= phaseDelins[1]:
			phaseIdxs = range(0 + 1*numFuncs,numFuncs + 1*numFuncs)
		elif phase <= phaseDelins[2]:
			phaseIdxs = range(0 + 2*numFuncs,numFuncs + 2*numFuncs)
		else:
			phaseIdxs = range(0 + 3*numFuncs,numFuncs + 3*numFuncs)

		
		
		output = h_piecewise[phaseIdxs] @ returnBezier3P_2ndDerivEval_dphasedincline(phase,stepLength,incline, phase_order, stride_length_order, incline_order)
		
	return output


#returns the evaluation of the three bezier basis functions

def returnBezierBasisEvald(inclineFuncs, stepLengthFuncs, phaseFuncs, vectorize=False):

	if vectorize:
		numInclineFuncs = inclineFuncs.shape[1]
		numStepLengthFuncs = stepLengthFuncs.shape[1]
		numPhaseFuncs = phaseFuncs.shape[1]
		
		bezierCoeffs3P = np.zeros((inclineFuncs.shape[0], numInclineFuncs*numStepLengthFuncs*numPhaseFuncs))
		N=0
		for ii in range(numInclineFuncs):
			inclineFunc = inclineFuncs[:,ii]
			for jj in range(numStepLengthFuncs):
				stepLengthFunc = stepLengthFuncs[:,jj]
				for kk in range(numPhaseFuncs):
					phaseFunc = phaseFuncs[:,kk]
					bezierCoeffs3P[:,N] = inclineFunc * stepLengthFunc * phaseFunc
					N += 1

	else:
		numInclineFuncs = len(inclineFuncs)
		numStepLengthFuncs = len(stepLengthFuncs)
		numPhaseFuncs = len(phaseFuncs)
		bezierCoeffs3P = np.kron(inclineFuncs, np.kron(stepLengthFuncs, phaseFuncs))
		

	return bezierCoeffs3P

# evaluation functions

def returnBezierEval3P(phase,stepLength,incline, phase_order=3, stride_length_order=1, incline_order=1):
	inclineFuncs = selectOrderBezier(incline_order, incline)
	stepLengthFuncs = selectOrderBezier(stride_length_order, stepLength)
	phaseFuncs = selectOrderBezier(phase_order, phase)


	if isinstance(phase, np.ndarray):

		bezierCoeffs3P = returnBezierBasisEvald(inclineFuncs, stepLengthFuncs, phaseFuncs, vectorize=True)

	else:
		bezierCoeffs3P = returnBezierBasisEvald(inclineFuncs, stepLengthFuncs, phaseFuncs, vectorize=False)


	return bezierCoeffs3P

# derivatives
def returnBezier3PDerivEval_dphase(phase,stepLength,incline, phase_order, stride_length_order, incline_order):

	inclineFuncs = selectOrderBezier(incline_order, incline)
	stepLengthFuncs = selectOrderBezier(stride_length_order, stepLength)
	phaseFuncs = selectOrderBezierDeriv(phase_order, phase)


	if isinstance(phase, np.ndarray):

		bezierDerivCoeffs3P = returnBezierBasisEvald(inclineFuncs, stepLengthFuncs, phaseFuncs, vectorize=True)

	else:
		bezierDerivCoeffs3P = returnBezierBasisEvald(inclineFuncs, stepLengthFuncs, phaseFuncs, vectorize=False)




	return bezierDerivCoeffs3P


def returnBezier3PDerivEval_dsL(phase,stepLength,incline, phase_order, stride_length_order, incline_order):
	inclineFuncs = selectOrderBezier(incline_order, incline)
	stepLengthFuncs = selectOrderBezierDeriv(stride_length_order, stepLength)
	phaseFuncs = selectOrderBezier(phase_order, phase)


	if isinstance(phase, np.ndarray):

		bezierDerivCoeffs3P = returnBezierBasisEvald(inclineFuncs, stepLengthFuncs, phaseFuncs, vectorize=True)

	else:
		bezierDerivCoeffs3P = returnBezierBasisEvald(inclineFuncs, stepLengthFuncs, phaseFuncs, vectorize=False)




	return bezierDerivCoeffs3P


def returnBezier3PDerivEval_dincline(phase,stepLength,incline, phase_order, stride_length_order, incline_order):

	inclineFuncs = selectOrderBezierDeriv(incline_order, incline)
	stepLengthFuncs = selectOrderBezier(stride_length_order, stepLength)
	phaseFuncs = selectOrderBezier(phase_order, phase)


	if isinstance(phase, np.ndarray):

		bezierDerivCoeffs3P = returnBezierBasisEvald(inclineFuncs, stepLengthFuncs, phaseFuncs, vectorize=True)

	else:
		bezierDerivCoeffs3P = returnBezierBasisEvald(inclineFuncs, stepLengthFuncs, phaseFuncs, vectorize=False)




	return bezierDerivCoeffs3P


# second derivatives

def returnBezier3P_2ndDerivEval_dphase2(phase,stepLength,incline, phase_order, stride_length_order, incline_order):

	inclineFuncs = selectOrderBezier(incline_order, incline)
	stepLengthFuncs = selectOrderBezier(stride_length_order, stepLength)
	phaseFuncs = selectOrderBezier2ndDeriv(phase_order, phase)


	if isinstance(phase, np.ndarray):

		bezier2ndDerivCoeffs3P = returnBezierBasisEvald(inclineFuncs, stepLengthFuncs, phaseFuncs, vectorize=True)

	else:
		bezier2ndDerivCoeffs3P = returnBezierBasisEvald(inclineFuncs, stepLengthFuncs, phaseFuncs, vectorize=False)




	return bezier2ndDerivCoeffs3P



def returnBezier3P_2ndDerivEval_dphasedsL(phase,stepLength,incline, phase_order, stride_length_order, incline_order):


	inclineFuncs = selectOrderBezier(incline_order, incline)
	stepLengthFuncs = selectOrderBezierDeriv(stride_length_order, stepLength)
	phaseFuncs = selectOrderBezierDeriv(phase_order, phase)


	if isinstance(phase, np.ndarray):

		bezier2ndDerivCoeffs3P = returnBezierBasisEvald(inclineFuncs, stepLengthFuncs, phaseFuncs, vectorize=True)

	else:
		bezier2ndDerivCoeffs3P = returnBezierBasisEvald(inclineFuncs, stepLengthFuncs, phaseFuncs, vectorize=False)




	return bezier2ndDerivCoeffs3P

def returnBezier3P_2ndDerivEval_dphasedincline(phase,stepLength,incline, phase_order, stride_length_order, incline_order):


	inclineFuncs = selectOrderBezierDeriv(incline_order, incline)
	stepLengthFuncs = selectOrderBezier(stride_length_order, stepLength)
	phaseFuncs = selectOrderBezierDeriv(phase_order, phase)


	if isinstance(phase, np.ndarray):

		bezier2ndDerivCoeffs3P = returnBezierBasisEvald(inclineFuncs, stepLengthFuncs, phaseFuncs, vectorize=True)

	else:
		bezier2ndDerivCoeffs3P = returnBezierBasisEvald(inclineFuncs, stepLengthFuncs, phaseFuncs, vectorize=False)




	return bezier2ndDerivCoeffs3P


#the most basic bezier functions
def returnBezierCubic(t):
	
	# print(type(t))
	if isinstance(t, np.ndarray):
		# print('NP')
		bezierCoeffs = np.array([(1-t)**3,
			3*(1-t)**2*t,
			3*(1-t)*(t)**2,
			(t)**3]).T

	else:
		bezierCoeffs = np.array([(1-t)**3,
			3*(1-t)**2*t,
			3*(1-t)*(t)**2,
			(t)**3])

	# print(bezierCoeffs.shape)
	return bezierCoeffs

def returnBezierQuadratic(t):

	if isinstance(t, np.ndarray):
		bezierCoeffs = np.array([(1-t)**2,
			2*(1-t)*t,
			t**2]).T

	else:
		bezierCoeffs = np.array([(1-t)**2,
		2*(1-t)*t,
		t**2])

	return bezierCoeffs

def returnBezierLinear(t):

	if isinstance(t, np.ndarray):
		#t : (N, )
		bezierCoeffs = np.array([(t),
			(1-t)]).T

	else:
		bezierCoeffs = np.array([(t),
			(1-t)])

	return bezierCoeffs


def returnBezierDerivCubic(t):
	if isinstance(t, np.ndarray):

		bezierCoeffs = np.array([-3*(1-t)**2,
			(3*(1-t)**2 - 6*(1 - t)*t),
			(6*(1 - t)*t - 3*(t**2)),
			3*(t**2)]).T

	else:
		bezierCoeffs = np.array([-3*(1-t)**2,
		(3*(1-t)**2 - 6*(1 - t)*t),
		(6*(1 - t)*t - 3*(t**2)),
		3*(t**2)])

	return bezierCoeffs

def returnBezierDerivQuadratic(t):

	if isinstance(t, np.ndarray):
		bezierCoeffs = np.array([-2*(1-t),
			(-2*(t) + 2*(1 - t)),
			2*t]).T

	else:
		bezierCoeffs = np.array([-2*(1-t),
			(-2*(t) + 2*(1 - t)),
			2*t])
	return bezierCoeffs


def returnBezierDerivLinear(t):

	if isinstance(t, np.ndarray):
		#t : (N, )
		bezierCoeffs = np.array([(1),
			(-1)]).T

	else:
		bezierCoeffs = np.array([(1),
			(-1)])

	return bezierCoeffs

def returnBezier2ndDerivCubic(t):

	if isinstance(t, np.ndarray):
		
		#t : (N, )
		bezierCoeffs = np.array([
			(6 * (1 - t)),
			( -2*(6*(1 - t)) +  (6*t) ), 
			( 6*(1 - t) + (-2 * (6 * t)) ),
			(6 * t)]).T

	else:
		bezierCoeffs = np.array([
			(6 * (1 - t)),
			( -2*(6*(1 - t)) +  (6*t) ), 
			( 6*(1 - t) + (-2 * (6 * t)) ),
			(6 * t)])

	return bezierCoeffs



#evaluate the appropriate order Beziers at a point t

def selectOrderBezier(order, t):
	if order == 1:
		function = returnBezierLinear(t)
	elif order == 2:
		function = returnBezierQuadratic(t)

	elif order == 3:
		function = returnBezierCubic(t)

	return function

def selectOrderBezierDeriv(order, t):
	if order == 1:
		function = returnBezierDerivLinear(t)
	elif order == 2:
		function = returnBezierDerivQuadratic(t)

	elif order == 3:
		function = returnBezierDerivCubic(t)

	return function

def selectOrderBezier2ndDeriv(order, t):

	if order == 3:
		function = returnBezier2ndDerivCubic(t)

	return function





