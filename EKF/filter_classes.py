"""Contains classes that implement various linear filters
"""
import numpy as np



class FirstOrderLinearFilter():

	"""A generic first order linear filter
	
	Attributes:
		dt (float): the filter timestep
		fc (float): the filter cutoff frequency
	"""
	
	def __init__(self, fc, dt):
		"""Initialize
		
		Args:
			fc (float): the filter cutoff frequency
			dt (float): the filter timestep
		"""
		self.fc = fc
		self.dt = dt

	def step(self,):
		"""A placeholder function meant to be overriden
		
		Returns:
			float: placeholder
		"""
		return -1



class FirstOrderLowPassLinearFilter(FirstOrderLinearFilter):

	"""A first order linear lowpass filter
	
	Attributes:
		alpha_lowpass (float): the smoothing factor
		yi (int): the filtered output
	"""
	
	def __init__(self, fc, dt):
		"""Initialize
		
		Args:
			fc (float): the filter cutoff frequency
			dt (float): the filter timestep
		"""
		super().__init__(fc, dt)
		self.alpha_lowpass = (2*np.pi*(self.dt)*self.fc)/(2 * np.pi * (self.dt) * self.fc + 1)
		self.yi = 0

	def step(self, i, xi):
		"""Step through the filter
		
		Args:
			i (int): the current iteration
			xi (float): the signal to be filtered
		
		Returns:
			float: the lowpass-filtered signal
		"""
		if i == 0:
			self.yi = xi
		else:
			self.yi = self.alpha_lowpass * xi + (1 - self.alpha_lowpass)*(self.yi)
		return self.yi

class FirstOrderHighPassLinearFilter(FirstOrderLinearFilter):

	"""Summary
	
	Attributes:
		alpha_high_pass (float): the smoothing factor
		xi_prev (int): the unfiltered signal from the previous iteration
		yi (int): the filtered output
	"""
	
	def __init__(self, fc, dt):
		"""Initialize
		
		Args:
			fc (float): the filter cutoff frequency
			dt (float): the filter timestep
		"""
		super().__init__(fc, dt)
		self.alpha_high_pass = 1/(2 * np.pi * (self.dt) * self.fc + 1)
		self.yi = 0
		self.xi_prev = None

	def step(self, i, xi):
		"""Step through the filter
		
		Args:
			i (int): the current iteration
			xi (float): the signal to be filtered
			xi_prev (float): the previous unfiltered signal
		
		Returns:
			float: the highpass-filtered signal
		"""
		if i == 0:
			self.yi = xi
		else:
			self.yi = self.alpha_high_pass * self.yi + self.alpha_high_pass*(xi - self.xi_prev)
		self.xi_prev = xi
		return self.yi


class GenericLinearFilter():

	"""A generic second order linear filter
	Contains the state space representation of a filter
	Also contains the discretized version of that filter
	
	"""
	
	def __init__(self, A, B, C, D, X0, dt=1/100):
		"""Summary
		
		Args:
			A (np matrix): state transition matrix
			B (np matrix): input matrix
			C (np matrix): measurement matrix
			D (np matrix): direct input matrix
			X0 (np matrix): initial filter states
			dt (float, optional): Time step
		
		"""
		self.A = A
		self.B = B
		self.C = C
		self.D = D
		self.dt = dt
		self.MAX_TIME_STEP = 0.05
		self.rediscretize(self.dt)
		self.Cd = C
		self.Dd = D
		self.X0d = X0
		self.X = self.X0d # (2,1)
		self.X_prev = self.X
		self.U_filt = self.X
		self.U_filt_prev = self.U_filt
		self.OVERRIDE_TIMESTEP = False

	def rediscretize(self,dt):
		"""Discretizes the continuous state space system using the Taylor series
		approximation of the matrix exponential
		
		Args:
			dt (float): the time step
		"""
		# if dt > self.MAX_TIME_STEP:
		# 	self.dt = self.MAX_TIME_STEP

		# else:
		# 	self.dt = dt
			# self.dt = np.min([dt,self.MAX_TIME_STEP])
		# self.dt = np.min([dt,self.MAX_TIME_STEP])
		self.Ad = np.eye(self.A.shape[0]) + self.A * self.dt + \
			(0.5 * self.A @ self.A * self.dt**2) + \
			(1/6 * self.A @ self.A @ self.A* self.dt**3)

		self.Bd = (self.Ad - np.eye(self.A.shape[0])) @ np.linalg.solve(self.A,self.B)


	def step(self, i, dt, U_arg):
		"""Step through the filter
		
		Returns:
			the current state, the filtered signal
		
		Args:
			i (int): iteration
			U_arg (np matrix): signal to be filtered (1,1), or a float
		"""
		# print(type(U_arg))

		#discretize
		self.OVERRIDE_TIMESTEP = False
		if dt > self.MAX_TIME_STEP:
			self.dt = self.MAX_TIME_STEP
			# self.OVERRIDE_TIMESTEP = True

		else:
			self.dt = dt

		self.rediscretize(self.dt)

		if not (isinstance(U_arg, np.ndarray) or isinstance(U_arg, np.float64) or isinstance(U_arg, np.int64)):
			U = np.array([U_arg])
		else:
			U = U_arg

		U = U.reshape(-1,1)

		assert(self.X.shape[0] == self.Ad.shape[1])
		assert(U.shape[0] == self.Bd.shape[1])

		if i == 0:
			if not self.OVERRIDE_TIMESTEP:
				self.X = (self.Ad @ self.X0d + self.Bd @ U)

			else:
				print('OVERRIDING X STATE')
				self.X = self.X_prev

			self.U_filt = U

		else:
			if not self.OVERRIDE_TIMESTEP:
				self.X = (self.Ad @ self.X + self.Bd @ U)

			else:
				print('OVERRIDING X STATE')
				self.X = self.X_prev

			self.U_filt = self.Cd @ self.X + self.Dd @ U


		self.X_prev = self.X
		return self.X, self.U_filt

	












