import time
from math import sqrt
class StatProfiler():
	def __init__(self, name):
		self.N, self.agg, self.aggvar = 0, 0.0, 0.0
		self.name=name

	def __del__(self):
		mean = self.agg/self.N
		try:
			stddev = sqrt((self.aggvar - self.N*mean**2)/(self.N-1)) 
		except ZeroDivisionError:
			stddev=float('NaN')
		print(f"StatProfiler {self.name}: {self.N} reps, avg: {mean*1e3} ms, stddev: {stddev*1e3} ms, total: {self.agg} s")

	def tic(self):
		self.t0=time.time()

	def toc(self):
		t = time.time()-self.t0
		self.N+=1
		self.agg+=t
		self.aggvar+=t**2

	def profile(self, func):
		self.tic()
		x = func()
		self.toc()
		return x

def main():
	StatProfiler("test1").profile(lambda: 1+1)
	p2 = StatProfiler("test2")
	x=0
	for i in range(10000):
		x=p2.profile(lambda: (x+1)**(.5))

if __name__ == '__main__':
	main()