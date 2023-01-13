"""
Graceful Death Loop---a class designed to allow clean exits from infinite loops
with the potential for post-loop cleanup operations executing.

The Graceful Killer object watches for the key shutdown signals on the UNIX operating system (which runs on the PI)
when it detects a shutdown signal, it sets a flag, which is used by the Graceful Death Loop to stop iterating

the floop argument to the Graceful Death Loop's blocking_loop method is the function to be run every loop.
Note that this is intended to be a locally defined lambda expression or equivalent, so that it can influence
the relevant variables, even though it has no arguments
"""

import signal
import time
import asyncio
# print(dir(asyncio))
# print(asyncio.__name__)
# exit()

class GracefulKiller:
  kill_now = False
  def __init__(self):
    signal.signal(signal.SIGTERM, self.exit_gracefully)
    signal.signal(signal.SIGINT, self.exit_gracefully)
    signal.signal(signal.SIGHUP, self.exit_gracefully)

  def exit_gracefully(self,signum, frame):
    self.kill_now = True

class GracefulDeathLoop(object):
  def __init__(self):
    self.t0 = self.t1 = time.time()
    self.killer = GracefulKiller()

  def blocking_loop(self, floop, dt=0.01):
    self.t0 = self.t1 = time.time()
    while not self.killer.kill_now:
      floop()
      while time.time()<self.t1+dt and not self.killer.kill_now:
        asyncio.sleep(0.01)
      self.t1+=dt
    print("End of the graceful death loop. I was killed gracefully :)")

  def non_blocking_loop(self, floop, dt=0.01):
    self.t0 = self.t1 = time.time()
    async def _non_blocking_loop():
      while not self.killer.kill_now:
        ret = floop()
        if ret==0:
          self.killer.kill_now=True
        while time.time()<self.t1+dt and not self.killer.kill_now:
          asyncio.sleep(0.0001)
        self.t1+=dt
    loop = asyncio.get_event_loop()
    loop.run_until_complete(_non_blocking_loop())
    print("End of the graceful death loop. I was killed gracefully :)")

  def stop(self):
    self.killer.kill_now=True

  def time(self):
    return time.time()-self.t0

  def time_since(self):
    return time.time()-self.t1


if __name__ == '__main__':
  t0 = time.time()
  # GracefulDeathLoop().blocking_loop(lambda: print("in the loop", time.time()-t0), dt=0.1)
  GracefulDeathLoop().non_blocking_loop(lambda: print("in the loop", time.time()-t0), dt=0.1)

  