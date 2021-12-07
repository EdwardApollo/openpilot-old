from collections import deque


class FirstOrderFilter:
  # first order filter
  def __init__(self, x0, rc, dt, initialized=True):
    self.x = x0
    self.dt = dt
    self.update_alpha(rc)
    self.initialized = initialized

  def update_alpha(self, rc):
    self.alpha = self.dt / (rc + self.dt)

  def update(self, x):
    if self.initialized:
      self.x = (1. - self.alpha) * self.x + self.alpha * x
    else:
      self.initialized = True
      self.x = x
    return self.x


class Delay:
  def __init__(self, x0, rc, dt):
    self.n = int(rc / dt)
    self.reset(x0)

  @property
  def x(self):
    return self.q[0]

  def reset(self, x0):
    self.q = deque([x0] * self.n, maxlen=self.n)

  def update(self, x):
    ret = self.q[0]
    self.q.append(x)

    return ret
