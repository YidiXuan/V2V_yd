import numpy as np
import math
x = np.array([6, 1])
y = np.array([6, 1])
k = x.dot(y)
lx = math.sqrt(x.dot(x))
ly = math.sqrt(y.dot(y))
cos_value = x.dot(y) / (lx * ly)
cos_value = round(2.5555555555555, 5)
angle = np.arccos(cos_value)