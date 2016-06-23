import turbotrig
import numpy as np
from math import atan2, acos, asin, cos, sin, tan, pi, sqrt
import matplotlib.pyplot as plt
import operator

vals = np.arange(-2*pi,2*pi, 0.001)

atan2_true = [atan2(1, t) for t in vals]
atan2_approx = [turbotrig.turboatan2(1000, t*1000)/1000.0 for t in vals]
error = map(operator.sub, atan2_true, atan2_approx)
max_error = max(error)
print(max_error)

plt.figure(1)
plt.subplot(211)
plt.title("turboatan2")
plt.plot(vals,atan2_true, label="True")
plt.plot(vals,atan2_approx, label="Approx")
plt.legend()

plt.subplot(212)
plt.plot(vals, error, label="Error")
plt.ylabel("error, (rad)")

vals = np.arange(0,1, 0.001)
asin_true = [asin(t) for t in vals]
asin_approx = [turbotrig.turboasin(1000*t)/1000.0 for t in vals]
asin_error = map(operator.sub, asin_true, asin_approx)
max_asin_error = max(asin_error)

asin_lookup = [int(t*1000) for t in asin_true]
print(asin_lookup)
print(len(asin_lookup))

plt.figure(2)
plt.subplot(211)
plt.title("turboasin")
plt.plot(vals, asin_true, label="True")
plt.plot(vals, asin_approx, label="Approx")
plt.legend()
plt.subplot(212)
plt.plot(vals, asin_error, label="Error")
plt.ylabel("error, (rad)")


vals = np.arange(0, 100000, 1)
sqrt_true = [sqrt(t) for t in vals]
sqrt_approx = [turbotrig.turbosqrt(t) for t in vals]
sqrt_error = map(operator.sub, sqrt_true, sqrt_approx)
max_sqrt_error = max(sqrt_error)

sqrt_lookup = [int(t) for t in sqrt_true]
print(sqrt_lookup)
print(len(sqrt_lookup))

plt.figure(3)
plt.subplot(211)
plt.title("turbosqrt")
plt.plot(vals, sqrt_true, label="True")
plt.plot(vals, sqrt_approx, label="Approx")
plt.legend()
plt.subplot(212)
plt.plot(vals, sqrt_error, label="Error")
plt.ylabel("error, (rad)")
plt.show()




