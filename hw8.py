import numpy as np

x0 = []

known = [81,4,64,1,33,24,50,53,74,42,41,94,37,33,50,55,48,73,29,39]

for p in known:
    not_p = 100 - p
    temp = [not_p, p]
    x0.append(temp)

x0 = np.asarray(x0)

print(x0)
print("\n\n\n")

T = np.array([[.7, .3],
              [.3, .7]])

x1 = np.dot(x0, T)

print(x1)
print("\n\n\n")

S0 = np.array([.1, .8])

f1 = np.multiply(S0, x1)
s0 = np.sum(f1, axis=1)

for i in range(len(f1)):
    f1[i] = np.divide(f1[i], s0[i])

print(f1)
print("\n\n\n")

