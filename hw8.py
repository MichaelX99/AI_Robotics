import numpy as np
from random import *

x0 = []
N = 20

# These were for my original numbers...
#known = [81,4,64,1,33,24,50,53,74,42,41,94,37,33,50,55,48,73,29,39]

for i in range(N):
    p = randint(0,100)
    p /= 100.
    #p = known[i]
    not_p = 1. - p
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


new_x1 = []
while len(new_x1) != N:
    for i in range(N):
        particle = f1[i]
        most_prob_state = np.argmax(particle)
        w = s0[most_prob_state]
        p = randint(0,100) / 100.
        if p < w:
            new_x1.append(particle)
        blag = 4
        if len(new_x1) == N:
            break

x1 = np.asarray(new_x1)
print(x1)
