#!/usr/bin/env python3
#-*-coding:utf-8 -*-

from casadi import *

# The single most central functionality of CasADi is AD(Algorithmic/Automatic Differentiation).

'''
Directional derivatives의 forward/reverse mode는 f(x)의 cost에 비례하게 계산됨
x의 차원에 관계없이

CasADi는 sparse jacobian matrix도 효율적으로 만들 수 있음.
복잡하지만 아래와 같은 순서를 따름
• Automatically detect the sparsity pattern of the Jacobian
• Use graph coloring techniques to find a few forward and/or directional derivatives
  needed to construct the complete Jacobian
• Calculate the directional derivatives numerically or symbolically
• Assemble the complete Jacobian

Hessians are calculated by first calculating the gradient and then performing the same
steps as above to calculate the Jacobian of the gradient in the same way as above, while
exploiting symmetry.
'''

# Syntax
'''
Jacobian과 달리 gradient는 항상 dense vector임
'''
A = SX.sym('A',3,2)
x = SX.sym('x',2)
print(A)
print(jacobian(mtimes(A,x),x))
print(gradient(dot(A,A), A))

[H,g] = hessian(dot(x,x), x)
print(H)
print(g)

# (jtimes) Jacobian-times-vector product
'''
jtimes는 forward mode AD를 수행함.
종종 full Jacobian을 만들거나 &  matrix-vector 곱을 수행하는 것보다 효율적임

또한 reverse AD = transposed-Jacobian-times-vector product 가능
'''
v = SX.sym('v',2)
f = mtimes(A,x)
print(jtimes(f,x,v))

w = SX.sym('w',3)
f = A @ x
print(jtimes(f,x,w,True))