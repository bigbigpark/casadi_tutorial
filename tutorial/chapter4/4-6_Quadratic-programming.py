#!/usr/bin/env python3
#-*-coding:utf-8 -*-

from casadi import *
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab

'''
QP를 푸는 interface도 제공함.
qpOASES, OOQP, 또는 상업용인 CPLEX and GUROBI도 제공함
'''

'''
[CasADi에서 QP를 푸는 2가지 방법]
1. High-level interface
2. Low-level interface
'''

# 1. High-level interface
''''
NLP와 똑같음
1. Objective function f(x,p)는 반드시 convex quadratic function in x 이어야함
2. Constraint function g(x,p)는 linear in x이어야 함.
3. initial guess x로부터 linearization point에서 해가 구해짐

만약 objective function이 convex하지 않으면 solver는
해를 찾을 수도 있고, 못 찾을 수도 있고 또 찾더라도 여러 개일 수 도 있다.
'''

'''
min x^2 + y^2
x,y
s.t   x+y-10 >= 0

QP를 풀기 위해서 NLP에서는 "nlpsol"을 썻지만 여기서는 "qpsol"
솔버는 qpOASES 사용할 거임
'''

x = SX.sym('x')
y = SX.sym('y')
qp = {
  'x': vertcat(x,y), # decision variable
  'f': x**2 + y**2,
  'g': x+y-10
}
S = qpsol('S', 'qpoases', qp)
r = S(lbg=0)
x_opt = r['x']
print('x_opt: {}'.format(x_opt))


# 2. Low-level interface
'''
min 0.5 x.T*H*x + g.T*x
x
s.t.     x_lb <=  x <= x_up
         a_lb <= Ax <= a_up
'''
H = 7*DM.eye(2)
A = DM.ones(1,2)
g = DM.zeros(2)
lba = 10

'''
solver instance를 생성하기 위해서 QP를 위한 symbolic expression을 넘겨주는 대신에,
우리는 matrix H,A의 sparsity pattern을 넘겨줄 거임.
우리는 CasADi의 DM-type을 사용했기 때문에 쉽게 sparsity pattern을 query 할 수 있다!
'''
qp = {}
qp['h'] = H.sparsity()
qp['a'] = A.sparsity()
S = conic('S','qpoases',qp)

'''
반환받은 S 함수는 high-level에서 받았던 것이랑 비교했을 때 input/output signature가 다를 것이다.
'''
r = S(h=H, g=g, a=A, lba=lba)
x_opt = r['x']
print(x_opt)