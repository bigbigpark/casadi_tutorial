#!/usr/bin/env python3
#-*-coding:utf-8 -*-

from casadi import *
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
pylab.rcParams['figure.figsize'] = 5,4
pylab.rcParams['lines.linewidth'] = 2
pylab.rcParams['font.size'] = 16
'''
CasADi안에 있는 NLP 여러 solver들이 존재함
가장 유명한것이 [IPOPT], SNOPT, WORHP, KNITRO 등
어떤 NLP를 사용하든 [solver]와 [option]은 반드시 필요함

전형적으로 NLP solver는 constrained function의 Jacobian과
x에 대해서 Lagrangian function의 Hessian을 구해주는 함수가 필요함.
'''

# Creating NLP solvers
'''
NLP solver는 CasADi 안에 있는 nlpsol 함수를 이용하여 만들어짐
'''

# Example: Rosenbrock problem min[x,y,z] (x^2 + 100z^2), s.t z+(1-x)^2-y=0 
x = SX.sym('x')
y = SX.sym('y')
z = SX.sym('z')
nlp = {
  'x': vertcat(x,y,z),  # decision variable
  'f': x**2 + 100*z**2, # objective function
  'g': z+(1-x)**2-y     # constraint
}
S = nlpsol('S', 'ipopt', nlp)

'''
solver를 이렇게 만들고 나면, initial guess를 이용해서 최적화 문제를 풀 수 있음
'''
r = S(x0=[2.5, 3.0, 0.75], lbg=0, ubg=0)
x_opt = r['x']
print('x_opt: {}'.format(x_opt))