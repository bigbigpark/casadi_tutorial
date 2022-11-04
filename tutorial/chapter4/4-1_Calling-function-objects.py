#!/usr/bin/env python3
#-*-coding:utf-8 -*-

from casadi import *

# MX of function object
x = MX.sym('x',2)
y = MX.sym('y')
f = Function('f', [x,y], [x, sin(y)*x], ['x','y'],['r','q'])
print(f)

'''
scalar나 list[]도 Function object 호출할 때 대입 가능
Python에서 dict{}도 가능
'''

r0, q0 = f(1.1, 3.3)
print(r0)
print(q0)

res = f(x=1.1, y=3.3)
print(res)

arg = [1.1, 3.3]
res = f.call(arg)
print(res)

arg = {'x':1.1, 'y':3.3}
res = f.call(arg)
print(res)