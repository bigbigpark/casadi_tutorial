#!/usr/bin/env python3
#-*-coding:utf-8 -*-

from casadi import *


'''
The Opti stack is a collection of CasADi helper classes that provides a close correspondence
between mathematical NLP notation, e.g.
'''

# min ( y - x^2 )^2

opti = casadi.Opti()
x = opti.variable()
y = opti.variable()

opti.minimize( (y-x**2)**2)
opti.subject_to(  x**2+y**2 ==1)
opti.subject_to(        x+y >= 1)

opti.solver('ipopt')
sol = opti.solve()

sol.value(x)
sol.value(y)

print(x)
print(y)

# The main characteristics of the Opti stack are:
'''
1. 구속조건에 대한 natural syntax가 가능함
'''