#!/usr/bin/env python3
#-*-coding:utf-8 -*-

from casadi import *

'''
CasADi는 function object를 만들 수 있도록 한다. C++에서는 Functor라고 불린다.
이것은 symbolic 표현, ODE/DAE integrators, QP sovler, NLP solver에서 정의된 함수를 포함한다.

[Syntax]
f = functionname(name, arguments, ..., [options])
options은 python에서는 dictionary 타입으로 넘겨주면 됨.

CasADi에서 모든 function object는 multiple matrix-valued input & output임
'''

'''
f: R^2 x R -> R^2 x R^2
'''
# SX of function object
x = SX.sym('x',2)
y = SX.sym('y')
f = Function('f', [x,y], [x, sin(y)*x])
print(f)

# MX of function object
x = MX.sym('x',2)
y = MX.sym('y')
f = Function('f', [x,y], [x, sin(y)*x], ['x','y'],['r','q'])
print(f)

'''
위에서처럼 input과 ouput의 이름을 선택해줄 수 있음 = naming
이름을 붙이는 것이 선호되는데, 이유는
• No need to remember the number or order of arguments
• Inputs or outputs that are absent can be left unset
• More readable and less error prone syntax. E.g. f.jacobian(’x’,’q’) instead of
  f.jacobian(0,1).
'''

'''
나중에 다룰 Function instances들은 expressions로부터 '바로' 만들어지는 것이 아니라,
the inputs & the outputs 가 automatically named 됨
'''