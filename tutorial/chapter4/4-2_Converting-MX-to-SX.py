#!/usr/bin/env python3
#-*-coding:utf-8 -*-

from casadi import *

'''
[연산 속도와 메모리 이슈]

A function object defined by an MX graph that only contains built-in operations (e.g.
element-wise operations such as addition, square root, matrix multiplications and calls to
SX functions, can be converted into a function defined purely by an SX graph using the
syntax:
s x f u n c t i o n = m x f u n c t i o n . expand ( )
This might speed up the calculations significantly, but might also cause extra memory
overhead.
'''