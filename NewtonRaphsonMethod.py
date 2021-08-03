# -*- coding: utf-8 -*-
"""
Created on Wed Jul 28 23:08:56 2021

@author: Phoenix WANG
contanct:shadowdouble76@gmail.com
"""

from scipy.misc import derivative
import sympy as sp
def original_function(x):
    return 0.5-sp.sin(x)

def newnumb_calculation(x):
    y = original_function(x)
    dif = derivative(original_function,x, dx=1e-6)
    xnew = x - y/dif
    return xnew

def main():
    numbold = 0.2
    z = round(original_function(numbold),9)
    global i
    i = 0
    while z != 0:
        Numb = newnumb_calculation(numbold)
        z = round(original_function(Numb),9)
        numbold = Numb
        print(round(numbold,9))
    return

main()

