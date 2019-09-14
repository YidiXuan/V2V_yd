#coding=utf-8
import matlab.engine
from numpy import *

if __name__ == '__main__':
    eng = matlab.engine.start_matlab('MATLAB_R2017b')
    A = matlab.double([[1,2],[5,6]])
    print(type(A),A.size,A)
    print(eng.eig(A))
    eng.quit()
    pass
