"""
The Python Example to Invoke the C++ lib
"""

import os
import sys

# add library path
sys.path.append(os.path.abspath(os.path.join(__file__, os.pardir, os.pardir, 'cmake-build-Debug/lib')))
sys.path.append(os.path.abspath(os.path.join(__file__, os.pardir, os.pardir, 'cmake-build-Release/lib')))


def ex01():
    """
    Example of pybind01_Basic
    """
    import pybind01_Basic
    print(f'what = {pybind01_Basic.what}, answer = {pybind01_Basic.answer}')
    print(f'result01 = {pybind01_Basic.add()}')
    print(f'result02 = {pybind01_Basic.add(10, 20)}')
    print(f'result03 = {pybind01_Basic.add(i=10)}')


if __name__ == "__main__":
    ex01()