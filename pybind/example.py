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
    print(f'result = {pybind01_Basic.add(1, 2)}')


if __name__ == "__main__":
    ex01()