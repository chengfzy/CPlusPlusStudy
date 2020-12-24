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
    print(f'==================== Ex01 ====================')
    import pybind01_Basic

    print(f'what = {pybind01_Basic.what}, answer = {pybind01_Basic.answer}')
    print(f'result01 = {pybind01_Basic.add()}')
    print(f'result02 = {pybind01_Basic.add(10, 20)}')
    print(f'result03 = {pybind01_Basic.add(i=10)}')


def ex02():
    """Example of pybind02_Class"""
    print(f'==================== Ex02 ====================')
    import pybind02_Class
    p = pybind02_Class.Pet('Mimi')
    # repr
    print(p)
    # function
    print(p.getName())
    p.setName('Mimix')
    print(p.getName())
    # property
    print('property: ', p.name)
    p.name = 'Mimiy'
    print('property: ', p.name)
    # dynamic attributes
    # p.age = 5
    # print(p.__dict__)

    # inheritance
    p2 = pybind02_Class.Dog("Pet")
    print(p2.bark())


if __name__ == "__main__":
    # ex01()
    ex02()