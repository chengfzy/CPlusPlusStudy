% Conversion among Rotation Matrix, Euler Angle
% Ref
% [1] https://blog.csdn.net/mulinb/article/details/51227597
close all
clear
clc

% angle = [Rx, Ry, Rz] = [roll, pitch, yaw]
roll = 1.0;
pitch = 0.8;
yaw = 0.5;
angle = [roll, pitch, yaw]

% Rotation Sequence: ZYX(Default), R = Rx * Ry * Rz;
R0 = eul2rotm([yaw, pitch, roll])
angle0 = rotm2eul(R0)


% Rotation Sequence: XYZ, R = Rz * Ry * Rx;
R1 = eul2rotm([roll, pitch, yaw], 'XYZ')
angle1 = rotm2eul(R1, 'XYZ')


% Rotation Matrix => Rotation Angle(Euler-Rodrigues Vector)
angle = rotationMatrixToVector(R0)