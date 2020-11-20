% Conversion between rotation matrix and Euler angle
close all
clear
clc

% angle = [Rx, Ry, Rz] = [roll, pitch, yaw]
angle0a = [-0.0104279, -0.0143366, -0.00132262];
angle0b = [3.131164754, -3.127256054, 3.140270034];
angle = angle0a;

R = [0.9998963581386361, 0.001472040373203251, -0.0143215249967308;
    -0.001322483692352086, 0.9999445576072266, 0.0104466620779747;
0.01433610888546081, -0.01042663938321411, 0.9998428682414036]

% % Rotation Sequence: ZYX(Default), R = Rx * Ry * Rz;
% R01 = eul2rotm(angle)
            % angle01 = rotm2eul(R01)
    %
    %
    % % Rotation Sequence: XYZ, R = Rz * Ry * Rx;
% R02 = eul2rotm(angle, 'XYZ')
            % angle02 = rotm2eul(R02, 'XYZ')


% Rotation Matrix => Rotation Angle(Euler-Rodrigues Vector)
angle03 = rotationMatrixToVector(R)

% log: SO3 => so3
angle04skew = logm(R);
angle04 = [angle04skew(3,2), angle04skew(1,3), angle04skew(2,1)]