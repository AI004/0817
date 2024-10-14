%% 基于MATLAB的关节型六轴机械臂仿真
% Base 不控制
% EndEffort 需要增加偏置，距离地面的高度
%% clear
clear;
close all;
clc;
%% D-H参数表
theta1 = pi/2;   D1 = 0;   A1 = 1;  alpha1 = 0;     offset1 = 0;
theta2 = 0;      D2 = 0;   A2 = 0;  alpha2 = pi/2;  offset2 = 0;
theta3 = pi/2;   D3 = 0;   A3 = 0;  alpha3 = pi/2;  offset3 = 0;
theta4 = pi;     D4 = 1;   A4 = 1;  alpha4 = 0;     offset4 = 0;
theta5 = 0;      D5 = 0;   A5 = 1;  alpha5 = 0;     offset5 = 0;
theta6 = pi;     D6 = 0;   A6 = -1; alpha6 = -pi/2; offset6 = 0;
theta7 = 0;      D7 = 0;   A7 = 0;  alpha7 = 0; offset7 = 0;

%% DH法建立模型,关节转角，关节距离，连杆长度，连杆转角，关节类型（0转动，1移动），'standard'：建立标准型D-H参数
L(1) = Link([theta1, D1, A1, alpha1, offset1], 'standard');
L(2) = Link([theta2, D2, A2, alpha2, offset2], 'standard');
L(3) = Link([theta3, D3, A3, alpha3, offset3], 'standard');
L(4) = Link([theta4, D4, A4, alpha4, offset4], 'standard');
L(5) = Link([theta5, D5, A5, alpha5, offset5], 'standard');
L(6) = Link([theta6, D6, A6, alpha6, offset6], 'standard');
L(7) = Link([theta7, D7, A7, alpha7, offset7], 'standard');

%% 定义关节范围
L(1).qlim = [-pi, pi];
L(2).qlim = [-pi, pi];
L(3).qlim = [-pi, pi];
L(4).qlim = [-pi, pi];
L(5).qlim = [-pi, pi];
L(6).qlim = [-pi, pi];
L(7).qlim = [-pi, pi];

%% 显示机械臂（把上述连杆“串起来”）
robot0 = SerialLink(L, 'name', 'Left-Leg');
theta = [pi/2 0 pi/2 pi 0 pi 0];  % 初始关节角度
figure(1);
robot0.plot(theta, 'workspace', [-5, 5, -5, 5, -5, 1]);  % 设置显示范围
robot0.teach(theta)
title('六轴机械臂模型');