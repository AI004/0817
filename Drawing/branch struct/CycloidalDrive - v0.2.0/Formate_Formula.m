clear;clc;close all;
syms t
% N   -   滚轮数量
% Rr  -   滚轮半径
% R   -   滚轮的节圆直径（PcD）
% E   -   偏心距 - 从输入轴到摆线盘的偏移量
N = 11;
Rr = 6/2;
R = 50/2;
E = 1; 
x=( R*cos(t))-(Rr*cos(t+atan(sin((1-N)*t)/((R/(E*N))-cos((1-N)*t)))))-(E*cos(N*t));
y=(-R*sin(t))+(Rr*sin(t+atan(sin((1-N)*t)/((R/(E*N))-cos((1-N)*t)))))+(E*sin(N*t));

% 将符号表达式转换为字符串
x_str = char(x);
y_str = char(y);
% 将 atan 替换为 arctan
x_str = strrep(x_str, 'atan', 'arctan');
y_str = strrep(y_str, 'atan', 'arctan');
% 输出结果
disp(x_str);
disp(y_str);