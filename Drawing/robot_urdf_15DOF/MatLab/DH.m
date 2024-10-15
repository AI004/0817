%% 基于MATLAB的关节型六轴机械臂仿真
clear;
close all;
clc;
%% 建模
% D-H参数表
theta1 = pi/2;   D1 = 0;   A1 = 1;  alpha1 = 0;     offset1 = 0;
theta2 = 0;      D2 = 0;   A2 = 0;  alpha2 = pi/2;  offset2 = 0;
theta3 = pi/2;   D3 = 0;   A3 = 0;  alpha3 = pi/2;  offset3 = 0;
theta4 = pi;     D4 = 1;   A4 = 1;  alpha4 = 0;     offset4 = 0;
theta5 = 0;      D5 = 0;   A5 = 1;  alpha5 = 0;     offset5 = 0;
theta6 = pi;     D6 = 0;   A6 = -1; alpha6 = -pi/2; offset6 = 0;
theta7 = 0;      D7 = 0;   A7 = 0;  alpha7 = 0;     offset7 = 0;

% DH法建立模型,关节转角，关节距离，连杆长度，连杆转角，关节类型（0转动，1移动），'standard'：建立标准型D-H参数
L(1) = Link([theta1, D1, A1, alpha1, offset1], 'standard');
L(2) = Link([theta2, D2, A2, alpha2, offset2], 'standard');
L(3) = Link([theta3, D3, A3, alpha3, offset3], 'standard');
L(4) = Link([theta4, D4, A4, alpha4, offset4], 'standard');
L(5) = Link([theta5, D5, A5, alpha5, offset5], 'standard');
L(6) = Link([theta6, D6, A6, alpha6, offset6], 'standard');
L(7) = Link([theta7, D7, A7, alpha7, offset7], 'standard');

% 定义关节范围
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
angle = [0 0 pi/6 pi/6 0 0 0];
theta = theta + angle;
robot0.teach(theta)
title('六轴机械臂模型');
view([-59.08 22.49]);
%% 正解计算
[T, allt] = robot0.fkine(theta);  % 计算末端执行器的位姿

position = T.t';% 位置 (x, y, z)
orientation = tr2rpy(T.R);  % 姿态 (roll, pitch, yaw)
offset_T= eye(4);
offset_R= rotx(0)*roty(90)*rotz(180);
offset_T(1:3,1:3) = offset_R;
disp('末端执行器的位置:');
disp(position');
disp('末端执行器的姿态 (RPY):');
disp(orientation');
disp('调整方向与base坐标系一致后，末端执行器的姿态 (RPY):');
disp(tr2rpy(T.R * offset_R)');
%% 逆解计算
% 定义符号变量
syms theta2 theta3 theta4 theta5 theta6 theta7 real

% 定义DH参数
a = [A1, A2, A3, A4, A5, A6, A7]; % 连杆长度
alpha = [sym(alpha1), sym(alpha2), sym(alpha3), sym(alpha4), sym(alpha5), sym(alpha6), sym(alpha7)]; % 连杆扭角
d = [D1, D2, D3, D4, D5, D6, D7]; % 连杆偏移
theta = [sym(pi/2), theta2, theta3, theta4, theta5, theta6, theta7]; % 关节角

% 构建并保存每个变换矩阵
for i = 1:length(a)
    % 动态生成变量名
    varName = sprintf('T%d', i);
    
    % 计算变换矩阵
    T_i = [cos(theta(i)), -sin(theta(i))*cos(alpha(i)),  sin(theta(i))*sin(alpha(i)), a(i)*cos(theta(i));
           sin(theta(i)),  cos(theta(i))*cos(alpha(i)), -cos(theta(i))*sin(alpha(i)), a(i)*sin(theta(i));
           0,              sin(alpha(i)),                cos(alpha(i)),               d(i);
           0,              0,                            0,                           1];
    
    % 将变换矩阵保存到动态生成的变量中
    eval([varName ' = T_i;']);
end

% 开启日记功能，将命令行输出保存到文件
diaryFilename = 'MATRIX.txt';
fileID = fopen(diaryFilename, 'w');
fclose(fileID);
diary(diaryFilename);

T1;
T2;
T3;
T4;
T5;
T6;
T7 = T7 * offset_T;
T17 = simplify(T1*T2*T3*T4*T5*T6*T7);

T17 = subs(T17, ...
           {theta2, theta3, theta4, theta5, theta6, theta7}, ...
           {0+pi/6, pi/2+pi/6, pi+pi/6, 0+pi/6, pi+pi/6, 0+pi/6});
RPY = tr2rpy(T17(1:3,1:3));

% 测试T15和T15_temp
T15 = simplify(T1*T2*T3*T4*T5);
for i = 1:4
    for j = 1:4
        % 动态生成变量名
        varName = sprintf('T15_%d%d', i, j);
        
        T15_ij = T15(i,j);
        
        eval([varName ' = T15_ij;']);
    end
end
T15_11
T15_12
T15_13
T15_14
T15_21
T15_22
T15_23
T15_24
T15_31
T15_32
T15_33
T15_34
T15_temp = vpa(simplify(T17 / (T6*T7)),6)
% T15_temp = simplify(T17 / (T6*T7))
% subs(T15 - T15_temp, ...
%     {theta2, theta3, theta4, theta5, theta6, theta7}, ...
%     {0+pi/6, pi/2+pi/6, pi+pi/6, 0+pi/6, pi+pi/6, 0+pi/6})

diary off;% 关闭日记功能






















