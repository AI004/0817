clear;clc;close all;
%% 导入模型
robot = importrobot('../DuckDuck/urdf/DuckDuck.urdf');
% show(robot);
%% 开启日记功能，将命令行输出保存到文件
diaryFilename = 'output.txt';
fileID = fopen(diaryFilename, 'w');
fclose(fileID);
diary(diaryFilename);
%% 获取关节名称和body名称
numBodies = robot.NumBodies;    % 获取body数量
jointNames = cell(1, numBodies);
bodyNames = cell(1, numBodies);
% 获取所有关节名称
for i = 1:robot.NumBodies
    body = robot.Bodies{i};
    if ~isempty(body.Joint)
        jointNames{i} = body.Joint.Name;
    end
end
% 提取并使用关节名称
joint1Name = jointNames{1};  % 提取第一个关节名称
joint6Name = jointNames{6};  % 提取第六个关节名称
% disp(['First Joint Name: ', joint1Name]);
% disp(['Sixth Joint Name: ', joint6Name]);

% 获取所有 body 名称
for i = 1:robot.NumBodies
    body = robot.Bodies{i};
    bodyNames{i} = body.Name;
end
% 提取并使用body名称
body1Name = bodyNames{1};  % 提取第一个body名称
body6Name = bodyNames{6};  % 提取第六个body名称
% disp(['First Body Name: ', body1Name]);
% disp(['Sixth Body Name: ', body6Name]);
%% 计算变换矩阵 && 计算雅可比矩阵
config = homeConfiguration(robot);              % 机器人的配置
numJoints = numel(robot.homeConfiguration);     % 获取机器人的关节数
jointAngles = [0.1, 0.1, -0.4, 0.4, 0.1, 0.1];   % 目标关节角度 
for i = 1:length(jointAngles)
    config(i).JointPosition = jointAngles(i);   % 更新关节角度
end

left_foot = 'left_ankle_roll';
right_foot = 'right_ankle_roll';
head = 'neck_yaw';
base = 'base';

T_left_foot_relative_base = getTransform(robot, config, left_foot, base)    % 计算左脚相对于base的变换矩阵
T_left_foot_relative_base(1:3,4)'
eulerAngles1 = tform2eul(T_left_foot_relative_base, 'ZYX');
eulerAngles2 = rotationMatrixToZYXEulerAngles(T_left_foot_relative_base(1:3,1:3))
T_right_foot_relative_base = getTransform(robot, config, right_foot, base); % 计算右脚相对于base的变换矩阵
T_head_relative_base = getTransform(robot, config, head, base);             % 计算头相对于base的变换矩阵

jacobian_left_foot = geometricJacobian(robot, config, left_foot);           % 计算左脚雅可比
jacobian_right_foot = geometricJacobian(robot, config, right_foot);         % 计算右脚雅可比
jacobian_head = geometricJacobian(robot, config, head);                     % 计算头部雅可比
%% 计算正逆运动学
% targetPose = T_left_foot_relative_base;                                     % 目标变换矩阵
% 
% ik = inverseKinematics('RigidBodyTree',robot);                              % 创建逆运动学求解器
% ik.RigidBodyTree
% weights = [1 1 1 1 1 1];                                                    % 设置权重
% initialguess = robot.homeConfiguration;                                     % 初始猜测角度
% [configSoln,solnInfo] = ik(left_foot,targetPose,weights,initialguess);      % 计算逆运动学解
% configSoln(1:6).JointPosition

% 测试C++使用RBDL计算 %
jointAngles = [0.15, 0.15, -0.45, 0.5, 0.15, 0.15];
for i = 1:length(jointAngles)
    config(i).JointPosition = jointAngles(i);   % 更新关节角度
end
T_left_foot_relative_base = getTransform(robot, config, left_foot, base)    % 计算左脚相对于base的变换矩阵
T_left_foot_relative_base(1:3,4)'
eulerAngles3 = tform2eul(T_left_foot_relative_base, 'ZYX');
eulerAngles4 = rotationMatrixToZYXEulerAngles(T_left_foot_relative_base(1:3,1:3))


jointAngles = [0.0790421  0.156649  -0.46605  0.564931    0.1664       0.1];

for i = 1:length(jointAngles)
    config(i).JointPosition = jointAngles(i);   % 更新关节角度
end
T_left_foot_relative_base = getTransform(robot, config, left_foot, base)    % 计算左脚相对于base的变换矩阵
T_left_foot_relative_base(1:3,4)'
eulerAngles5 = tform2eul(T_left_foot_relative_base, 'ZYX');
eulerAngles6 = rotationMatrixToZYXEulerAngles(T_left_foot_relative_base(1:3,1:3))
%% 更新机器人配置
for i = 1:6
    config(i).JointPosition = config(i).JointPosition;
end
% 显示更新后的机器人模型
% show(robot, config);
%% 关闭日记功能
diary off;