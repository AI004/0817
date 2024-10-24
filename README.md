# 0817

## 网页版查看urdf模型方法

打开网页`https://gkjohnson.github.io/urdf-loaders/javascript/example/bundle/`

将目录`Drawing`下`robot_urdf_12DOF`或`robot_urdf_13DOF`文件夹整体拖入网页中即可打开模型

## webots仿真环境生成

### 步骤

1. 将`urdf`模型转换为`webots`模型，`solidworks`生成`urdf`模型时，`urdf`文件名字为`robot`，在导入`webots`操作时会有冲突

2. 参考`webots_world/adam_lite_simple`中`urdf`文件中，会有关于`world`和`floating_joint`的描述，但被注释，这是正确的，当调用各自目录下`urdf2proto.sh`脚步将`urdf`模型转换为`protos`时，`world`和`floating_joint`应被注释才能顺利执行

   #### 以`DuckDuck-13DOF`为例，执行成功时输出如下

        guobb@pndrobot-REN9000K:~/Documents/GBB/0817/webots_world/DuckDuck-13DOF$ sh urdf2proto.sh
        Robot name: DuckDuck
        Directory "./protos/DuckDuck_textures" already exists!
        Root link: base
        There are 14 links, 13 joints and 0 sensors

3. `webots`中点击`File`,点击`New`,点击`New Project Directory...`新建工程，右键`Add New`添加`PROTO nodes(Webots Projects)`下面`objects`中`floor`,右键`Add New`继续添加`PROTO nodes(Current Project)`中对应执行`urdf2proto.sh`脚步生成的要添加的机器人模型，本例中为`DuckDuck(Robot)`

4. 调整导入的模型在环境中的位置，避免穿模

5. 将生成的模型用于`Control`中时，除此之外，还需要适当修改，如`添加控制器`，`定义模型名称`，更改参考`meshs`文件路径，拷贝`meshs`和`urdf`文件至相应目录等

6. 在`webotsInterface.cpp`中修改`webots`的接口

7. 功能调试

## Control说明

1. 在`./Control/Webots/controllers/mpc_controller/src/webotsInterface.cpp`文件中这一行`Waist = robot->getFromDef("DuckDuck");`中修改模型名称，其中`"DuckDuck"`为`.wbt`文件中定义的机器人模型名称。如果想导入`PND`相关的机器人模型，将`"DuckDuck"`修改为`"Adam"`

2. `DuckDuck`腿部`DH`建模参考链接:<https://blog.csdn.net/qq_60580136/article/details/136062448>

## MatLab

1. 在MatLab计算过程中，计算的结果有 `6.1232e-17` 怎么处理，参考连接:<https://blog.csdn.net/jh1513/article/details/131243390>

2. MatLab计算左腿解析解分下

    1. 把所有变换矩阵都打印出来，参考MATRIX.txt文件
    2. 根据机器人的腿部结构，`关节123`可以抽象为一个`球关节`

        方案一：<span style="color: red; font-weight: bold;">failed</span>

            尝试把`关节123`相对于`base`的变换矩阵整合，观察得到的矩阵，可以看到`T14(3,3)`为`-cos(theta3)`，可以根据单一变量原则计算得到`theta3`，得到theta3后可以根据theta3得到其他，按照矩阵内数据分布可以得到全部的解析解

        方案二：尝试对最后一个变换矩阵求逆<span style="color: red; font-weight: bold;">failed</span>

            1. T16 * T7 = T17
            2. T16_temp = T17 * T7_inv
            3. T16 = T16_temp
            4. T16(3, 2) = cos(theta3)
            5. T16_temp(3, 2) = sin(theta7)
            6. sin(theta7) = cos(theta3)
            结论：
                theta3 = theta7 - pi/2
        方案三：参考链接:<https://blog.csdn.net/fengyu19930920/article/details/81144042> <span style="color: red; font-weight: bold;">failed</span>

            1. DuckDuck的腿部结构符合Pieper准则
            2. 参考UR机械臂逆解方法，把连续三个平行的关节的旋转矩阵单独拎出来，等号左边利用矩阵求逆，构建矩阵相等，观察数据

        方案四：改为MDH模型测试

## Log 说明

### DuckDuck对应的日志说明如下所示

1. 000~000:`时间(以0.0025s递增)`
2. 001~009:`程序内部各部分时间增量`
3. 010~018:`IMURPY角位置` `IMURPY角速度` `IMURPY角加速度`
4. 019~019:`0:start` `1:zero` `2:Z2S` `3:stand` `4:S2W` `5:walk` `6:UniGait` `7:Dual2Single` `8:SingleStand` `9:Single2Dual` `10:stop` `11:swing`
5. 020~020:`各状态时间`
6. 021~041:`q_a`
7. 042~062:`q_dot_a`
8. 063~083:`tau_a`
9. 084~095:`grf`
10. 096~116:`q_c`
11. 117~137:`q_dot_c`
12. 138~156:`tau_c`
13. 159~170:`contact_force`
14. 171~176:`left_foot_position`
15. 177~182:`left_foot_velocity`
16. 183~188:`right_foot_position`
17. 189~194:`right_foot_veloctiy`
18. 195~200:`com_position`
19. 201~206:`com_velocity`
20. 207~212:`body_position`
21. 213~218:`body_velocity`
22. 219~219:`motionTurnOn`
23. 220~220:`motionNumber`
24. 221~221:`momentumTurnOn`
25. 222~222:`carryBoxState`
26. 494~496:`v_body`
27. 497~499:`euler_world`

### DuckDuck对应的关节顺序

1. `left_hip_yaw_joint`
2. `left_hip_roll_joint`
3. `left_hip_pitch_joint`
4. `left_knee_pitch_joint`
5. `left_ankle_pitch_joint`
6. `left_ankle_roll_joint`
7. `right_hip_yaw_joint`
8. `right_hip_roll_joint`
9. `right_hip_pitch_joint`
10. `right_knee_pitch_joint`
11. `right_ankle_pitch_joint`
12. `right_ankle_roll_joint`
13. `throat_pitch_joint`
14. `throat_roll_joint`
15. `throat_yaw_joint`
