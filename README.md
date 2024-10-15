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

2. `DuckDuck`腿部`DH`建模参考链接`https://blog.csdn.net/qq_60580136/article/details/136062448`

## MatLab

1. 在MatLab计算过程中，计算的结果有 `6.1232e-17` 怎么处理，参考连接：<https://blog.csdn.net/jh1513/article/details/131243390>

2. MatLab计算左腿解析解分下

    1. 把所有变换矩阵都打印出来，参考MATRIX.txt文件
    2. 根据机器人的腿部结构，`关节123`可以抽象为一个`球关节`

        方案一：<span style="color: red; font-weight: bold;">failed</span>

            尝试把`关节123`相对于`base`的变换矩阵整合，观察得到的矩阵，可以看到`T14(3,3)`为`-cos(theta3)`，可以根据单一变量原则计算得到`theta3`，得到theta3后可以根据theta3得到其他，按照矩阵内数据分布可以得到全部的解析解

        方案二：<span style="color: red; font-weight: bold;">not still ok</span>
            尝试对最后一个变换矩阵求逆

            1. T16 * T7 = T17
            2. T16_temp = T17 * T7_inv
            3. T16 = T16_temp
            4. T16(3, 2) = cos(theta3)
            5. T16_temp(3, 2) = sin(theta7)
            6. sin(theta7) = cos(theta3)
            结论：
                theta3 = theta7 - pi/2
