# robot_arm_demo
A demo shows a 6-DoF robot arm move from a position to another position.

六轴机械臂点对点运动控制仿真
运动gif
/home/rose5710/robot_arm_demo/demo.gif

## 项目简介
一个简洁、清晰的仿真演示，展示如何控制一个六轴仿真机械臂从三维空间中的指定起始点，规划一条平滑轨迹，并精确运动到指定目标点。

## 主要特性

· 逆运动学求解：基于数值或解析方法，将目标位置转换为关节角度。
· 轨迹规划：在关节空间或笛卡尔空间生成平滑、无碰撞的运动轨迹。
· 仿真可视化：使用项目中仿真软件提供清晰的实时运动仿真。
· 简单易用：提供清晰的脚本，只需提供可达起止坐标即可运行新演示。

## 快速开始（核心部分）

环境要求

· Python 3.10.12  
· 具体见requirement.txt


## 1. 克隆仓库
git https://github.com/andunan/robot_arm_demo

## 2. 安装依赖

## 3. 运行仿真模拟程序(该项目使用的仿真模拟程序不开源,可以根据接口换成自己的模拟程序)

## 4. 在虚拟机里面运行ik_self_test.py

运行后，仿真界面将打开，机械臂将自动从预定义的起始位姿运动到目标位姿。

```
robot_arm_demo/
├── README.md               # 本文件
├── requirements.txt        # Python依赖列表
├── ik_self_test.py
├── LICENSE
├── msg
│   └── TrajectoryCommand.msg
├── package.xml
├── ros_simulator_connector.py #链接模拟器
├── src 						 #源代码
│   ├── cubic_trajectory_generation.py
│   ├── __init__.py
│   ├── publisher.py                    #发布控制消息
│   ├── save_trajectory_to_file.py
│   ├── SOARM101.py                 #正逆运动学
```

## 5.算法与原理简介
本项目实现了从笛卡尔空间到关节空间的运动控制流程：

1. 路径点设置：在三维空间中定义起始点与目标点（位姿）。
2. 轨迹插值：在两点之间插值出多个中间路径点（使用多项式或直线插值）。
3. 逆运动学（IK）求解：对每个路径点，计算出一组可行的关节角度。
4. 关节控制：控制器驱动各关节平滑地跟随计算出的角度序列。

## 常见问题

· Q：运行时提示缺少模块？  
· A：请确保已通过 pip install -r requirements.txt 安装所有依赖。

## 未来展望

· 增加障碍物环境下的避障规划。  
· 支持更复杂的轨迹形状（如圆弧、自定义曲线）。  
· 集成视觉伺服，实现“手眼”协调抓取。



