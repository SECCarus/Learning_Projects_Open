clc,clear,close all;

test_mode = 1; % 测试模式，无机器人也可运行

if test_mode == 0
    % % 驱动连接信息
    motorCom = 6; %设置串口编号，请检查设备管理器确认
    Mcom = Com_On_2019(motorCom); %开串口，Mcom为串口结构体，波特率默认9600
else
    Mcom = -1;
end

% % 全局属性设置
global joint_num;
global L1 L2 L3 L4 L5;
ToDeg = 180 / pi;
ToRad = pi / 180;

% % 机器人属性设置
joint_num = 7 % 除去基关节便是joint_num - 1个

% 机器人的实际高度
A1 = 5.5; %基座圆柱高度+第一个关节高度
A2 = 4.5;
A3 = 4.5;
A4 = 4.5;
A5 = 4.5; % 考虑末端（防止撞击）
% 模拟机器人的长度，与实际值的比例为1:prop
prop = 10; % 放大十倍
L1 = A1 * prop;
L2 = A2 * prop;
L3 = A3 * prop;
L4 = A4 * prop;
L5 = A5 * prop;

% 初始状态
th = [0, 90, 0, 0, 0, 0]'; 
dect_p = [-50 30 100]; % 目标点
% spd = 10; % 电机速度（最大为10）

% % 机器人动作规划
DHfk3Dof_Lnya(th(1), th(2), th(3), th(4), th(5), th(6), 0); % 绘制初状态（ifcla=0,不擦除）
view(134, 12);

% % 四自由度求逆解
[th1,th2,th3,th4] = Inverse_Kinematics_Cal(dect_p(1),dect_p(2),dect_p(3),0); % 求逆解
dect_th = [th1,th2,th3,th4];
dect_th = round(dect_th * ToDeg);
% dect_th(1) = dect_th(1) + 180; % 补充关节角度，使得求解的坐标和实际坐标匹配
dect_th(2) = dect_th(2) + 90;

dect_th

% % 信息解析
start_angle = th'; % 运动的初始角度
end_angle = [dect_th th(5) th(6)]; % 这里暂时保留后两个关节不动
move_joint = [1 2 3 4 5 6]; % 运动对应的关节序号

MovementState = Create_Movement_State_Matrix(start_angle,end_angle);

% % 正运动学到达
[th] = Forward_Kinematics_Cal(MovementState,th,dect_p,Mcom)


