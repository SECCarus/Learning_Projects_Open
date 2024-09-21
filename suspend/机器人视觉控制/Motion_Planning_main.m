close all;
clear;

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
th = [0, 90, 0, 0, 0, 0]'; % 初始状态

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

% % 机器人动作规划
DHfk3Dof_Lnya(th(1), th(2), th(3), th(4), th(5), th(6), 0); % 绘制初状态（ifcla=0,不擦除）
view(134, 12);
spd = 10; %电机速度（最大为10）
start_angle = [0 125 90 0 0 0 -90 25 -35]; % 运动的初始角度
end_angle = [125 0 0 90 75 -90 0 -35 25];
move_joint = [1 1 2 3 4 5 5 6 6]; % 运动对应的关节序号
stp = [5 -5 -5 5 5 -5 5 -5 5]; % 运动的关节步进值（一来一回）
% start_angle = [25 -25];
% end_angle = [-25 25];
% move_joint = [6 6];
% stp = [-5 5];
% fixed_transfor = 0;

for move_i = 1 : length(end_angle)

    % % 开始运动
    for i = start_angle(move_i) : stp(move_i) : end_angle(move_i)

        th(move_joint(move_i)) = i;

        if i == end_angle(move_i) & move_i == length(end_angle)
            DHfk3Dof_Lnya(th(1), th(2), th(3), th(4), th(5), th(6), 0 ); % 保留最后的运动情况
        else
            DHfk3Dof_Lnya(th(1), th(2), th(3), th(4), th(5), th(6), 1 ); % 绘制并擦除
        end

        if ~test_mode 
            % % 发送运动指令
            jnt = Sj2Rj(th); % 关节补偿
            for n = 1 : 1 : 6
                sendbuf = JointCmd(n, spd, jnt(n)); % 生成对应关节的控制指令（编号，速度（1-20），目标角度）
                write(Mcom, sendbuf, 'uint8'); %向串口发送关节控制指令
            end
        end

    end

    pause(0.1);

end


