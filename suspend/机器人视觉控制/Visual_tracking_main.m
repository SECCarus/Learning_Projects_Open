

clc, clear, close all;

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
save_img = 0; % 是否保持图片
save_dect_p = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%机器人属性设置%%%%%%%%%%%%%%%%%%%%%%
joint_num = 7; % 除去基关节便是joint_num - 1个

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
th = [0, 90, 0, 0, 0, 0]; 
dect_p = [-50 50 150]; % 目标点
% spd = 10; % 电机速度（最大为10）

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%相机属性设置%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % 创建相机并设置图片属性
cap = videoinput('winvideo', 1, 'RGB24_640x360');
% set(cap,"ReturnedColorSpace",'grayscale'); % 转换为灰度图
preview(cap); % 回调取图


% % 导入匹配模板
cmd2_img = imread('template_images\cmd2.jpg');
% % 计算像素面积和深度z的函数的拟合参数
workspace_range = L1+L2+L3+L4;
S_threshold = [900 640*360];
Z_threshold = [20 workspace_range];
coef = cal_z_coef(S_threshold,Z_threshold,1);
coef = coef .* (coef > 0.00001); % 精度控制

% %  图片获取
count = 0;
update_loc = 1;
location_clock = 25; % 等待时钟
area = 4000; % 默认距离
tool_signal = 1; 
tool_move_type = 1; % 奇数代表张开
tool_signal_clock = 25;
old_dect_p = dect_p;

while true
    % % 回调取图
    frame = getsnapshot(cap);
    frame = flip(frame, 2); % 按竖直轴翻转

    % % 正逆运动学
    if update_loc

        figure(2)
        subplot(1,2,1);
        view(175, 24);
        DHfk3Dof_Lnya(th(1), th(2), th(3), th(4), th(5), th(6), 0);

        % % 判断总运动是否可达
        [th1,th2,th3,th4] = Inverse_Kinematics_Cal(dect_p(1),dect_p(2),dect_p(3),0); % 求逆解
        if th1 == -720 | th2 == -720 | th3 == -720 | th4 == -720
            disp("运动不可达");
        else
            % 分段运动规划
            norm(dect_p - old_dect_p)
            if norm(dect_p - old_dect_p) > 100
                [th] = kinematic_analysis(th,dect_p,old_dect_p,Mcom,1);
            else
                [th] = kinematic_analysis(th,dect_p,old_dect_p,Mcom,0);
            end
        end

        old_dect_p = dect_p; % 保留运动位置
        update_loc = 0; % 关闭入口
    end
    
    % % 控制工具
    if tool_signal
        
        if mod(tool_move_type,2)
           start_angle = [th(1:5) 30]; % 张开
           end_angle = [th(1:5) -30]; 
        else
           start_angle = [th(1:5) -30]; % 闭合
           end_angle = [th(1:5) 30]; 
        end
        subplot(1,2,1);
        MovementState = Create_Movement_State_Matrix(start_angle,end_angle);
        th = Forward_Kinematics_Cal(MovementState,th,dect_p,Mcom);
        tool_signal = 0; % 关闭工具控制入口
        hold off;
    end
    
    % % 模板匹配
    [findloc,center,p_all,theta,check_work] = template_matching_v2(frame,cmd2_img,0);
    subplot(1,2,2);
    imshow(frame); hold on;
    
    % 如果捕获正常
    if check_work
        plot(center(1),center(2),'r*',MarkerSize=10); hold on;
        rectangle("Position",findloc,EdgeColor='b',LineWidth=3); % 绘制出匹配的位置
        hold off;
    else
        findloc = [0 0 30 30];
    end
    drawnow;

    % % 定点截取图像或者更新点坐标
    if save_img | save_dect_p
        % 是否截图
        if count == 50 
            imwrite(frame, "test.jpg");
            save_img = 0;
            count = 1; 
        % 是否更新坐标
        elseif save_dect_p 
            
            % 根据像素面积计算深度dect_pz
            area = findloc(3) * findloc(4);
            S = -log10(area)/2 + 2.6; % 对数线性化
            dect_pz = coef(1) *S*S*S + coef(2)*S*S + coef(3)*S + coef(4); 
            dect_p = [center(1)/640,center(2)/360,dect_pz/workspace_range]; % 计算占比
            
            % 扩充控制范围
            dect_p(1) = ((dect_p(1) > 0.5) + (dect_p(1) < 0.5)*(-1))*dect_p(1);

            dect_p(1) = dect_p(1) * workspace_range/2;
            dect_p(2) = dect_p(2) * workspace_range/2;
            dect_p(3) = dect_p(3) * workspace_range/3;

            if sum(abs(dect_p)) > workspace_range | sum(abs(dect_p)) < workspace_range/2
                dect_p(3) = workspace_range*3/4 - dect_p(1) - dect_p(2);
            end

            dect_p = round(dect_p)
 
            % 在机器人坐标系里绘制捕获的点
            subplot(1,2,1);
            view(175, 24);
            plot3(dect_p(1),dect_p(2),dect_p(3),'r*',MarkerSize=10); hold on;
            DHfk3Dof_Lnya(th(1), th(2), th(3), th(4), th(5), th(6), 0);
            hold off;
        end
    end

    if save_img 
        count = count + 1
    end
    
    % 如果垂直跨度小于于水平跨度，则说明水平或倾斜
    if (findloc(3) >= findloc(4)) & (area > 1200) & check_work
       tmp_center = center;
       tmp_findloc = findloc;
       tmp_p_all = p_all;

       if (theta > 0.02) & (theta < 6)
           wait_clock = [location_clock,0];
       else
           wait_clock = [tool_signal_clock,1];
       end

       % 延迟时钟，防止误识别
       key = 1; % 识别钥匙
       for i = wait_clock(1):-1:0
            frame = getsnapshot(cap);
            frame = flip(frame, 2); % 按竖直轴翻转
            [findloc,center,p_all,theta,check_work] = template_matching_v2(frame,cmd2_img,0);
            % 如果捕获正常，在二维图像上绘制点
            if check_work & (findloc(3) >= findloc(4))
                subplot(1,2,2);
                imshow(frame); hold on;
                plot(tmp_center(1),tmp_center(2),'r*',MarkerSize=10); hold on;
                rectangle("Position",tmp_findloc,EdgeColor='b',LineWidth=3); % 绘制出匹配的位置
                
                if wait_clock(2) == 0
                    title("clock: " + i + "     location cmd");
                elseif wait_clock(2) == 1
                    title("clock: " + i + "     tool signal cmd");
                    plot(tmp_p_all(:,1),tmp_p_all(:,2),'g*',MarkerSize=6);
                    hold on;
                end

                drawnow;
                hold off;
            else 
                break;
            end
            
            if i == 0 
                if wait_clock(2) == 1
                    tool_signal = 1; % 说明要发送工具控制指令
                    tool_move_type = tool_move_type + 1;
                else
                    update_loc = 1;
                end

            end

       end
       
    end

end

function [th] = kinematic_analysis(th,dect_p,old_dect_p,Mcom,cut_n)
    % % 初始化
    MovementState = [];
    ToDeg = 180 / pi;
    ToRad = pi / 180;

    if cut_n == 0 | dect_p == old_dect_p
        sub_dect_p = dect_p;
        cut_n = 0;
    else
        cut_th_stp = (dect_p - old_dect_p)/(2*cut_n);
        for i = 1:3
            tmp = [old_dect_p(i) + cut_th_stp(i):cut_th_stp(i):dect_p(i)]';
            if length(tmp) == 0
                tmp = dect_p;
            end
            sub_dect_p(:,i) = tmp;
        end
        sub_dect_p = round(sub_dect_p);
    end
    
    % % 生成分段运动矩阵
    [m,n] = size(sub_dect_p);
    for i = 1:m 
        [th1,th2,th3,th4] = Inverse_Kinematics_Cal(sub_dect_p(i,1),sub_dect_p(i,2),sub_dect_p(i,3),0); % 求逆解

       if th1 == -720 | th2 == -720 | th3 == -720 | th4 == -720 
           disp("运动不可达");
           break
       else
            % 补充关节角度，使得求解的坐标和实际坐标匹配
            tmp_dect_th = [th1,th2,th3,th4];
            tmp_dect_th = round(tmp_dect_th * ToDeg);
            tmp_dect_th(2) = tmp_dect_th(2) + 90;
            
            disp('调整前角度');
            tmp_dect_th
            for i = 1:4
                if tmp_dect_th(i) > 180
                    tmp_dect_th(i) = tmp_dect_th(i) - 360;
                end
            end
            disp('调整后角度');
            tmp_dect_th
            % % 运动路径规划
            start_angle = th'; % 运动的初始角度
            end_angle = [tmp_dect_th th(5) th(6)]; % 这里暂时保留后两个关节不动
            move_joint = [1 2 3 4 5 6]; % 运动对应的关节序号
            
            MovementState = Create_Movement_State_Matrix(start_angle,end_angle);
    
            % % 正运动学到达
            th = Forward_Kinematics_Cal(MovementState,th,dect_p,Mcom);
       end

    end

end









