%{
    introduction: 正运动学
%}

function [th] = Forward_Kinematics_Cal(MovementState,th,dect_p,Mcom)

    % % 属性设置
    spd = 5; % 电机速度（最大为10）
    move_joint = [1 2 3 4 5 6]; % 运动对应的关节序号
    
    % % 正运动学到达
    epoch = size(MovementState,1); 
    
    for i = 1 : 1 : epoch
    
        % 绘制目标点
        plot3(dect_p(1),dect_p(2),dect_p(3),'r*',MarkerSize=40); hold on;
        
        % 更新角度
        th(move_joint) = MovementState(i,:);
    
        % 判断是否为最后一步运动
        if i == epoch
            DHfk3Dof_Lnya(th(1), th(2), th(3), th(4), th(5), th(6), 0); % 保留最后的运动情况
        else
            DHfk3Dof_Lnya(th(1), th(2), th(3), th(4), th(5), th(6), 1); % 绘制并擦除
        end
        
        if Mcom ~= -1
            % 发送运动指令
            jnt = Sj2Rj(th); % 关节补偿
            for n = 1 : 1 : 6
                sendbuf = JointCmd(n, spd, jnt(n)); % 生成对应关节的控制指令（编号，速度（1-20），目标角度）
                write(Mcom, sendbuf, 'uint8'); %向串口发送关节控制指令
            end
        end

    end
        
    th = MovementState(end,:);
end