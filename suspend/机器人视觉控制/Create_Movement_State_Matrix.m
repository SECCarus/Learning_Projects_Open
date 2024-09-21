%{
    introduction: 根据逆运动学求解结果生成运动矩阵。
                  该矩阵可以用于关机并行
%}


function [MovementState] = Create_Movement_State_Matrix(start_angle,end_angle)

    % % 属性设置
    move_joint = [1 2 3 4 5 6]; % 运动对应的关节序号

    % % 根据期望位置和现实位置设置步长
    for i = 1:length(move_joint)
        if end_angle(i) < start_angle(i)
            stp(i) = -5; % 运动的关节步进值（一来一回）
        else
            stp(i) = 5;
        end
    end
    
    % % 根据信息生成运动矩阵
    for i = 1:length(move_joint)
        tmp1 = start_angle(i) : stp(i) : end_angle(i);
        
        % 小步长前进
        if mod(end_angle(i),stp(i))
            n_stp = stp(i)/abs(stp(i)); % 步长单位化
            tmp2 = tmp1(end): n_stp :end_angle(i);
            tmp1 = [tmp1 tmp2(2:end)];
        end
        
        cell_tmp(i) = {tmp1};
        
    end
    
    % 统计最长的运动步数
    for i = 1:length(move_joint)
        MoveSteps(i) = length(cell2mat(cell_tmp(i)));
    end
    
    MovementState = zeros(max(MoveSteps),6); % 运动矩阵
    for i = 1:length(move_joint)
        tmp = cell2mat(cell_tmp(i));
        MovementState(1:MoveSteps(i),i) = tmp;
        MovementState(MoveSteps(i)+1:end,i) = tmp(end);
    end


end