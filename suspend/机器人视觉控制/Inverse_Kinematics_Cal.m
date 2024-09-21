%{
  introduction : 四自由度机器人的逆运动学求解  
%}

function [th1,th2,th3,th4]=Inverse_Kinematics_Cal(px,py,pz,default_mode)
    % 单位cm
    % 本函数功能:通过几何法，由x、y、z求解关节1到4的角度（th1~4返回弧度制，alpha返回角度制）
    
    % 连杆长度，全局变量
    global L1 L2 L3 L4 L5 is_out_range alpha;
        
    % % 是否采用默认的连杆长度
    if default_mode
        % 机器人的实际高度
        A1 = 5.5; % 基座圆柱高度+第一个关节高度
        A2 = 4.5;
        A3 = 4.5;
        A4 = 4.5 + 4.5; % 考虑末端（防止撞击）
    else
        % 机器人的实际高度
        A1 = L1; % 基座圆柱高度+第一个关节高度
        A2 = L2;
        A3 = L3;
        A4 = L4 + L5;% 考虑末端（防止撞击）
    end
    
    % 角度转化
    ToRad = pi/180;
    ToDeg = 180/pi;
    
    % alpha=th2+th3+th4,其中alpha用角度制标志，便于计数
    alpha = 180;
    
    angle_up = 180;
    angle_low = 180;
    
    ANGLE_ERR = 180;
    Distance = sqrt(px^2+py^2+pz^2);
    Max_Distance = A1+A2+A3+A4;
    count = 0;                 % 计数
    
    up_true_low_false = 1;  % true
    is_out_range = 0;       % 判断是否超出范围
    
    % 判断是否满足工作空间
    if (Distance > Max_Distance) | isnan(Distance)
        th1 = -720;
        th2 = -720;
        th3 = -720;
        th4 = -720;
        disp('超出工作空间')
        return;
    end
    
    % 关节1的转角
    if px==0 & py==0
        th1 = 0;
    else
        th1=atan2(py,px);
    end
        
    % length 用于计算后续角度
    % 在x-y平面上机械臂的投影长度
    % length = py^2+px^2;
    % hight = pz;
    if px~=0
        length = px/cos(th1);
        hight = pz;
    else
        length = abs(py);
        hight = pz;
    end
    
    % 在计算关节3的基础上计算3、2、4
    while count <= ANGLE_ERR
        r_alpha = alpha*ToRad;
        L = length - A4*sin(r_alpha);
        H = hight - A4*cos(r_alpha) - A1;
        costh3 = (L^2+H^2-A2^2-A3^2)/(2*A2*A3);
        
        % 当|costh3|<1时，计算3和2和4
        if costh3^2<1
            sinth3 = sqrt(1-costh3^2);
            th3 = atan2(sinth3,costh3);
            K1 = A2 + A3*costh3;
            K2 = A3*sinth3;         
            w = atan2(K2,K1);           %辅助角
            th2 = atan2(L,H) - w;       %关节2
            th4 = r_alpha - th2 -th3;   %关节4
            
            %存在多解，用约束条件来判断
            if 0<th1 && th1<pi && th2<pi/2 && th2>-pi/2 && th3<pi/2 && th3>-pi/2 && th4<pi/2 && th4>-pi/2
                disp('满足约束的逆解结果如下：')
                
                 % 角度推坐标进行验证
                x = (A2*sin(th2)+A3*sin(th2+th3)+A4*sin(th2+th3+th4))*cos(th1);
                y = (A2*sin(th2)+A3*sin(th2+th3)+A4*sin(th2+th3+th4))*sin(th1);
                z = A1+A2*cos(th2)+A3*cos(th2+th3)+A4*cos(th2+th3+th4);
                
                 % 注意，返回值是弧度制表示，但是在这里转化为角度制print，便于观察、调试
                fprintf('th1=%4.2f \n',th1*ToDeg);
                fprintf('th2=%4.2f \n',th2*ToDeg);
                fprintf('th3=%4.2f \n',th3*ToDeg);
                fprintf('th4=%4.2f \n',th4*ToDeg);
                % 原代码的th4转为角度后-15，不理解
                % fprintf('th4=%4.2f \n',th4*ToDeg-15);
                fprintf('alpha=%4.2f \n',alpha);

                % 用所求得的角度，推正运动学结果，对求解的结果进行验证
                fprintf('x=%4.2f,y=%4.2f,z=%4.2f',x,y,z);
                return
            else
                %disp('超出约束')
                is_out_range = 1;   %设置为true，表示超出范围
            end
        end 
        
        % 当costh3大于1或者任意角度超出舵机约束范围时，试探两边的解
        if costh3^2>1 || is_out_range==1
           if up_true_low_false == 1
               angle_up = angle_up + 1;
               alpha = angle_up;
           end
           if  up_true_low_false == 0
                angle_low = angle_low - 1;
                alpha = angle_low;
                count = count+1;
           end
           up_true_low_false = 1 - up_true_low_false;
        end
    end
        
        % 循环求解无果，无解
    if count > ANGLE_ERR
        % 默认值，如果报错返回默认值，主程序判断为默认值时，将保留上一个位置而不会异常弹出
        th1 = -720;
        th2 = -720;
        th3 = -720;
        th4 = -720;
        disp('无解')
        return
    end

end


