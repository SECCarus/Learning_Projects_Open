
% % 从xj中解析出发车时间
% interval_time 代表
function [Tj] = decoder_xj_to_Tj_v2(xj,k,interval_time)
    
    max_car_num = length(xj);
    Tj = zeros(1,max_car_num); % 初始化发车时刻
    t_m = zeros([1,interval_time]); % 用于标记该时刻已经有车
    for i = 1:max_car_num
        
         if xj(i) == 1 || xj(i) == 2 % 说明此时处于满电状态，且在换电站
            
            id = find(t_m == 0); % 寻找未使用的时刻
            sel_id = randi([1,length(id)],1); % 随机取出该时刻
            t = id(sel_id);
            t_m(t) = 1;

            Tj(1,i) = 1000 .* (k - 1) + t; % 记录当前的发车时间
            
        end
        
    end

end