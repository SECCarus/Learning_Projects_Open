
% % 目标函数（电车和电池调度仿真模型）
function [static,s1_resource,s2_resource] = cal_obj_val_based_on_simulation_model(xj,Tj,k,time_long,PD,SS1,threshold,max_car_num,max_battery_num,if_draw,if_show)
    
    % 道路模型如下：
    % S2___空载_____P_____满载______S1_____满载_____D______空载____S2
    % 我们的目的是计算发车策略的卸货次数和充电次数
    % |_____| 我们以每一个时间段的末端作为判断
    % 1个发车时刻（即相邻两次决策之间相隔的时间）,即0.2min
    % 行走距离 200m

    % % 客观常量
    v = 1; % 1km/min
    c_v1 = 0.01 / 3; % 空载耗电 0.01 / 3 min
    c_v2 = 0.01 / 2; % 载货耗电 0.01 / 2 min
    bty_num_per_car = 6;

    % % 车群状态
    dist_k = zeros(1,max_car_num); % 每辆车的行驶距离
    soc_k = ones(1,max_car_num); % 每辆车的电量状态
    
    car_empty = ones(1,max_car_num); % 处于空载车辆
    car_load = zeros(1,max_car_num); % 处于满载车辆

    % % 资源分配策略
%     resource_prop = length(find(xj > 0)) / max_car_num;
    resource_prop = 0.5;
    fast_charge_prop = 0; % 快充比例
    
    % % 设置节点资源（可以考虑给P和D设置资源）
    s1_resource = struct();
    s2_resource = struct();

    s2_resource.fully_soc_car = floor((max_car_num - length(find(xj > 0))) * resource_prop);
    s2_resource.empty_soc_car = 0;
    s2_resource.wait_restart_car = 0; % 属于空电车辆，但属于原定行驶车辆
    s2_resource.wait_exchange_good_car = 0; % 准备更换货物的车辆
    s2_resource.wait_exchange_car = 0 ; % 准备空载换车的车辆
    s2_resource.charging_bty = 0;
    s2_resource.wait_replace_bty = 0; % 准备卸下来的电池
    s2_resource.empty_bty = 0;
    s2_resource.fully_bty = floor((max_battery_num - max_car_num * 6) * resource_prop);

    s1_resource.fully_soc_car = max_car_num - length(find(xj > 0)) - s2_resource.fully_soc_car;
    s1_resource.empty_soc_car = 0;
    s1_resource.wait_restart_car = 0; % 属于空电车辆，但属于原定行驶车辆
    s1_resource.wait_exchange_good_car = 0; % 准备更换货物的车辆
    s1_resource.wait_exchange_car = 0 ; % 准备空载换车的车辆
    s1_resource.wait_replace_bty = 0; % 准备卸下来的电池
    s1_resource.charging_bty = 0;  
    s1_resource.empty_bty = 0;
    s1_resource.fully_bty = (max_battery_num - max_car_num * 6) - s2_resource.fully_bty;
    
    % 电池组模6调整
    ret = mod(s1_resource.fully_bty,6);
    if ret ~= 0
        s1_resource.fully_bty = s1_resource.fully_bty + (6 - ret);
        s2_resource.fully_bty = s2_resource.fully_bty - (6 - ret);
    end

    % 需要进行事件的车辆ID
    need_load_id = []; % 需要装货
    need_unload_id = []; % 需要卸货
    
    need_exchange_car_id = []; % 子事件，需要交换车辆的车辆
    need_exchange_good_id = []; % 子事件，需要移动货物的车辆
    need_replace_bty_id = []; % 子事件，需要更换电池的车辆

    need_wait_restart_car_id = []; % 等待重新出发的车辆
    need_wait_charging_pile_finish_id = [];% 等待充电完成的充电桩编号

    % % 正在进行事件
    % 超事件,正在进行电池调度决策的车辆
    % 0 代表未执行策略，1代表S1在执行策略，2代表S2在执行策略
    record_on_bty_strategy = zeros(1,max_car_num); 
    % 超事件,正在使用的充电桩
    % 0 代表该充电桩未在使用，1代表位于S1，2代表位于S2
    record_on_charging_pile = zeros(1,max_battery_num / bty_num_per_car);
    % 0 代表不处于等待出发状态，1代表位于S1，2代表位于S2
    record_wait_restart_car = zeros(1,max_car_num); 

    % 计时器，倒计时，为0时代表未在进行此事件
    % 正在装卸货的车辆
    on_load = zeros(1,max_car_num); % 装货，计时器
    on_unload = zeros(1,max_car_num); % 卸货，计时器
    on_charging_pile = zeros(1,max_battery_num / bty_num_per_car); % 正在充电的电池，后期可以安排快充
    
    % 子事件,计时器
    on_replace_bty = zeros(1,max_car_num); % 子事件 正在更换电池
    on_excharge_goods = zeros(1,max_car_num); % 子事件 正在移动货物
    on_excharge_car = zeros(1,max_car_num); % 子事件 正在交换车辆
    on_wait_restart_car = zeros(1,max_car_num); % 子事件，等待重启的车辆

    % % 各个事件的总执行时间
    load_time = 1 ; % 2min
    unload_time = 1;
    charging_time = 3 * 60 ;
    excharge_good_time = 2;
    replace_bty_time = 2;
    excharge_car_time = 0; % 交换车辆时间
    wait_restart_time = 3 * 60;

    % % 统计量
    static = struct();
    static.load_num = 0; % 装卸货次数
    static.max_charging_piles_used_num = 0; % 充电桩最大使用量
    static.max_wait_start_car_num = 0;
    static.s1_wait_start_car_num = 0;
    static.s2_wait_start_car_num = 0;
    static.s1_charged_finish_times = 0; % s1 换电站累计充电完成数
    static.s2_charged_finish_times = 0; % s2 换电站累计充电完成数
    static.total_charged_finish_time = static.s2_charged_finish_times + static.s1_charged_finish_times;
    static.xj_start = xj;
    static.Tj_start = Tj;
    static.end_time = 1; % 终止时间
    static.total_fully_soc_car = s1_resource.fully_soc_car + s2_resource.fully_soc_car; % 满电待机车辆
    static.total_empty_soc_car = s1_resource.empty_soc_car + s2_resource.empty_soc_car; % 空点车辆
    static.total_wait_restart_car = s1_resource.wait_restart_car + s2_resource.wait_restart_car; % 等待重启车辆
    static.total_wait_exchange_good_car = s1_resource.wait_exchange_good_car + s2_resource.wait_exchange_good_car; % 等待交换货物的车辆
    static.total_wait_exchange_car = s1_resource.wait_exchange_car + s2_resource.wait_exchange_car; % 等待交换的车辆
    static.total_wait_replace_bty = s1_resource.wait_replace_bty + s2_resource.wait_replace_bty; % 等待从车辆上更换下来的电池
    static.total_charging_bty = s1_resource.charging_bty + s2_resource.charging_bty; % 正在充电电池
    static.total_empty_bty = s1_resource.empty_bty + s2_resource.empty_bty; % 空电电池
    static.total_fully_bty = s1_resource.fully_bty + s2_resource.fully_bty; % 满电电池
    static.On_road_car = length(find(xj > 0.000001)); % 正在行驶车辆
    static.total_car = (static.On_road_car + static.total_fully_soc_car + static.total_empty_soc_car + static.total_wait_restart_car + static.total_wait_exchange_good_car); % 总车辆数
    static.total_bty = static.total_car * 6 + static.total_wait_replace_bty + static.total_charging_bty + static.total_empty_bty + static.total_fully_bty; % 总电池数
    static.min_car_soc = 1; % 最低充电量

    % % 开启仿真
    i = 1;
    % S2___空载_____P_____满载______S1_____满载_____D______空载____S2

    %%%%%%%%在不同地址出发的情况下，令S1出发车辆满载出发%%%%%%%%
    if ~isempty(find(xj == 2,1))
        id = find(xj == 2);
        dist_k(id) = SS1(1);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    while i < time_long

        % 得到已经发车并开始行驶的车辆的编号
        index = find((Tj(k,:) <= i) .* (Tj (k,:) > 0) .* (xj(k,:) > 0));

        if ~isempty(index)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%上端点——普通事件判断公式%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % 确认车辆空载、满载状态
            car_load = double(mod(dist_k,20) > PD(1)) .* double(mod(dist_k,20) < PD(2));
            car_empty = double(mod(dist_k,20) > PD(2)) + double(mod(dist_k,20) < PD(1)) .* double(mod(dist_k,20) >= 0);

            % 检查是否有新的车辆需要装货
            id = find((mod(dist_k,20) < (PD(1) .* 1.00001)) .* (mod(dist_k,20) > (PD(1) .* 0.999999)) .* (on_load < 0.0001));
            on_load(id) = load_time; % 设置装货时间
            need_load_id = unique([need_load_id,id]); % 排序并消去重复序号

            % 检查是否有新的车辆需要卸货
            id = find((mod(dist_k,20) <= PD(2)*1.00001) .* (mod(dist_k,20) >= (PD(2) .* 0.99999)) .* (on_unload < 0.0001));
            on_unload(id) = unload_time;  
            need_unload_id = unique([need_unload_id,id]);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%上端点——充电桩资源调度%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % 检查是否有新电池组需要充电(含S1和S2的电池组)
            % 注意，这里同时完成了事情安排工作
            % 且充电桩资源充足，忽略电池安装时间
            % S1换电站
            num = s1_resource.empty_bty / 6;
            if s1_resource.empty_bty / 6 > 0
                % 安排事件
                id = find(on_charging_pile < 0.00001); % 提取出空的充电桩序号
                sub_id = id(1:num); % 提取出前n(需要)个充电桩序号
                record_on_charging_pile(sub_id) = 1;
                on_charging_pile(sub_id) = charging_time;
                
                % 资源调整
                s1_resource.charging_bty = s1_resource.charging_bty + s1_resource.empty_bty;
                s1_resource.empty_bty = 0;

                need_wait_charging_pile_finish_id = unique([need_wait_charging_pile_finish_id,sub_id]);
            end
            
            % 更新空电车辆状态
            if s1_resource.empty_soc_car > 0 && s1_resource.fully_bty > 0
                if s1_resource.fully_bty >= s1_resource.empty_soc_car * 6
                    s1_resource.fully_bty = s1_resource.fully_bty - s1_resource.empty_soc_car * 6;
                    s1_resource.fully_soc_car = s1_resource.fully_soc_car + s1_resource.empty_soc_car;
                    s1_resource.empty_bty = s1_resource.empty_bty + s1_resource.empty_soc_car * 6;
                    s1_resource.empty_soc_car = 0;
                    
                else
                    s1_resource.empty_soc_car = s1_resource.empty_soc_car - floor(s1_resource.fully_bty / 6);
                    s1_resource.fully_soc_car = s1_resource.fully_soc_car + floor(s1_resource.fully_bty / 6);
                    s1_resource.empty_bty = s1_resource.empty_bty + s1_resource.fully_bty;
                    s1_resource.fully_bty = 0;
                end
            end
            
            % S2换电站
            num = s2_resource.empty_bty / 6;
            if s2_resource.empty_bty / 6 > 0
                % 安排事件
                id = find(on_charging_pile < 0.00001); % 提取出空的充电桩序号
                sub_id = id(1:num); % 提取出前n(需要)个充电桩序号
                record_on_charging_pile(sub_id) = 2;
                on_charging_pile(sub_id) = charging_time;
                % 资源调整
                s2_resource.charging_bty = s2_resource.charging_bty + s2_resource.empty_bty;
                s2_resource.empty_bty = 0;

                need_wait_charging_pile_finish_id = unique([need_wait_charging_pile_finish_id,sub_id]);
            end
            
            % 更新空电车辆状态
            % 卸下空电车辆上的空电池，并装上满电电池
            if s2_resource.empty_soc_car > 0 && s2_resource.fully_bty > 0
                if s2_resource.fully_bty >= s2_resource.empty_soc_car * 6   
                    s2_resource.fully_bty = s2_resource.fully_bty - s2_resource.empty_soc_car * 6;
                    s2_resource.fully_soc_car = s2_resource.fully_soc_car + s2_resource.empty_soc_car;
                    s2_resource.empty_bty = s2_resource.empty_bty + s2_resource.empty_soc_car * 6;
                    s2_resource.empty_soc_car = 0;
                else
                    s2_resource.empty_soc_car = s2_resource.empty_soc_car - floor(s2_resource.fully_bty / 6);
                    s2_resource.fully_soc_car = s2_resource.fully_soc_car + floor(s2_resource.fully_bty / 6);
                    s2_resource.empty_bty = s2_resource.empty_bty + s2_resource.fully_bty;
                    s2_resource.fully_bty = 0;
                end
            end

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%上端点——超事件判断公式%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % 检查新到达换电站2的车辆，是否满足更换电池条件
            id = find((soc_k > 0.1) .* (soc_k <= threshold) .* ((mod(dist_k,20) <= SS1(2)*1.001) .* (mod(dist_k,20) >= SS1(2)*0.999) + (mod(dist_k,20) < 0.0001)) .* (on_excharge_car < 0.0001) .* (on_excharge_goods < 0.0001) .* (on_replace_bty < 0.0001) .* (on_wait_restart_car < 0.0001)); 
            record_on_bty_strategy(id) = 2; % 注意这里不是时间，而是在S2执行决策
           
            %%%%%%%%%%%%%%%%%%%%%%%S2电池调度策略%%%%%%%%%%%%%%%%%%%%%%%%
            % % 位于S2时的电池调度策略，划分为子事件
            % 到达S2的车辆必定空载，优先空载换车
            if ~isempty(id)
                % 满电空车充足
                if s2_resource.fully_soc_car >= length(id) 
                    % 更新参数
                    s2_resource.fully_soc_car = s2_resource.fully_soc_car - length(id);
                    s2_resource.wait_exchange_car = s2_resource.wait_exchange_car + length(id);

                    % 分配给子事件
                    on_excharge_car(id) = excharge_car_time;
                    need_exchange_car_id = unique([need_exchange_car_id,id]);

                % 满电车辆不够分配，电池资源充足 
                elseif (s2_resource.fully_bty >= (length(id) - s2_resource.fully_soc_car) * 6) 
                    
                    ptr = 0;
                    if s2_resource.fully_soc_car > 0
                        sub_id = id(1:s2_resource.fully_soc_car);
                        s2_resource.fully_soc_car = s2_resource.fully_soc_car - length(sub_id);
                        s2_resource.wait_exchange_car = s2_resource.wait_exchange_car + length(sub_id);
                        
                        on_excharge_car(sub_id) = excharge_car_time;
                        need_exchange_car_id = unique([need_exchange_car_id,sub_id]);
                        ptr = ptr + length(sub_id);
                    end
                    
                    sub_id = id(ptr+1:end);
                    s2_resource.fully_bty = s2_resource.fully_bty - length(sub_id) * 6;
                    s2_resource.wait_replace_bty = s2_resource.wait_replace_bty + length(sub_id) * 6;
                    
                    on_replace_bty(sub_id) = replace_bty_time;
                    need_replace_bty_id = unique([need_replace_bty_id,sub_id]);

                % 若电池资源不足,空载车辆也不足
                elseif (s2_resource.fully_bty + s2_resource.fully_soc_car * 6) < length(id) * 6

                    ptr = 0;
                    if s2_resource.fully_soc_car > 0
                        sub_id = id(1:s2_resource.fully_soc_car);
                        s2_resource.fully_soc_car = s2_resource.fully_soc_car - length(sub_id);
                        s2_resource.wait_exchange_car = s2_resource.wait_exchange_car + length(sub_id);
                        
                        on_excharge_car(sub_id) = excharge_car_time;
                        need_exchange_car_id = unique([need_exchange_car_id,sub_id]);
                        ptr = ptr + length(sub_id);
                    end
                    
                    if s2_resource.fully_bty > 0
                        sub_id = id(ptr + 1:ptr + floor(s2_resource.fully_bty/6));
                        s2_resource.fully_bty = s2_resource.fully_bty - length(sub_id) * 6;
                        s2_resource.wait_replace_bty = s2_resource.wait_replace_bty + length(sub_id) * 6;
    
                        on_replace_bty(sub_id) = replace_bty_time;
                        need_replace_bty_id = unique([need_replace_bty_id,sub_id]);
                        ptr = ptr + length(sub_id);
                    end

                    % 剩余车辆停止行驶
                    sub_id = id(ptr + 1:end);
                    s2_resource.wait_restart_car = s2_resource.wait_restart_car + length(sub_id);
                    
                    record_wait_restart_car(sub_id) = 2; % 打上标记
                    on_wait_restart_car(sub_id) = wait_restart_time;
                    need_wait_restart_car_id = unique([need_wait_restart_car_id,sub_id]);
                    xj(sub_id) = 0; % 代表停止行驶
                end
                
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % 检查新到达换电站1的车辆，是否满足更换电池条件
            id = find((soc_k > 0.1) .* (soc_k <= threshold) .* (mod(dist_k,20) <= SS1(1)*1.001) .* (mod(dist_k,20) >= SS1(1)*0.999) .* (on_excharge_car < 0.0001) .* (on_excharge_goods < 0.0001) .* (on_replace_bty < 0.0001) .* (on_wait_restart_car < 0.0001)); 
            record_on_bty_strategy(id) = 1; % 注意这里不是时间，而是在S1执行决策

            %%%%%%%%%%%%%%%%%%%%%%%%%S1电池调度策略%%%%%%%%%%%%%%%%%%%%%%%%%
            % % 位于S1时的电池调度策略，划分为子事件
            % 到达S1的车辆必定满载，采用满载的换车策略 
            % 选择策略，哪个资源充足优先哪一个策略（因为交换货物和更换电池均是2min）
            if ~isempty(id)
                sel = 0;
                if s1_resource.fully_bty >= (s1_resource.fully_soc_car * 6)
                    sel = 1; % 电池资源较多
                elseif s1_resource.fully_bty < (s1_resource.fully_soc_car * 6)
                    sel = 2; % 空车资源较多
                end
            end

            if ~isempty(id) && (sel == 2)
                % 车辆充足
                if s1_resource.fully_soc_car >= length(id) 
                    % 更新参数
                    s1_resource.fully_soc_car = s1_resource.fully_soc_car - length(id);
                    s1_resource.wait_exchange_good_car = s1_resource.wait_exchange_good_car + length(id);

                    % 分配给子事件
                    on_excharge_goods(id) = excharge_good_time;
                    need_exchange_good_id = unique([need_exchange_good_id,id]);
                % 车辆不足，电池资源充足
                elseif (s1_resource.fully_bty >= (length(id) - s1_resource.fully_soc_car) * 6) 
 
                    ptr = 0;
                    if s1_resource.fully_soc_car > 0
                        sub_id = id(1:s1_resource.fully_soc_car);
                        s1_resource.fully_soc_car = s1_resource.fully_soc_car - length(sub_id);
                        s1_resource.wait_exchange_good_car = s1_resource.wait_exchange_good_car + length(sub_id);
                        
                        on_excharge_goods(sub_id) = excharge_good_time;
                        need_exchange_good_id = unique([need_exchange_good_id,sub_id]);
                        ptr = ptr + length(sub_id);
                    end

                    sub_id = id(ptr + 1:end);
                    s1_resource.fully_bty = s1_resource.fully_bty - length(sub_id) * 6;
                    s1_resource.wait_replace_bty = s1_resource.wait_replace_bty + length(sub_id) * 6;
                    
                    on_replace_bty(sub_id) = replace_bty_time;
                    need_replace_bty_id = unique([need_replace_bty_id,sub_id]);

                % 满电车辆资源不足，电池资源不足
                elseif (s1_resource.fully_bty + s1_resource.fully_soc_car * 6) < length(id) * 6
                    
                    ptr = 0;
                    if s1_resource.fully_soc_car > 0
                        sub_id = id(1:s1_resource.fully_soc_car);
                        s1_resource.fully_soc_car = s1_resource.fully_soc_car - length(sub_id);
                        s1_resource.wait_exchange_good_car = s1_resource.wait_exchange_good_car + length(sub_id);
                        
                        on_excharge_goods(sub_id) = excharge_good_time;
                        need_exchange_good_id = unique([need_exchange_good_id,sub_id]);
                        ptr = ptr + length(sub_id);
                    end
                    
                    if s1_resource.fully_bty > 0
                        sub_id = id(ptr + 1:ptr + floor(s1_resource.fully_bty/6));
                        s1_resource.fully_bty = s1_resource.fully_bty - length(sub_id) * 6;
                        s1_resource.wait_replace_bty = s1_resource.wait_replace_bty + length(sub_id) * 6;
    
                        on_replace_bty(sub_id) = replace_bty_time;
                        need_replace_bty_id = unique([need_replace_bty_id,sub_id]);                        
                        ptr = ptr + length(sub_id);
                    end

                    % 剩余车辆停止行驶
                    sub_id = id(ptr + 1:end);
                    s1_resource.wait_restart_car = s1_resource.wait_restart_car + length(sub_id);
                
                    record_wait_restart_car(sub_id) = 1; % 打上标记
                    on_wait_restart_car(sub_id) = wait_restart_time;
                    need_wait_restart_car_id = unique([need_wait_restart_car_id,sub_id]);
                    xj(sub_id) = 0; % 代表停止行驶
                end
                
            end

            if ~isempty(id) && (sel == 1) % 电池资源占优
                % 如果电池资源充足
                if s1_resource.fully_bty >= (length(id) * 6)
                    s1_resource.fully_bty = s1_resource.fully_bty - length(id) * 6;
                    s1_resource.wait_replace_bty = s1_resource.wait_replace_bty + length(id) * 6;

                    on_replace_bty(id) = replace_bty_time;
                    need_replace_bty_id = unique([need_replace_bty_id,id]); 
                % 电池资源不足，但满电车辆资源充足
                elseif (s1_resource.fully_soc_car * 6 >= (length(id) * 6 - s1_resource.fully_bty))

                    ptr = 0;
                    if s1_resource.fully_bty > 0
                        sub_id = id(1:floor(s1_resource.fully_bty/6));
                        s1_resource.fully_bty = s1_resource.fully_bty - length(sub_id) * 6;
                        s1_resource.wait_replace_bty = s1_resource.wait_replace_bty + length(sub_id) * 6;    
                        
                        on_replace_bty(sub_id) = replace_bty_time;
                        need_replace_bty_id = unique([need_replace_bty_id,sub_id]);

                        ptr = ptr + length(sub_id);
                    end

                    sub_id = id(ptr + 1:end);
                    s1_resource.fully_soc_car = s1_resource.fully_soc_car - length(sub_id);
                    s1_resource.wait_exchange_good_car = s1_resource.wait_exchange_good_car + length(sub_id);
                    
                    on_excharge_goods(sub_id) = excharge_good_time;
                    need_exchange_good_id = unique([need_exchange_good_id,sub_id]);
                    
                % 电池资源不足，满电车辆资源不足
                elseif (s1_resource.fully_bty + s1_resource.fully_soc_car * 6) < length(id) * 6
 
                    ptr = 0;
                    if s1_resource.fully_soc_car > 0
                        sub_id = id(1:s1_resource.fully_soc_car);
                        s1_resource.fully_soc_car = s1_resource.fully_soc_car - length(sub_id);
                        s1_resource.wait_exchange_good_car = s1_resource.wait_exchange_good_car + length(sub_id);
                        
                        on_excharge_goods(sub_id) = excharge_good_time;
                        need_exchange_good_id = unique([need_exchange_good_id,sub_id]);
                        
                        ptr = ptr + length(sub_id);
                    end
                    
                    if s1_resource.fully_bty > 0
                        sub_id = id(ptr + 1:ptr + floor(s1_resource.fully_bty/6));
                        s1_resource.fully_bty = s1_resource.fully_bty - length(sub_id) * 6;
                        s1_resource.wait_replace_bty = s1_resource.wait_replace_bty + length(sub_id) * 6;
    
                        on_replace_bty(sub_id) = replace_bty_time;
                        need_replace_bty_id = unique([need_replace_bty_id,sub_id]);                        
                        
                        ptr = ptr + length(sub_id);
                    end                    

                    % 剩余车辆停止行驶 
                    % 重启车辆不会卸下电池，改变电池数，而是直接充电
                    sub_id = id(ptr + 1:end);
                    s1_resource.wait_restart_car = s1_resource.wait_restart_car + length(sub_id);

                    record_wait_restart_car(sub_id) = 1; % 打上标记
                    on_wait_restart_car(sub_id) = wait_restart_time;
                    need_wait_restart_car_id = unique([need_wait_restart_car_id,sub_id]);                    
                    xj(sub_id) = 0; % 代表停止行驶
                end
            end

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%过程2——更新事件倒计时计时器%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % 更新装货
            on_load(need_load_id) = on_load(need_load_id) - 0.2;
            
            % 更新卸货
            on_unload(need_unload_id) = on_unload(need_unload_id) - 0.2;

            % 更新更换电池，并将资源放回原位
            on_replace_bty(need_replace_bty_id) = on_replace_bty(need_replace_bty_id) - 0.2;

            % 更新更换货物
            on_excharge_goods(need_exchange_good_id) = on_excharge_goods(need_exchange_good_id) - 0.2;

            % 更新交换车辆
            on_excharge_car(need_exchange_car_id) = on_excharge_car(need_exchange_car_id) - 0.2;

            % 更新等待车辆时间（其本身携带电池，为电池充电时间）
            on_wait_restart_car(need_wait_restart_car_id) = on_wait_restart_car(need_wait_restart_car_id) - 0.2;
            
            % 更新充电完成的电池
            on_charging_pile(need_wait_charging_pile_finish_id) = on_charging_pile(need_wait_charging_pile_finish_id) - 0.2;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%下端点——资源调度%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % 装货完成
            id = find(on_load(need_load_id) < 0.00001);
            need_load_id(id) = []; % 消去完成装货的车辆

            % 卸货完成
            id = find(on_unload(need_unload_id) < 0.00001);
            need_unload_id(id) = [];
            static.load_num = static.load_num + length(id);

            % 等待重启车辆完成(必须将该事件置于所有换电站事件顶部，否则无法检测碰撞)
            id = find(on_wait_restart_car(need_wait_restart_car_id) < 0.00001);
            if ~isempty(id)
                sub_record = record_wait_restart_car(need_wait_restart_car_id(id));
                
                % 碰撞检测
                tmp_id1 = find((on_replace_bty(need_replace_bty_id) < 0.0000001));
                tmp_id2 = find((on_excharge_goods(need_exchange_good_id) < 0.0000001));
                tmp_id3 = find((on_excharge_car(need_exchange_car_id) < 0.0000001));
                tmp_index = unique([need_replace_bty_id(tmp_id1),need_exchange_good_id(tmp_id2),need_exchange_car_id(tmp_id3)]);
                
                ret1 = 0;
                if ~isempty(find(sub_record == 1,1)) && ~isempty(tmp_index)
                    for j = 1:length(tmp_index)
                        if record_on_bty_strategy(tmp_index(j)) == 1
                            ret1 = 1; % 发生碰撞
                        end
                    end
                end

                if ret1 == 1
                    collision_car_num = length(find(sub_record == 1));
                    for j = 1:collision_car_num   
                        on_wait_restart_car(need_wait_restart_car_id(id(j))) = on_wait_restart_car(need_wait_restart_car_id(id(j))) + 0.2 * j;
                        id(j) = 0;
                    end
                    id(find(id == 0)) = [];
                    sub_record(find(sub_record == 1)) = [];
                end
                
                ret2 = 0;
                if ~isempty(find(sub_record == 2,1)) && ~isempty(tmp_index)
                   for j = 1:length(tmp_index)
                        if record_on_bty_strategy(tmp_index(j)) == 2
                            ret2 = 2; % 发生碰撞
                        end
                   end
                end

                if ret2 == 2
                    collision_car_num = length(find(sub_record == 2));
                    for j = 1:collision_car_num   
                        on_wait_restart_car(need_wait_restart_car_id(id(j))) = on_wait_restart_car(need_wait_restart_car_id(id(j))) + 0.2 * j;
                        id(j) = 0;
                    end
                    id(find(id == 0)) = [];
                    sub_record(find(sub_record == 2)) = [];
                end

                if ret1 == 1 && length(find(sub_record == 1)) > 1                  
                    sub_id = find(record_wait_restart_car(need_wait_restart_car_id(id)) == 1);
                    for j = 2:length(sub_id)
                        on_wait_restart_car(need_wait_restart_car_id(sub_id(j))) = on_wait_restart_car(need_wait_restart_car_id(sub_id(j))) + 0.2 * (j - 1);
                    end
                    id(sub_id) = [];
                    sub_id2 = sub_record(find(sub_record == 1));
                    sub_record(sub_id2(2:end)) = [];
                end
                
                if ret2 == 2 && length(find(sub_record == 2)) > 1                  
                    sub_id = find(record_wait_restart_car(need_wait_restart_car_id(id)) == 2) ;
                    for j = 2:length(sub_id)
                        on_wait_restart_car(need_wait_restart_car_id(sub_id(j))) = on_wait_restart_car(need_wait_restart_car_id(sub_id(j))) + 0.2 * (j - 1);
                    end
                    id(sub_id) = [];
                    sub_id2 = sub_record(find(sub_record == 2));
                    sub_record(sub_id2(2:end)) = [];
                end

                if  ret2 == 0 || ret1 == 0
                    [s1_resource,s2_resource] = assign_wait_restart_car(s1_resource,s2_resource,sub_record);
                    record_wait_restart_car(need_wait_restart_car_id(id)) = 0;
                    tmp_xj_start = static.xj_start; % 出发仍旧设定为原始标记，代表不同的出发点
                    xj(need_wait_restart_car_id(id)) = tmp_xj_start(need_wait_restart_car_id(id)); % 代表重新出发
                    soc_k(need_wait_restart_car_id(id)) = 1;
                    need_wait_restart_car_id(id) = [];
                end
            end

            % 更换电池完成
            id = find(on_replace_bty(need_replace_bty_id) < 0.00001);
            if ~isempty(id)
                % sub_record 记录了这些资源存储于哪个换电站，便于归位（0，1，2）
                sub_record = record_on_bty_strategy(need_replace_bty_id(id));
                [s1_resource,s2_resource] = assign_replace_bty(s1_resource,s2_resource,sub_record);
                record_on_bty_strategy(need_replace_bty_id(id)) = 0;
                soc_k(need_replace_bty_id(id)) = 1;
                need_replace_bty_id(id) = [];
            end
 
            % 车辆交换货物完成(我们认为在交换货物的东西，也在更换电池)
            id = find(on_excharge_goods(need_exchange_good_id) < 0.00001);
            if ~isempty(id)
                sub_record = record_on_bty_strategy(need_exchange_good_id(id));
                [s1_resource,s2_resource] = assign_exchange_good(s1_resource,s2_resource,sub_record);
                record_on_bty_strategy(need_exchange_good_id(id)) = 0;
                soc_k(need_exchange_good_id(id)) = 1;
                need_exchange_good_id(id) = [];
            end

            % 空载车辆交换完成
            id = find(on_excharge_car(need_exchange_car_id) < 0.00001);
            if ~isempty(id)
                sub_record = record_on_bty_strategy(need_exchange_car_id(id));
                [s1_resource,s2_resource] = assign_exchange_car(s1_resource,s2_resource,sub_record);
                record_on_bty_strategy(need_exchange_car_id(id)) = 0;
                soc_k(need_exchange_car_id(id)) = 1;
                need_exchange_car_id(id) = [];
            end

            % 取出充电完成的电池
            id = find(on_charging_pile(need_wait_charging_pile_finish_id) < 0.00001);
            if ~isempty(id)
                sub_record = record_on_charging_pile(need_wait_charging_pile_finish_id(id));
                [s1_resource,s2_resource,static] = assign_charging_finish_bty(s1_resource,s2_resource,sub_record,static);
                record_on_charging_pile(need_wait_charging_pile_finish_id(id)) = 0;
                need_wait_charging_pile_finish_id(id) = [];
            end
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%过程1-更新车辆状态%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % % 更新距离dist
            % if_updata_dist(index) 提取出发后且正在行驶的车辆编号(0-1矩阵)
            if_updata_dist = double(on_load < 0.0001) .* ...
                            double(on_unload < 0.0001) .* ...
                            double(on_replace_bty < 0.0001) .* ...
                            double(on_excharge_goods < 0.0001) .* ...
                            double(on_excharge_car < 0.0001) .* ...
                            double(on_wait_restart_car < 0.0001);
    
            dist_k(index) = dist_k(index) + 0.2 .* v .* if_updata_dist(index);
    
            % % 更新电量
            % delta_soc_k(index) 是正在进行事件的车辆编号
            delta_soc_k = 0.2 .* c_v2 .* car_load + ...
                           0.2 .* c_v1 .* car_empty;
            
            soc_k(index) = soc_k(index) - delta_soc_k(index) .* if_updata_dist(index);

        end
        

        static.s1_wait_start_car_num = [static.s1_wait_start_car_num,s1_resource.wait_restart_car];
        static.s2_wait_start_car_num = [static.s2_wait_start_car_num,s2_resource.wait_restart_car];
        
        if static.max_charging_piles_used_num  < length(find(on_charging_pile > 0))
            static.max_charging_piles_used_num = length(find(on_charging_pile > 0)); % 统计最大充电桩使用量
        end

        if static.max_wait_start_car_num < s1_resource.wait_restart_car || (static.max_wait_start_car_num < s2_resource.wait_restart_car)
            static.max_wait_start_car_num = max([s1_resource.wait_restart_car,s2_resource.wait_restart_car]);
        end

        if static.min_car_soc  > min(soc_k .* double(xj > 0))
            static.min_car_soc = length(find(on_charging_pile > 0)); % 统计最低电量
        end

        static.xj_end = xj; % 修改这里为static.xj_end = [static.xj_end;xj]; 可以统计出每个时刻在路行驶车辆数
        static.Tj_end = Tj;
        static.end_time = i;
        static.total_charged_finish_time = static.s2_charged_finish_times + static.s1_charged_finish_times;
        static.total_fully_soc_car = s1_resource.fully_soc_car + s2_resource.fully_soc_car; % 满电待机车辆
        static.total_empty_soc_car = s1_resource.empty_soc_car + s2_resource.empty_soc_car; % 空点车辆
        static.total_wait_restart_car = s1_resource.wait_restart_car + s2_resource.wait_restart_car; % 等待重启车辆
        static.total_wait_exchange_good_car = s1_resource.wait_exchange_good_car + s2_resource.wait_exchange_good_car; % 等待交换货物的车辆
        static.total_wait_exchange_car = s1_resource.wait_exchange_car + s2_resource.wait_exchange_car; % 等待交换的车辆
        static.total_wait_replace_bty = s1_resource.wait_replace_bty + s2_resource.wait_replace_bty; % 等待从车辆上更换下来的电池
        static.total_charging_bty = s1_resource.charging_bty + s2_resource.charging_bty; % 正在充电电池
        static.total_empty_bty = s1_resource.empty_bty + s2_resource.empty_bty; % 空电电池
        static.total_fully_bty = s1_resource.fully_bty + s2_resource.fully_bty; % 满电电池
        static.On_road_car = [static.On_road_car,length(find(xj > 0.000001))]; % 正在行驶车辆数变化序列
        static.total_car = (static.On_road_car(end) + static.total_fully_soc_car + static.total_empty_soc_car + static.total_wait_restart_car + static.total_wait_exchange_good_car); % 总车辆数
        static.total_bty = static.total_car * 6 + static.total_wait_replace_bty + static.total_charging_bty + static.total_empty_bty + static.total_fully_bty; % 总电池数        
        
%         disp(['当前时间',num2str(i)]);
        
         %%%%%%%%%%%% 仿真模拟 %%%%%%%%%%%%%%%
         
        if if_draw
            t = 0.2*i;                                    % 实际时间（分钟）
            dist = mod(dist_k(index),20);                 % 所有开动车辆在一维化道路的位置(总路程对20取余)
            soc = soc_k(Tj == 1)*100;                     % 第一辆车的电量



            % 绘制公路
            plot([0,10],[-1.5,-1.5],'k',[0,10],[1.5,1.5],'k','linewidth',1.5),hold on,ylim([-10,10]);

            % 绘制公路上的关键点
            x_P = 0;
            x_S1 = 2.6;
            x_D = 10;
            x_S2 = 4.6;
            plot(x_S2,1.5,'bo',x_P,0,'ro',x_S1,-1.5,'co',x_D,0,'mo','linewidth',1.5),hold on;

            text(x_S2,2.5,'S2')
            text(x_P+0.2,0.5,'P')
            text(x_S1,-2.5,'S1')
            text(x_D-0.5,0.5,'D')

            xlabel(['当前总卸货数',num2str(static.load_num),',正在充电的电池数',num2str(static.total_charging_bty),',重启车辆',num2str(static.total_wait_restart_car),' 空电车辆',num2str(static.total_empty_soc_car) ...
                ,',空电池',num2str(static.total_empty_bty),',满电车辆',num2str(static.total_fully_soc_car),',满电电池',num2str(static.total_fully_bty),',第一辆车的电量',num2str(soc),'%']);


%             set(gcf,'position',[0 0 1500 820]);         % 大屏显示
%             set(gcf,'menubar','none')                   % 去掉figure菜单栏
            set( gca, 'XTick', [0:2:10], 'YTick', [] );


            % 绘制本次迭代的车辆
            draw_car_Q2(t,dist);
            
        end
        
        

        % % 紧急事情停止
        id = find(soc_k < 0.1 * 0.99999999);
        if ~isempty(id)
            error("停机：出现低于10%电池阈值的车辆，策略无法执行");
        elseif static.total_car ~= max_car_num
            error("停机：总车辆数不对等");
        elseif static.total_bty ~= max_battery_num
            error("停机：总电池数不对等");
        end
       
        if i == time_long - 1
            i;
        end

        i = i + 1;
    end

    static.load_num = static.load_num - length(find(Tj == 2));
    static.dist_k = dist_k;
    static.soc_k = soc_k;
    
    if if_show  
        disp(['当前阶段数：',num2str(i)," 时间：",num2str(time_long*(k-1) + i * 0.2)]);
       
        s1_resource
        s2_resource
        static
        
        disp(['最初派出车辆数: ',num2str(length(find(static.xj_start > 0)))]);
        disp(['最终剩余派出车辆数: ',num2str(length(find(static.xj_end > 0)))]);
    end
end

% % 配置更换后的电池资源
function [s1_resource,s2_resource] = assign_replace_bty(s1_resource,s2_resource,sub_record)
    
    % 换电站1
    s1 = find((sub_record == 1));
    s1_resource.empty_bty = s1_resource.empty_bty + length(s1) * 6;
    s1_resource.wait_replace_bty = s1_resource.wait_replace_bty - length(s1) * 6;

    % 换电站2
    s2 = find((sub_record == 2));
    s2_resource.empty_bty = s2_resource.empty_bty + length(s2) * 6;
    s2_resource.wait_replace_bty = s2_resource.wait_replace_bty - length(s2) * 6;

end


function [s1_resource,s2_resource] = assign_exchange_good(s1_resource,s2_resource,sub_record)
    
    % 换电站1
    % 车辆交换货物完成
    s1 = find((sub_record == 1));
    s1_resource.empty_soc_car = s1_resource.empty_soc_car + length(s1);
    s1_resource.wait_exchange_good_car = s1_resource.wait_exchange_good_car - length(s1);

    % 换电站2
    s2 = find((sub_record == 2));
    s2_resource.empty_soc_car = s2_resource.empty_soc_car + length(s2);
    s2_resource.wait_exchange_good_car = s2_resource.wait_exchange_good_car - length(s2);

end

function [s1_resource,s2_resource] = assign_exchange_car(s1_resource,s2_resource,sub_record)

    % 换电站1
    % 空载车辆交换完成
    s1 = find((sub_record == 1));
    s1_resource.empty_soc_car = s1_resource.empty_soc_car + length(s1);
    s1_resource.wait_exchange_car = s1_resource.wait_exchange_car - length(s1);

    % 换电站2
    s2 = find((sub_record == 2));
    s2_resource.empty_soc_car = s2_resource.empty_soc_car + length(s2);
    s2_resource.wait_exchange_car = s2_resource.wait_exchange_car - length(s2);
    

end

function [s1_resource,s2_resource] = assign_wait_restart_car(s1_resource,s2_resource,sub_record)

    % 换电站1
    % 空载车辆交换完成
    s1 = find((sub_record == 1));
    s1_resource.wait_restart_car = s1_resource.wait_restart_car - length(s1);
    
    % 换电站2
    s2 = find((sub_record == 2));
    s2_resource.wait_restart_car = s2_resource.wait_restart_car - length(s2);
    
end

function [s1_resource,s2_resource,static] = assign_charging_finish_bty(s1_resource,s2_resource,sub_record,static)
    
    % 位于换电站1
    s1 = find((sub_record == 1));
    s1_resource.fully_bty = s1_resource.fully_bty + length(s1) * 6;
    s1_resource.charging_bty = s1_resource.charging_bty - length(s1) * 6;

    static.s1_charged_finish_times = static.s1_charged_finish_times + length(s1);

    % 位于换电站2
    s2 = find((sub_record == 2));
    s2_resource.fully_bty = s2_resource.fully_bty + length(s2) * 6;
    s2_resource.charging_bty = s2_resource.charging_bty - length(s2) * 6;  

    static.s2_charged_finish_times = static.s2_charged_finish_times + length(s2); 

end









