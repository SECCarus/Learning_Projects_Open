% % 仿真结果可视化
clc,clear,close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%模型初始化%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % 初始化车群
max_car_num = 125; % 总车辆数
max_battery_num = 900; % 总电池数
select_car_num = 100; % 总出发的车辆数
select_car_num_at_S1 = 0 ; % 选择在S1处（中间满载端点）出发的车辆数
PD = [4.6,14.6]; % 换电站与P\D点距离km
SS1 = [9.2,20]; % 到左端点到S1和右端点的距离
threshold = 14 * 0.01; % 更换电池的阈值
time_long = 60000 * 5; % 1000h = 60000min (*5是因为除以0.2)
interval_time = 100; % 前interval_time个发车时间进行发车
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % 初始化发车策略
% 生成100个发车时刻内的发车情况
disp("初始化中...")
k = 1; % 第k个阶段做出的决策
xj = zeros([1,max_car_num]);
rnd1 = randperm(max_car_num,select_car_num);
xj(rnd1) = 1; 

id = find(xj == 1);
rnd2 = randperm(select_car_num,select_car_num_at_S1);
xj(id(rnd2)) = 2; % 这里为2代表在中间换电站S1出发

Tj = decoder_xj_to_Tj_v2(xj,k,interval_time); % 初始化发车时刻

% id = find(Tj == 1);
% xj(id) = 2;

adj_Tj_for_different_origin(Tj,xj,SS1,select_car_num_at_S1,0.2); % 用于调整发车时间，防止碰撞

disp(['初始化完成,初始化车辆：',num2str(length(find(xj > 0.000001)))]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[static,s2_resource] = cal_obj_val_based_on_simulation_model(xj,Tj,k,time_long,PD,SS1,threshold,max_car_num,max_battery_num,1,1);








