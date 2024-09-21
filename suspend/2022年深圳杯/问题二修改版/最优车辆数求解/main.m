% % 模拟退火求解最优电车和电池调度策略
clc,clear,close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%模型初始化%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % 初始化车群
max_car_num = 125; % 总车辆数
max_battery_num = 900; % 总电池数
select_car_num = 100; % 出发的车辆数
PD = [4.4,14.4]; % 换电站与P\D点距离km
SS1 = [7.0,20]; % 到左端点到S1和右端点的距离
threshold = 14 * 0.01; % 更换电池的阈值
time_long = 60000 * 5; % 1000h = 60000min (*5是因为除以0.2)

% % 初始化发车策略
% 生成100个发车时刻内的发车情况
disp("初始化中...")
k = 1; % 第k个阶段做出的决策

select_car_num = randi([60,100]);
xj = zeros([1,max_car_num]);
rnd = randperm(max_car_num,select_car_num);
xj(rnd) = 1;

Tj = decoder_xj_to_Tj(xj,k); % 初始化发车时刻
[static,s1_resource,s2_resource] = cal_obj_val_based_on_simulation_model(xj,Tj,k,time_long,PD,SS1,threshold,max_car_num,max_battery_num,1);

disp(['初始化完成,初始化车辆：',num2str(sum(xj))]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%模型超参数设置%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
iner_epoch = 100; % 内循环次数 更新 Tj_xn
outer_epoch = 30; % 次内层循环 更新解xn
max_iter = 100; % 最外层循环次数 整体循环
tolerant_wait_num = 20;

var_dim = max_car_num; % 变量维度[1,125]，125辆车
var_lb = 0;
var_hb = 1;
input_shape = [1,var_dim];

T0 = 100;
decay_coef = 0.95;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%模拟退化求解%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp(['求解中...'])

iter_x = [];
iter_y = [];
iter_T = [];

best_x = xj; % 最优解的发车数量
best_Tj = Tj; % 最优解的发车时间
fval_xn = static.load_num;
best_fval = fval_xn; % 最优解的函数值

T = T0;
xn = xj;
Tj_xn = Tj;

tic
end_i = static.end_time; % 终止时刻
t = 1;
condition = 1;
wait_num  = 1;
while condition % 外循环迭代发车数
    
    for out_n = 1:outer_epoch
    
        % % 生成新解x_new
        fprintf('*******************************************************\n');
        disp(['生成新解x_new t=',num2str(t)])
        x_new = gen_new_x_base_on_xn(xn,(1 - out_n/outer_epoch));
        while sum(x_new) >= 100 
            x_new = gen_new_x_base_on_xn(xn,(1 - out_n/outer_epoch));
        end
    
        % % 解算发车时间(每一次都不一样)
        Tj_x_new = decoder_xj_to_Tj(x_new,k); % 这里是产生发车时间的新解
        disp(['1 生成新的发车时间Tj_x_new, n=',num2str(out_n),' 最早发车的电车序号',num2str(find(Tj_x_new == min(Tj_x_new(find(Tj_x_new > 0)))))])
        disp(['1 当前车辆数',num2str(sum(x_new))])
        disp("2 计算目标函数值")
        % % 计算目标函数值
        [static,s1_resource,s2_resource] = cal_obj_val_based_on_simulation_model(x_new,Tj_x_new,k,time_long,PD,SS1,threshold,max_car_num,max_battery_num,0);
        fval_x_new = static.load_num;
        end_i = static.end_time;
        x_new_end = static.xj_end;
        Tj_x_new_end = static.Tj_end;
        
        disp(['3 当前目标函数值:',num2str(fval_x_new),'判断解的情况'])
        % % 判断是否到达终止时间
        if end_i ~= time_long
            continue % 放弃该解
        end
    
        % % 防止陷入局部最优
        re_ok = 1;
        if fval_xn < fval_x_new
            xn = x_new;
            Tj_xn = Tj_x_new;
            re_ok = 0;
            disp("4 更优解，接受新解")
        else
            Ct = 1/T;
            pt = exp(-abs(fval_x_new - fval_xn) * Ct);
            r = rand(1);
    
            disp("4 非更优解，以一定概率接受新解")
            if r < pt
                xn = x_new;
                Tj_xn = Tj_x_new;
                disp("4 接受新解")
                re_ok = 0;
            end 
        end
        
        % % 是否更新最优解
        if re_ok == 0
            fval_xn = fval_x_new;
        elseif re_ok == 1
            fval_xn;
            xn;
        end
        disp(['5 判断是否更新全局最优解 fval_xn=',num2str(fval_xn),' 原最优值best_fval：',num2str(best_fval),' 新解fval_x_new：',num2str(fval_x_new)]);
    
        disp(['6 当前最迭代解的车辆数：',num2str(sum(xn))]);
        if fval_xn > best_fval
            best_x = xn;
            best_Tj = Tj_xn;
            best_fval = fval_xn;
            disp(['7 更新完成，当前best_x的车辆数: ',num2str(sum(best_x))]);
            wait_num = 1;
        else
            disp(['7 更新失败，当前xn的车辆数: ',num2str(sum(xn)),' 当前x_new的车辆数: ',num2str(sum(x_new))]);
            wait_num = wait_num + 1;
            if wait_num == tolerant_wait_num
                disp(["wait_num=",num2str(wait_num)])
                break
            end
        
        end
    
    end
    % % 储存结果
    iter_x = [iter_x;best_x];
    iter_y = [iter_y;best_fval];
    iter_T = [iter_T;T];

    save(['解',num2str(t),'目标值',num2str(best_fval),'.mat']);

    % % 判断是否停止(可以根据需要设置停止条件)
    T = T * decay_coef;
    t = t + 1;
    if T < 0.01
        condition = 0; % 停止迭代
    end
end
toc

disp("打印最优结果...")
[static,s1_resource,s2_resource] = cal_obj_val_based_on_simulation_model(best_x,best_Tj,k,time_long,PD,SS1,threshold,max_car_num,max_battery_num,1);

static
s1_resource
s2_resource


























