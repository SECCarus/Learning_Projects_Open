
% % 从旧解附近产生新解
function [x_new] = gen_new_x_base_on_xn(x,rate)
    
    x_new = x;
    max_car_num = length(x);
    change_num = 100;

    for i = 1:ceil((change_num * rate))

        loc = randi([1,max_car_num],[1,2]); % 减少车辆，或者派出车辆
        while loc(1) == loc(2)
            loc = randi([1,max_car_num],[1,2]);
        end
        x_new(loc) = ~ x_new(loc);

    end

end