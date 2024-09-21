% 用于调整Tj,防止不同地址出发的车辆产生碰撞
function adj_Tj_for_different_origin(Tj,xj,SS1,select_car_num_at_S1,time_step)
    
    interval_time = SS1(1) / time_step;
    id1 = find(((xj == 1) .* (Tj)) > 0);
    id2 = find(((xj == 2) .* (Tj)) > 0);
    
    for i = 1:select_car_num_at_S1 
        ret = 1;
        record_time = Tj(id2(i)) - 36;
        while ret
            if record_time <= 0 
                ret = 0;
            elseif isempty(find(Tj(id1) == record_time,1))
                ret = 0;
            else 
                ok = 0;
                while ~ok
                    Tj(id2(i)) = Tj(id2(i)) + 1;
                    if length(find(Tj(id2) == Tj(id2(i)))) == 1
                        ok = 1;
                        ret = 0;
                    end
                end
            end
        end
    end

end