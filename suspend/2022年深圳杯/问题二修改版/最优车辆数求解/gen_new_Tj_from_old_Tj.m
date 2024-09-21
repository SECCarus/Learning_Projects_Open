
% % 产生新的时刻
function [Tj_x_new] = gen_new_Tj_from_old_Tj(Tj_x_old,x,k)

    Tj_x_new = Tj_x_old;
    cmp_p = [2/3 1];
    sel_p = rand(1);
    
    use_time_point = Tj_x_new(find(Tj_x_new > 0)); % 已经用到的发车时间
    not_use_time = []; % 未使用的发车时间

    for i = 1:100
        id = find(use_time_point == i);
        if isempty(id)
            not_use_time = [not_use_time,i];
        end

    end

    len1 = length(use_time_point);
    len2 = length(not_use_time);

    % 随机改变时刻
    if (sel_p < cmp_p(1)) && (len2 > 1)

        loc1 = randi([1,len1],[1,2]); % 随机选择两个使用到的时间
        loc2 = randi([1,len2],[1,2]); % 随机选择未使用到的时间           
        
        while (loc1(1) == loc1(2)) && (loc2(1) == loc2(2))
            loc1 = randi([1,len1],[1,2]); % 随机选择位置
            loc2 = randi([1,len2],[1,2]);
        end
        
        id1 = find(Tj_x_new > 0);
        id2 = id1(loc1);

        Tj_x_new(id2) = not_use_time(loc2);
    
    % 随机生成序列
    elseif (sel_p < cmp_p(2))
        Tj_x_new = decoder_xj_to_Tj(x,k);
    end

end