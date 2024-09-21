
% % 计算倾斜矩形角度
% % X,Y 是按行排序的

function [p_all theta] = cal_inclined_rect_v2(edge_point,center)
    
    X = edge_point(:,1);
    Y = edge_point(:,2);
    sort_Y = sort(Y);
    
    max_x = X(end);
    min_x = X(1);
    max_y = sort_Y(end);
    min_y = sort_Y(1);
    
    
    p1 = [min(X(find(Y == max_y))),max_y];
    p2 = [max(X(find(Y == max_y))),max_y];
    p3 = [max_x,min(Y(find(X == max_x)))];
    p4 = [max_x,max(Y(find(X == max_x)))];
    p5 = [min(X(find(Y == min_y))),min_y];
    p6 = [max(X(find(Y == min_y))),min_y];
    p7 = [min_x,min(Y(find(X == min_x)))];
    p8 = [min_x,max(Y(find(X == min_x)))];

    p9 = (p1 + p2)/2;
    p10 = (p5 + p6)/2;
    p11 = (p3 + p4)/2;
    p12 = (p7 + p8)/2;
    
    p_all = [p1;p2;p3;p4;p5;p6;p7;p8;p9;p10;p11;p12];

    vector1 = p9 - p12;
    vector2 = p11 - p10;

    theta = acosd(sum(vector2 .* vector1)/(norm(vector1) * norm(vector2)));

%     if theta < 7 & theta > 0
%         fprintf("倾斜 %f \n",theta)
%     else
%         fprintf("水平或竖直 %f \n",theta)
%     end
end