% % 深度与像素面积的方程求解
% % 找出拟合二者的函数关系

function [coef] = cal_z_coef(S_threshold,Z_threshold,show_fig)
    % 生成点
%     S_threshold = [900 640*360];
%     Z_threshold = [0 190];
    
    % 利用类似绳子分段的方式生成比例因子
    % 例如 1___0.25___0.5___0.75___1
    % 最后总长度相乘，得到分段结果
    
    cut_n = 100; % 分段次数，得到是数据量是2*cut_n
    cut_stp1 = (S_threshold(2) - S_threshold(1))/(2*cut_n);
    S = S_threshold(1) : cut_stp1 : S_threshold(2);
    S = -log10(S)/2 + 2.6; % 对数线性化
    
    cut_stp2 = (Z_threshold(2) - Z_threshold(1))/(2*cut_n);
    Z = Z_threshold(1) : cut_stp2 : Z_threshold(2);
    
    % 拟合函数
    coef = polyfit(S,Z,3);
    
    if show_fig
        syms x y(x)
        y(x) = coef(1) *x*x*x + coef(2)*x*x + coef(3)*x + coef(4);
        figure()
        plot(S,Z,'r'); 
        hold on;
        plot(S,y(S),'b-');
    end

end
