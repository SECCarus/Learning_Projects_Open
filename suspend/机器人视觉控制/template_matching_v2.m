%{
    input raw_img:待匹配的图像
          raw_cmp_img:匹配模板
    return center ：中心坐标
           findloc : 匹配的位置
%}  


% % 模板匹配函数
function [findloc,center,p_all,theta,check_work] = template_matching_v2(raw_img,raw_cmp_img,test_mode)
    
    % % 属性设置
    findloc = [];
    k = 3;
    center = [-720,-720];
    p_all = zeros(2,4);
    theta = -720;

    % % 导入模板
    cmp_img = imresize(raw_cmp_img,1/k);
    img = imresize(raw_img,1/k);
    
    % % hsv空间变换
    [h1,s1,v1] = rgb2hsv(img);
    [h2,s2,v2] = rgb2hsv(cmp_img);

    
    % % 红色物体识别
    h_cmp_value = mean(mean(h2)); % 取出匹配物的h特征值
    s_cmp_value = mean(mean(s2)); % 取出s特征值

    if h_cmp_value < 0.5
        roi_img = double(img) .* (h1 < h_cmp_value) .* (s1 > s_cmp_value);
    else
        roi_img = double(img) .* ((h1 > h_cmp_value) + (h1 < 0.02)) .* (s1 > s_cmp_value);
    end
    
    index = find(roi_img ~= 0);
    roi_img(index) = 255;

    % % 形态学处理
    se3 = strel('rectangle',[3 3]);
    se5 = strel('rectangle',[5 5]); % 扫描核
    se9 = strel('rectangle',[9 9]); % 扫描核

    roi_img = imfill(roi_img,"holes"); ;% 填充
    roi_img = imopen(roi_img,se5); % 开运算，消去噪声
    roi_img = imdilate(roi_img,se5); % 膨胀
    roi_img = imfill(roi_img,"holes"); % 填充


    % % 轮廓提取
    roi_img = rgb2gray(roi_img);
    img_edge = edge(roi_img,"canny");
    
    % % 图像群中心计算
    [Y,X] = find(img_edge ~= 0);% 中心(颠倒回来)
    
    Y = Y * k;
    X = X * k;

    edge_point = sortrows([X,Y]);% 按行进行排序
    
    sort_X = sort(X);
    sort_Y = sort(Y);

    max_x = max(X);
    min_x = min(X);
    max_y = max(Y);
    min_y = min(Y);
    
    center = round([mean(sort_X(2:end-1)),mean(sort_Y(2:end-1))]); 
    delta_x = max_x - min_x;
    delta_y = max_y - min_y;
    left_top_y = round(center(2) - delta_y/2); % 这里记得调换顺序
    left_top_x = round(center(1) - delta_x/2);
    
    % % 判断是否检测到图片
    if isempty(X) | isempty(Y) | center(1) == -720 | center(2) == -720 | isnan(center(1)*center(2))
        findloc = [0 0 0 0];
        check_work = 0;
    else
        % findloc 指定为 [x y w h]
        findloc = [left_top_x,left_top_y,delta_x,delta_y];
        check_work = 1;
    end

    if check_work
        [p_all theta] = cal_inclined_rect_v2(edge_point,center); % 计算倾斜矩阵的角度
    end
    
    if test_mode
        % 测试代码
        figure()
        img_edge = imresize(img_edge,k);
        imshow(img_edge);
    
        figure()
        imshow(raw_img);
        hold on;
        rectangle("Position",findloc,EdgeColor='b',LineWidth=3); % 绘制出匹配的位置

        figure()
        imshow(raw_img);
        hold on;
        plot(p_all(:,1),p_all(:,2),'b*',MarkerSize=6);
%         text(p_all(:,1),p_all(:,2),["1","2","3","4","5","6","7","8","9","10","11","12"],FontSize=10);
        
        figure()
        imshow(raw_img);
        hold on;
        plot(edge_point(:,1),edge_point(:,2),'g*',MarkerSize=6)

        drawnow;
    end
    
end



