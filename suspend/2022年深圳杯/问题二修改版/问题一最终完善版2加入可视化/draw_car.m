function pic = draw_car(t,dist)

    % DRAW_PIC 用于绘制不同时间下的车况
    % t : 实际时间
    % dist : 车的位置
    
    % 绘制迭代变化的车辆
    plot(dist,zeros(1,length(dist)),'>')
    title(['实际时间',num2str(t),'min对应的车况'])
    drawnow;
    pic=getframe;
    cla;

end

