function pic = draw_car_Q2(t,dist)

    % DRAW_PIC 用于绘制不同时间下的车况
    % t : 实际时间
    % dist : 车的位置

    % 绘制迭代变化的车辆
    S2P_index = find((0<dist)&(dist<4.6));                   % S2到P方向
    S2P_car = dist(S2P_index);
    plot(4.6-S2P_car,1.2*ones(1,length(S2P_car)),'<')
    
    PD_index = find((4.6<dist)&(dist<14.6));                 % P到D方向
    PD_car = dist(PD_index);                        
    plot(PD_car-4.6,-1.2*ones(1,length(PD_car)),'>')
    
    DS2_index = find((14.6<dist)&(dist<20));                 % D到S2方向
    DS2_car = dist(DS2_index);                        
    plot(24.6-DS2_car,1.2*ones(1,length(DS2_car)),'<')
      
    title(['实际时间',num2str(t),'min对应的车况'])
    drawnow;
    pic=getframe;
    cla;

end

