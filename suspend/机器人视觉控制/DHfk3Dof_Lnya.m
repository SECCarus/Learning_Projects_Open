function pic=DHfk3Dof_Lnya(th1,th2,th3,th4,th5,th6,fcla)
% close all

global Link
global joint_num

% D-H法创建机器人
Build_3DOFRobot_Lnya;
radius    = 15;
len       = 30;
joint_col = 0;
L = 30; % 局部坐标长度
large_k = 1.5; % 全局坐标长度倍数 200 * k

plot3(0,0,0,'ro'); hold on;

th = [th1,th2,th3,th4,th5,th6];
for i = 1:joint_num-1
    Link(i+1).th=th(i)*pi/180;
end

p0=[0,0,0]';

% 创建关节之间的变换矩阵
for i=1:joint_num
Matrix_DH_Ln(i); 
end

% 从第i个关节开始往后绘制
for i=2:joint_num
    % 利用相对变换，从第一个关节开始逐步往后转换
    Link(i).A=Link(i-1).A*Link(i).A; 
    Link(i).p= Link(i).A(:,4);
    Link(i).n= Link(i).A(:,1);
    Link(i).o= Link(i).A(:,2);
    Link(i).a= Link(i).A(:,3);
    Link(i).R=[Link(i).n(1:3),Link(i).o(1:3),Link(i).a(1:3)];
    Connect3D(Link(i-1).p,Link(i).p,'k',2); hold on; % 连接两个关节的原点
    DrawCylinder(Link(i-1).p, Link(i-1).R * Link(i).az, radius,len, joint_col); hold on; % 绘制关节
    % 绘制局部坐标轴
    P = Link(i).p;
    OR = Link(i).R * L;
    plot3([P(1), P(1) + OR(1,1)], [P(2), P(2)+OR(2,1)], [P(3), P(3)+OR(3,1)], 'r-', 'LineWidth', 2); hold on;
    plot3([P(1), P(1) + OR(1,2)], [P(2), P(2)+OR(2,2)], [P(3), P(3)+OR(3,2)], 'g-', 'LineWidth', 2); hold on;
    plot3([P(1), P(1) + OR(1,3)], [P(2), P(2)+OR(2,3)], [P(3), P(3)+OR(3,3)], 'b-', 'LineWidth', 2); hold on;
  
end

grid on;
% view(134,12);
view(175, 24);
axis([-200*large_k,200*large_k,-200*large_k,200*large_k,-200*large_k,200*large_k]);
xlabel('x');
ylabel('y');
zlabel('z');
drawnow;
pic=getframe; % 获得每一帧图像
if(fcla)
    cla;
end


