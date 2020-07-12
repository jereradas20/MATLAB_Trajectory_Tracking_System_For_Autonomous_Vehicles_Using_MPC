%% trajectory scatter diagram
scatter(vehicle_y_,vehicle_x_,50,v_x_,'filled','MarkerFaceAlpha',.75)
colorbar
legend('Vehicle trajectory')
test_x0=10;
test_y0=10;
width=750;
height=600;
set(gcf,'color','w','position',[test_x0,test_y0,width,height]);
hXLabel = xlabel('{\it Y}[m]');
hYLabel = ylabel('{\it X}[m]');
c = colorbar;
c.Label.String = '{\it v_x}[m/s]';
%(1:end-1,1)
grid on
%% scenario plot
plot(scenario)
hold on
set(gcf,'color','w');
hXLabel = xlabel('{\it X}[m]');
hYLabel = ylabel('{\it Y}[m]');
grid on
%% plot of the vehicle path in regards to track lines 
figure
plot(vehicle_y_,vehicle_x_,'LineWidth',1)
hold on
% plot(scenario_waypoints(1:end,2),scenario_waypoints(1:end,1),'LineWidth',1)
% hold on
plot(center_line_y,center_line_x,'LineWidth',1, 'Color','k','LineStyle','--')
hold on
plot(RB(:,2),RB(:,1),'LineWidth',1,'Color','k')
hold on
plot(RB(:,5),RB(:,4),'LineWidth',1,'Color','k')
legend('Vehicle path', 'Centerline','Lane boundary');
test_x0=10;
test_y0=10;
width=750;
height=600;
set(gcf,'color','w','position',[test_x0,test_y0,width,height]);
hXLabel = xlabel('{\it Y}[m]');
hYLabel = ylabel('{\it X}[m]');
grid on
%% plot of the vehicle path in regards to the reference
figure
plot(vehicle_y_,vehicle_x_,'LineWidth',1)
hold on
plot(scenario_waypoints(1:end,2),scenario_waypoints(1:end,1),'LineWidth',1)
hold on
legend('Vehicle path','Reference path');
test_x0=10;
test_y0=10;
width=750;
height=600;
set(gcf,'color','w','position',[test_x0,test_y0,width,height]);
hXLabel = xlabel('{\it Y}[m]');
hYLabel = ylabel('{\it X}[m]');
grid on
%% at which step which lateral model is being used
bar(is_kin_)
hold on
bar(is_dyn_)
hold on
ylim([-0.125 1.125]);
legend('Usage of the kinematic model','Usage of the dynamic model');
hXLabel = xlabel('{\it k}');
hYLabel = ylabel('ON/OFF');
test_x0=10;
test_y0=10;
width=1000;
height=200;
set(gcf,'color','w','position',[test_x0,test_y0,width,height]);
grid on
%% delta_f_ - Front wheel steering angle
figure
plot(delta_f_)
% legend('\delta_f');
hold on
delta_max = 10*ones(size(delta_f_));
plot(delta_max,'--','Color','r')
hold on
delta_min = -10*ones(size(delta_f_));
plot(delta_min,'--','Color','r')
hXLabel = xlabel('{\it k}');
hYLabel = ylabel('{\it \delta_f}[^{o}]');
hLegend = legend( 'Front wheel steering angle','Constraints' );
test_x0=10;
test_y0=10;
width=1000;
height=200;
set(gcf,'color','w','position',[test_x0,test_y0,width,height]);
% xlim([-0.05 10]);
ylim([-15.5 15.5]);
% 
% xticks([0:1:10]);
yticks([-10 -5 0 5 10]);
grid on
%% delta_delta_f_ - Derivative of the front wheel steering angle
figure
plot(delta_delta_f_)
hold on
delta_max = 5*ones(size(delta_f_));
plot(delta_max,'--','Color','r')
hold on
delta_min = -5*ones(size(delta_f_));
plot(delta_min,'--','Color','r')
hold on
hXLabel = xlabel('{\it k}');
hYLabel = ylabel('{\it \Delta\delta_f}[^{o}]');
hLegend = legend( 'Derivative of the front wheel steering angle','Constraints');
test_x0=10;
test_y0=10;
width=1000;
height=200;
set(gcf,'color','w','position',[test_x0,test_y0,width,height]);
% xlim([-0.05 10]);
ylim([-10.5 10.5]);
% 
% xticks([0:1:10]);
% yticks([-0.03:0.01:0.09]);
grid on

%% X_kin_i - Position X of the kinematic model

figure
plot(X_kin_i_)
% legend('vehicle_x_');
hold on
X_max = 2*ones(size(delta_f_));
plot(X_max,'--','Color','r')
hold on
X_min = -2*ones(size(delta_f_));
plot(X_min,'--','Color','r')
hold on
hXLabel = xlabel('{\it k}');
hYLabel = ylabel('{\it X}[m]');
hLegend = legend( 'Position X of the kinematic model','Constraints');
test_x0=10;
test_y0=10;
width=1000;
height=200;
set(gcf,'color','w','position',[test_x0,test_y0,width,height]);
% xlim([-0.05 10]);
ylim([-2.25 2.25]);
% 
% xticks([0:1:10]);
% yticks([-0.03:0.01:0.09]);

grid on

%% Y_kin_i - Position Y of the kinematic model

figure
plot(Y_kin_i_)
% legend('vehicle_x_');
hold on
Y_max = 1*ones(size(delta_f_));
plot(Y_max,'--','Color','r')
hold on
Y_min = -1*ones(size(delta_f_));
plot(Y_min,'--','Color','r')
hold on
hXLabel = xlabel('{\it k}');
hYLabel = ylabel('{\it Y}[m]');
hLegend = legend( 'Position Y of the kinematic model','Constraints');
test_x0=10;
test_y0=10;
width=1000;
height=200;
set(gcf,'color','w','position',[test_x0,test_y0,width,height]);
% xlim([-0.05 10]);
ylim([-1.25 1.25]);
% 
% xticks([0:1:10]);
% yticks([-0.03:0.01:0.09]);

grid on
%% YAW_kin_i - Orientation \psi of the kinematic model
figure
plot(YAW_kin_i_*180/pi)
% legend('vehicle_x_');
hold on
YAW_max = 7*ones(size(delta_f_));
plot(YAW_max,'--','Color','r')
hold on
YAW_min = -7*ones(size(delta_f_));
plot(YAW_min,'--','Color','r')
hold on
hXLabel = xlabel('{\it k}');
hYLabel = ylabel('{\it \psi}[^{o}]');
hLegend = legend( 'Orientation \psi of the kinematic model','Constraints' );
test_x0=10;
test_y0=10;
width=1000;
height=200;
set(gcf,'color','w','position',[test_x0,test_y0,width,height]);
% xlim([-0.05 10]);
ylim([-15.5 15.5]);
% 
% xticks([0:1:10]);
yticks([-10 -7 0 7 10]);
grid on
%% y_i - Position y of the dynamic model

figure
plot(y_i_)
hold on
y_i_max = 0.5*ones(size(delta_f_));
plot(y_i_max,'--','Color','r')
hold on
y_i_min = -0.5*ones(size(delta_f_));
plot(y_i_min,'--','Color','r')
hold on
hXLabel = xlabel('{\it k}');
hYLabel = ylabel('{\it y}[m]');
hLegend = legend( 'Position y of the dynamic model' ,'Constraints');
test_x0=10;
test_y0=10;
width=1000;
height=200;
set(gcf,'color','w','position',[test_x0,test_y0,width,height]);
% xlim([-0.05 10]);
ylim([-1.25 1.25]);
% 
% xticks([0:1:10]);
% yticks([-0.03:0.01:0.09]);
grid on
%% dot_y_i - Derivative of the y position of the dynamic model

figure
plot(dot_y_i_)
hold on
dot_y_i_max = 0.5*ones(size(delta_f_));
plot(dot_y_i_max,'--','Color','r')
hold on
dot_y_i_min = -0.5/2*ones(size(delta_f_));
plot(dot_y_i_min,'--','Color','r')
hold on
hXLabel = xlabel('{\it k}');
hYLabel = ylabel('{\it \Deltay}[m]');
hLegend = legend( 'Derivative of the y position of the dynamic model' ,'Constraints');
test_x0=10;
test_y0=10;
width=1000;
height=200;
set(gcf,'color','w','position',[test_x0,test_y0,width,height]);
% xlim([-0.05 10]);
ylim([-1.25 1.25]);
% 
% xticks([0:1:10]);
% yticks([-0.03:0.01:0.09]);
grid on
%% yaw_i_ - Orientation \psi of the dynamic model
figure
plot(yaw_i_*180/pi)
hold on
yaw_i_max = 3*ones(size(delta_f_));
plot(yaw_i_max,'--','Color','r')
hold on
yaw_i_min = -3*ones(size(delta_f_));
plot(yaw_i_min,'--','Color','r')
hold on
hXLabel = xlabel('{\it k}');
hYLabel = ylabel('{\it \psi}[^{o}]');
hLegend = legend( 'Orientation \psi of the dynamic model' ,'Constraints');
test_x0=10;
test_y0=10;
width=1000;
height=200;
set(gcf,'color','w','position',[test_x0,test_y0,width,height]);
% xlim([-0.05 10]);
ylim([-5.5 5.5]);
% 
% xticks([0:1:10]);
yticks([-5 -3 0 3 5]);
grid on
%% dot_yaw_i_ - Derivative of the \psi orientation of the dynamic model

figure
plot(dot_yaw_i_*180/pi)
hold on
dot_yaw_i_max = 3*ones(size(delta_f_));
plot(dot_yaw_i_max,'--','Color','r')
hold on
dot_yaw_i_min = -3*ones(size(delta_f_));
plot(dot_yaw_i_min,'--','Color','r')
hold on
hXLabel = xlabel('{\it k}');
hYLabel = ylabel('{\it \Delta\psi}[^{o}]');
hLegend = legend( 'Derivative of the \psi orientation of the dynamic model' ,'Constraints');
test_x0=10;
test_y0=10;
width=1000;
height=200;
set(gcf,'color','w','position',[test_x0,test_y0,width,height]);
% xlim([-0.05 10]);
ylim([-5.5 5.5]);
% 
% xticks([0:1:10]);
yticks([-5 -3 0 3 5]);
grid on
%% v_x_ - Vehicle speed

figure
plot(v_x_)
hold on
v_x_max = 18*ones(size(delta_f_));
plot(v_x_max,'--','Color','r')
hold on
v_x_min = 3*ones(size(delta_f_));
plot(v_x_min,'--','Color','r')
hold on
hXLabel = xlabel('{\it k}');
hYLabel = ylabel('{\it v}[m/s]');
hLegend = legend( 'Vehicle speed' ,'Constraints');
test_x0=10;
test_y0=10;
width=1000;
height=200;
set(gcf,'color','w','position',[test_x0,test_y0,width,height]);
% xlim([-0.05 10]);
ylim([-0.5 18.5]);
% 
% xticks([0:1:10]);
yticks([0 3 6 9 12 15 18]);
grid on
%% a_x_ - Vehicle acceleration

figure
plot(a_x_)
hold on
a_x_max = 3*ones(size(delta_f_));
plot(a_x_max,'--','Color','r')
hold on
a_x_min = -5*ones(size(delta_f_));
plot(a_x_min,'--','Color','r')
hold on
hXLabel = xlabel('{\it k}');
hYLabel = ylabel('{\it a}[m/s^2]');
hLegend = legend( 'Vehicle acceleration' ,'Constraints');
test_x0=10;
test_y0=10;
width=1000;
height=200;
set(gcf,'color','w','position',[test_x0,test_y0,width,height]);
% xlim([-0.05 10]);
ylim([-5.5 3.5]);
% 
% xticks([0:1:10]);
yticks([-5 -3 -1.5 0 1.5 3]);
grid on
%% Duration of each step
bar(T_)
hXLabel = xlabel('{\it k}');
hYLabel = ylabel('{\it T}[s]');
hLegend = legend( 'Duration of each step' );
test_x0=10;
test_y0=10;
width=1000;
height=200;
set(gcf,'color','w','position',[test_x0,test_y0,width,height]);
ylim([-0.1 1.1]);
grid on