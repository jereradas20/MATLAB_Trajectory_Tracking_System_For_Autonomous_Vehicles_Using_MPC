%% PARAMETER INITIALIZATION FOR THE MAIN SCRIPT


clear all;

% loop parameters
STOP=0;
T=0;
runtime=0;
i=1;
N=300; % terminal step




% longitudinal controller parameters
v_x=0; % new speed
a_x=0; % new acceleration
v_x_t=0; % current speed
a_x_t=0; % current acceleration



% lateral controller parameters
delta_f=0; %front wheel steering angle


% initial vehicle parameters

YAW_0= 173.5;
X_0= 72.1479; 
Y_0= 104.2;
V_0=0;

xi_1 = zeros(4,1);% initial lateral dynamic state
xi_1_kin = zeros(3,1);% initial lateral kinematic state
X=X_0;
Y=Y_0;
V_x=0;
V_y=0;
YAW=YAW_0;


% memory allocation
is_dyn_ = zeros(N,1);
is_kin_ = zeros(N,1);

yaw_ref_= zeros(N,1);
dot_yaw_ref_ = zeros(N,1);
y_ref_ = zeros(N,1);
dot_y_ref_ = zeros(N,1);

Curvature_ = zeros(N,1);
    
dot_YAW_des_= zeros(N,1);
YAW_des_= zeros(N,1);
X_des_= zeros(N,1);
Y_des_= zeros(N,1);

v_x_= zeros(N,1);
a_x_= zeros(N,1);
v_ref_cal_= zeros(N,1);

delta_f_= zeros(N,1);
problem_= zeros(N,1);

X_kin_i_ = zeros(N,1);
Y_kin_i_ = zeros(N,1);
YAW_kin_i_ = zeros(N,1);

y_i_ = zeros(N,1);
dot_y_i_ = zeros(N,1);
yaw_i_ = zeros(N,1);
dot_yaw_i_ = zeros(N,1);

vehicle_yaw_= zeros(N,1);
   
vehicle_y_= zeros(N,1);
vehicle_x_= zeros(N,1);

vehicle_v_y_= zeros(N,1);
vehicle_v_x_= zeros(N,1);

T_= zeros(N,1);


