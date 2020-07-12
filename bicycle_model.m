%% LATERAL DYNAMIC MODEL

% Parameters
m = 1683;
l_f = 1.1;
l_r = 1.58;
I_z = m*l_f*l_r;
C_af = 2*117.24*4;
C_ar = 2*142.72*4;



% speed
V_x = sym('V_x','real');
%%
% y - Lateral position
% yaw - Yaw angle 
% x = [y; dot_y; yaw; dot_yaw;]
% dot_x = A*x + B*delta_f

A = [0 1 0 0;
    0 -((2*C_af+2*C_ar)/(m*V_x)) 0 -(V_x+(2*C_af*l_f-2*C_ar*l_r)/(m*V_x));
    0 0 0 1;
    0 -((2*C_af*l_f-2*C_ar*l_r)/(I_z*V_x)) 0 -((2*C_af*l_f*l_f+2*C_ar*l_r*l_r)/(I_z*V_x))];

B = [0; 2*C_af/m; 0; 2*C_af*l_f/I_z];

C = eye(4);

D = [0; 0; 0; 0];
