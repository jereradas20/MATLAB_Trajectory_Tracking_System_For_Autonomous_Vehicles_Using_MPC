%BICYCLE MODEL

%inicijalizacija

% % Parametri Rajamani
% m = 1573;
% I_z = 2873;
% l_f = 1.1;
% l_r = 1.58;
% C_af = 80000;
% C_ar = 80000;

% Parametri https://www.researchgate.net/publication/305519046_Vehicle_parameter_estimation_using_a_model-based_estimator
% m = 1683;
% l_f = 1.1;
% l_r = 1.58;
% I_z = m*l_f*l_r;
% C_af = 2*117.24;
% C_ar = 2*142.72;

% % Parametri test
m = 1683;
l_f = 1.1;
l_r = 1.58;
I_z = m*l_f*l_r;
C_af = 2*117.24*4;
C_ar = 2*142.72*4;

% %Parametri Simulink
% m = 2000;
% l_f = 1.1;
% l_r = 1.58;
% I_z = 4e3;
% C_af = 3*1.2e3;
% C_ar = 3*1.1e3;

%brzina koja se kasnije mijenja za referentnu
V_x = sym('V_x','real');
%%
%matrice stanja
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
