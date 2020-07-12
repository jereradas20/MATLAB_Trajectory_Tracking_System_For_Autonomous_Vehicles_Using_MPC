function [x_i] = get_state_dynamic(v_x, delta, xi_1, i)
%% GET NEW STATE 

if i==1
    x_i=zeros(4,1);
else

% dynamic model
bicycle_model;

% speed linerization
A = double(subs(A,V_x,v_x));




x_i = A*xi_1 + B*delta; % new state
y_i = C*x_i;  % new output
    
end
end

