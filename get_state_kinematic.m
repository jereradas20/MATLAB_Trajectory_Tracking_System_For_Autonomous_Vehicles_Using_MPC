function [x_i] = get_state_kinematic(v_x, delta, xi_1_kin, i, scenario)
%% GET NEW STATE 

if i==1
    x_i=zeros(3,1);
else

% kinematic model
bicycle_model_kinematic;

% yaw linerization
allPoses = actorPoses(scenario);
% A = subs(A,yaw,allPoses.Yaw);
A = subs(A,yaw,allPoses.Yaw*pi/180);

% speed linerization
A = double(subs(A,V,v_x));
B = double(subs(B,V,v_x));



x_i = A*xi_1_kin + B*delta; % new state
y_i = C*x_i;  % new output
    
end
end

