function [x_i] = get_state_kinematic(v_x, delta, xi_1_kin, i, scenario)
%GET_STATE SPACE VECTOR
%%
if i==1
    x_i=zeros(3,1);
else

%MODEL
%ucitavanje matrice stanja
bicycle_model_kinematic;

%linearizacija u toèki u kojoj se vozilo nalazi
allPoses = actorPoses(scenario);
% A = subs(A,yaw,allPoses.Yaw);
A = subs(A,yaw,allPoses.Yaw*pi/180);

%kod matrice stanja zamjena varijable brzine sa v_x
A = double(subs(A,V,v_x));
B = double(subs(B,V,v_x));



x_i = A*xi_1_kin + B*delta; % buduce stanje
y_i = C*x_i;  % buduci izlaz
    
end
end

