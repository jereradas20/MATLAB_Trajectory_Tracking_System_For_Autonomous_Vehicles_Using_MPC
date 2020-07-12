function [x_i] = get_state_dynamic(v_x, delta, xi_1, i)
%GET_STATE Summary of this function goes here

if i==1
    x_i=zeros(4,1);
else

bicycle_model;

%kod matrice stanja zamjena varijable brzine sa v_x
A = double(subs(A,V_x,v_x));




x_i = A*xi_1 + B*delta; % buduce stanje
y_i = C*x_i;  % buduci izlaz
    
end
end

