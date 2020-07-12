function [delta_f,problem]=lateral_controller_test(v_x, y_ref, dot_y_ref, yaw_ref, dot_yaw_ref,i,scenario);
%% LATERAL DYNAMIC CONTROLLER
% MPC controller for calculating the front wheel steering angle

problem=0;
if ((isequal(zeros(10,1),y_ref)) || i==1)
    delta_f=0;
else 
    
allPoses = actorPoses(scenario);

X_c = (allPoses.Position(1));
Y_c = (allPoses.Position(2));
YAW_c_deg = (allPoses.Yaw);
    

% dynamic model
bicycle_model;
    
% speed linearization
A = double(subs(A,V_x,v_x));

% controller parameters
N = 10; % prediction horizon
N_u = 2; % control horizon

% constraints: xmin <= x <= xmax, umin <= u <= umax, dumin <= du <= dumax
xmin = -[2.85/2; 2.85/2; 7*pi/180; 7*pi/180];
xmax = [2.85/2; 2.85/2; 7*pi/180; 7*pi/180];

umin = -15*pi/180;
umax = 15*pi/180;

dumin = -7*pi/180;
dumax = 7*pi/180;

% vector dimensions
nx = size(A,1);
nu = size(B,2);
ny = size(C,1);

% reference matrix 
ref = [y_ref, dot_y_ref, yaw_ref*pi/180, dot_yaw_ref*pi/180]; 

% setting sdp variable
U = sdpvar(repmat(nu,1,N),ones(1,N));

% initialization
constraints = [];
objective = 0;
x0 = 0*ones(nx,1);
xk_1 = x0;
Pk = 0.5*diag([10 1 10 1]); % for e in k==N
Qk = diag([10 1 10 1]); % for e in k<N
Rk = 0.1; % for delta u
Sk = 0.1; % for u

% soft constraints for the output vector
M = [1; 1; 3*pi/180; 3*pi/180];
epsilon_x = sdpvar(repmat(1,1,N),ones(1,N));
ro = 1;

% soft constraints for the derivative of the input vector
epsilon_u = sdpvar(repmat(1,1,N),ones(1,N));
ro_u = 0.0003;
M_u = 3*pi/180;


% problem construction
for k = 1:N
    xk=A*xk_1 + B*U{k}; 
    yk = C*xk;  % predicted output
    
    refk = ref(k,:)'; % reference reorganization
   
    
    % constraints
    if k == 1
        constraints=[constraints,...
            (xk + M*epsilon_x{k})>= xmin, (xk - M*epsilon_x{k}) <= xmax,... % state constraints
            U{k} >= umin, U{k} <= umax]; % input constraints
        
    elseif k > 1 && k <= N_u
        constraints=[constraints,...
            (xk + M*epsilon_x{k}) >= xmin, (xk - M*epsilon_x{k}) <= xmax,... % state constraints
            U{k} >= umin, U{k} <= umax,... % input constraints
            ((U{k} - U{k-1})+M_u*epsilon_u{k}) >= dumin, ((U{k} - U{k-1})- M_u*epsilon_u{k}) <= dumax]; % constraints on the derivative of the input
        
    else % k > N_u
        constraints=[constraints,...
            (xk + M*epsilon_x{k}) >= xmin, (xk - M*epsilon_x{k}) <= xmax]; % state constraints
        
    end
       
    
    % criterion function
    if k==1
       objective = objective + ro*ro*epsilon_x{k} + ro_u*ro_u*epsilon_u{k} + 0.5*(yk - refk)'*Qk*(yk - refk) + 0.5*U{k}'*Sk*U{k}; % QP
        
    elseif k > 1 && k <= N_u
        objective = objective + ro*ro*epsilon_x{k} + ro_u*ro_u*epsilon_u{k} + 0.5*(yk - refk)'*Qk*(yk - refk) + 0.5*(U{k}-U{k-1})'*Rk*(U{k}-U{k-1}) + 0.5*U{k}'*Sk*U{k}; 
        
    elseif k > N_u && k < N
        objective = objective + ro*ro*epsilon_x{k} + ro_u*ro_u*epsilon_u{k} + 0.5*(yk - refk)'*Qk*(yk - refk);
        
    else % k == N
        objective = objective + ro*ro*epsilon_x{k} + ro_u*ro_u*epsilon_u{k} + 0.5*(yk - refk)'*Pk*(yk - refk);
        
    end
    
    xk_1=xk;
end

% solution
ops = sdpsettings('solver','cplex','verbose',0,'warning',0);
diagnostics = optimize(constraints,objective,ops)
u0=value(U{1}); % take only first element of the vector

% check if there is a problem with solving the problem
if (diagnostics.problem~=0)
    problem=problem+1;
end

% ONLY FOR TESTING
x = zeros(nx*(N+1),1);
y = zeros(ny*(N+1),1);
x(1:nx) = x0;
y(1:ny) = C*x0;
u = u0;

for k = 1:N
    x(k*nx+1:k*nx+nx)=A*x((k-1)*nx+1:(k-1)*nx+nx) + B*U{k}; 
    y(k*ny+1:k*ny+ny) = C*x(k*nx+1:k*nx+nx); 
    u = double([u; U{k}]);
end

tracking_error = abs(y(ny+1:end)-ref(:));
tracking_error = reshape(tracking_error,N,nx);
delta_f=u0;

end
end

