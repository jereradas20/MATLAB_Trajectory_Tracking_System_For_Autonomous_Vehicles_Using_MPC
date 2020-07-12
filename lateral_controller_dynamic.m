function [delta_f,problem]=lateral_controller_test(v_x, y_ref, dot_y_ref, yaw_ref, dot_yaw_ref,i,scenario);
%LATERAL_CONTROLLER
%   MPC kontroler za izracun zakreta kura prednjih kotaca
%%

problem=0;
if ((isequal(zeros(10,1),y_ref)) || i==1)
    delta_f=0;
else 
    
allPoses = actorPoses(scenario);

X_c = (allPoses.Position(1));
Y_c = (allPoses.Position(2));
YAW_c_deg = (allPoses.Yaw);
    

%MODEL
%ucitavanje matrice stanja
bicycle_model;
    
%kod matrice stanja zamjena varijable brzine sa v_x
A = double(subs(A,V_x,v_x));

%PARAMETRI REGULATORA
N = 10; % predikcijski horizont
N_u = 2; % upravljački horizont

% ogranicenja: xmin <= x <= xmax, umin <= u <= umax, 
xmin = -[2.85/2; 2.85/2; 7*pi/180; 7*pi/180];
xmax = [2.85/2; 2.85/2; 7*pi/180; 7*pi/180];


umin = -15*pi/180;
umax = 15*pi/180;
dumin = -7*pi/180;
dumax = 7*pi/180;

% Dimenzije sustava (velicine vektora)
nx = size(A,1);
nu = size(B,2);
ny = size(C,1);

%Referenca 
ref = [y_ref, dot_y_ref, yaw_ref*pi/180, dot_yaw_ref*pi/180]; 


% Postavljanje sdp varijable U --> ona se izračunava
U = sdpvar(repmat(nu,1,N),ones(1,N));

% Inicijalizacija
constraints = [];
objective = 0;
x0 = 0*ones(nx,1);
xk_1 = x0;
Pk = 0.5*diag([10 1 10 1]);
Qk = diag([10 1 10 1]);
Rk = 0.1;
Sk = 0.1;

%meka ograničenja za izlazni vektor
M = [1; 1; 3*pi/180; 3*pi/180];
epsilon_x = sdpvar(repmat(1,1,N),ones(1,N));
ro = 1;

%meka ograničenja za promjenu upravljačke veličine
epsilon_u = sdpvar(repmat(1,1,N),ones(1,N));
ro_u = 0.0003;
M_u = 3*pi/180;


% konstrukcija problema
for k = 1:N
    xk=A*xk_1 + B*U{k}; % buduce stanje
    yk = C*xk;  % buduci izlaz
    
    refk = ref(k,:)'; % referenca
   
    
    %ograničenja
    if k == 1
        constraints=[constraints,...
            (xk + M*epsilon_x{k})>= xmin, (xk - M*epsilon_x{k}) <= xmax,... % ogranicenja stanja
            U{k} >= umin, U{k} <= umax]; % ogranicenja upravljacke velicine
        
    elseif k > 1 && k <= N_u
        constraints=[constraints,...
            (xk + M*epsilon_x{k}) >= xmin, (xk - M*epsilon_x{k}) <= xmax,... % ogranicenja stanja
            U{k} >= umin, U{k} <= umax,... % ogranicenja upravljacke velicine
            ((U{k} - U{k-1})+M_u*epsilon_u{k}) >= dumin, ((U{k} - U{k-1})- M_u*epsilon_u{k}) <= dumax]; % ogranicenja promjene upravljacke velicine
        
    else % k > N_u
        constraints=[constraints,...
            (xk + M*epsilon_x{k}) >= xmin, (xk - M*epsilon_x{k}) <= xmax]; % ogranicenja stanja
        
    end
       
    
    %kriterijska funkcija
    if k==1
       objective = objective + ro*ro*epsilon_x{k} + ro_u*ro_u*epsilon_u{k} + 0.5*(yk - refk)'*Qk*(yk - refk) + 0.5*U{k}'*Sk*U{k};
        
    elseif k > 1 && k <= N_u
        objective = objective + ro*ro*epsilon_x{k} + ro_u*ro_u*epsilon_u{k} + 0.5*(yk - refk)'*Qk*(yk - refk) + 0.5*(U{k}-U{k-1})'*Rk*(U{k}-U{k-1}) + 0.5*U{k}'*Sk*U{k}; % QP problem slijeđenja
        
    elseif k > N_u && k < N
        objective = objective + ro*ro*epsilon_x{k} + ro_u*ro_u*epsilon_u{k} + 0.5*(yk - refk)'*Qk*(yk - refk);
        
    else % k == N
        objective = objective + ro*ro*epsilon_x{k} + ro_u*ro_u*epsilon_u{k} + 0.5*(yk - refk)'*Pk*(yk - refk);
        
    end
    
    xk_1=xk;
end

% proracun rjesenja
ops = sdpsettings('solver','cplex','verbose',0,'warning',0);
diagnostics = optimize(constraints,objective,ops)
u0=value(U{1}); % <-- upravljacka velicina

if (diagnostics.problem~=0)
    problem=problem+1;
end

% izračun rješenja na horizontu
x = zeros(nx*(N+1),1); % +1 zbog toga sto ubacujemo i x0 u vektor
y = zeros(ny*(N+1),1);
x(1:nx) = x0;
y(1:ny) = C*x0;
u = u0;

for k = 1:N
    x(k*nx+1:k*nx+nx)=A*x((k-1)*nx+1:(k-1)*nx+nx) + B*U{k}; % buduce stanje
    y(k*ny+1:k*ny+ny) = C*x(k*nx+1:k*nx+nx);  % buduci izlaz
    u = double([u; U{k}]);
end

tracking_error = abs(y(ny+1:end)-ref(:));
tracking_error = reshape(tracking_error,N,nx);
delta_f=u0;

end
end

