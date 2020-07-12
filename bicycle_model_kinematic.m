% state variables
Y = sym('Y','real');
X = sym('X','real');
yaw = sym('yaw','real');

% input variable
delta_f = sym('delta_f','real');


% state vector
x = [X; Y; yaw];

% input vector
u = [delta_f];

% vehicle parameters
l_f = 1.1;
l_r = 1.58;

% l_f = sym('l_f','real');
% l_r = sym('l_r','real');

% speed
V = sym('V','real');


% equations of motion
dX = V*cos(yaw);
dY = V*sin(yaw);
% with approximation tan(delta_f)=delta_f
dyaw = (V*delta_f)/(l_f+l_r);



%dX = A*X + U
dx = [dX; dY; dyaw];

A = jacobian(dx,x);
B = jacobian(dx,u);
C = eye(3);
D = zeros(3,1);