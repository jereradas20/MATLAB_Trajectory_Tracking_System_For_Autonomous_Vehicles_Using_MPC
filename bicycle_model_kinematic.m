%varijable stanja
Y = sym('Y','real');
X = sym('X','real');
yaw = sym('yaw','real');

%varijable ulaza
delta_f = sym('delta_f','real');


%vektor stanja
x = [X; Y; yaw];

%vektor ulaza
u = [delta_f];

%parametri vozila
l_f = 1.1;
l_r = 1.58;

% l_f = sym('l_f','real');
% l_r = sym('l_r','real');

%brzina koja se kasnije mijenja za referentnu
V = sym('V','real');


%jednadzbe gibanja
dX = V*cos(yaw);
dY = V*sin(yaw);
% uz aproksimaciju tan(delta_f)=delta_f
dyaw = (V*delta_f)/(l_f+l_r);



%dX = A*X + U
dx = [dX; dY; dyaw];

A = jacobian(dx,x);
B = jacobian(dx,u);
C = eye(3);
D = zeros(3,1);