% Micro Coaxial Shielding Fan Drone
clear; clc; close all;

%% Parameter Settings 
nx                  = 12;
nu                  = 6;
m                   = 1;
g                   = 9.81;
dz                  = 0.3;
d                   = 0.1;
finA                = 0.005;
Ix                  = 0.01;
Iy                  = 0.01;
Iz                  = 0.01;
rpmMax              = 8786;
thrustMax           = 1.233*9.81;
cT                  = 0.032;

%% Initial Value Settings 
N_i                  = 0;
E_i                  = 0;   
D_i                  = 0;
u_i                  = 0;
v_i                  = 0;
w_i                  = 0;
roll_i               = 0;
pitch_i              = 0;
yaw_i                = 0;
p_i                  = 0;
q_i                  = 0;
r_i                  = 0;

X0                   = [N_i; E_i; D_i; u_i; v_i; w_i; roll_i; pitch_i; yaw_i; p_i; q_i; r_i];
XDot0                = zeros(nx,1);
U0                   = zeros(nu,1);

% Setting State/Control Variables 
X                    = X0;
XDot                 = XDot0;
U                    = U0;
%% Simulation Settings 
t0                  = 0.0;
tf                  = 20;
dt                  = 0.001;
t                   = 0:dt:tf;
nT                  = length(t);

% History Buffer 
XHist               = zeros(nx, nT);
XDotHist            = zeros(nx, nT);
UHist               = zeros(nu, nT);

% Initialize setting 
XHist(:,1)          = X0;
XDotHist(:,1)       = XDot0;
UHist(:,1)          = U0;
%% Shiedling Fan Drone Class (Main Simulation)
Model = Shieding_Drone(m, g, dz, d, finA, Ix, Iy, Iz, rpmMax, thrustMax, cT);

for i = 2:nT
    TCmd = -0.5 * (10.5 * (-0.0 - X(6))-9.81);
    DelCmd = 0.5 * (0 - X(12));
    U = [TCmd; TCmd; DelCmd; DelCmd; DelCmd; DelCmd];
  
    Xdot = Model.Dynamics(X, U);
    X    = X + Xdot' * dt;
    
    XHist(:,i) = X;
    XDotHist(:,i) = Xdot;
    UHist(:,i)  = U;
end

figure(1) 
plot(t, XHist(6,:), '-b')

figure(2) 
plot(t, XHist(12,:), '-b')

figure(3)
plot(t, XHist(9,:), '-b')

