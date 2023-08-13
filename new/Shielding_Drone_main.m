% Micro Coaxial Shielding Fan Drone
clear; clc; close all;

%% Parameter Settings 
nx                  = 12;
nu                  = 6;
m                   = 0.798;
g                   = 9.81;
dz                  = 0.3;
d                   = 0.1;
finA                = 0.005;
Ix                  = 1.08 * 10^(-2);
Iy                  = 1.08 * 10^(-2);
Iz                  = 9.05 * 10^(-3);
thrustMax           = 1.233*9.81;
cT                  = 3.467 * 10^(-7);
cM                  = 5.713 * 10^(-9);
cL                  = 2 * pi;
lprop               = 0.15;
rprop               = 0.3302;
finMax              = 40 * pi/180;

I_term              = zeros(6,1);
%% Initial Value Settings 
N_i                  = 0;
E_i                  = 0;   
D_i                  = -3;
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
xCmd0                = zeros(nx,1);

% Setting State/Control Variables 
X                    = X0;
XDot                 = XDot0;
U                    = U0;
xCmd                 = xCmd0;


%% Simulation Settings 
t0                  = 0.0;
tf                  = 20;
dt                  = 1/400;
t                   = 0:dt:tf;
nT                  = length(t);

% Histroy Buffer 
XHist(:,1)          = X;
XDotHist(:,1)       = XDot;
UHist(:,1)          = U;
XCmdHist(:,1)       = xCmd;


%% Shiedling Fan Drone Class (Main Simulation)
Model = Shieding_Drone(m, g, dz, d, finA, Ix, Iy, Iz, thrustMax, cT, cM, cL, lprop, rprop, finMax);
U1 = zeros(6,1);
U2 = zeros(6,1);
for i = 2:nT
%     TCmd = -0.5 * (10.5 * (-0.0 - X(6))-9.81);
%     DelCmd = 0.5 * (0 - X(12));
%     U = [TCmd; TCmd; DelCmd; DelCmd; DelCmd; DelCmd];
    
    % Ref Command
%     xCmd            = Command(i, dt);
    xCmd(3,1) = -3;
    xCmd(7,1) = D2R(20);
    xCmd(8,1) = D2R(0);
    xCmd(9,1) = D2R(0);
    % Controller 
    [U, BaCmd, InnerCmd] = controller_PID(xCmd, X, I_term, dt, m, g, Ix, Iy, Iz);
    
    % Acuator Mixer
    Vs   = Model.Get_Vs(U1);
    
    % Allocation
    U1   = Model.ControlAllocation(U,Vs);
    
    % first order actuator dynamics 
%     U2(1:2,1) = U2(1:2,1) + (1/0.09)*(U1(1:2,1)-U2(1:2,1))*dt;
%     U2(3:6,1) = U2(3:6,1) + (1/0.2)*(U1(3:6,1)-U2(3:6,1))*dt;
    
    % Model Dynamics 
    Xdot = Model.Dynamics(X, U1);
    
    % Model Propagation
    X    = X + Xdot' * dt;
    
    % Buffer save 
    XCmdHist(:,i) = [xCmd(1); xCmd(2); xCmd(3); 0;0;0; xCmd(7); xCmd(8); xCmd(9); InnerCmd(2,1); InnerCmd(3,1); InnerCmd(4,1)];
    XHist(:,i) = X;
    XDotHist(:,i) = Xdot;
    UHist(:,i)  = U2;
end

figure(1) 
title('Posistion') 
subplot(3,1,1);
plot(t,XCmdHist(1,:))
hold on 
plot(t,XHist(1,:));
subtitle("N")
legend("Cmd", "State")

subplot(3,1,2);
plot(t,XCmdHist(2,:))
hold on 
plot(t,XHist(2,:));
subtitle("E")
legend("Cmd", "State")

subplot(3,1,3);
plot(t,XCmdHist(3,:))
hold on 
plot(t,XHist(3,:));
subtitle("D")
legend("Cmd", "State")

% figure(2) 
% title('Velocity')
% plot(t,XHist(4,:));
% hold on 
% plot(t,XHist(5,:));
% plot(t,XHist(6,:));
% legend("u","v","w");

figure(3)
title('Attitude')
subplot(3,1,1);
plot(t,R2D(XCmdHist(7,:)))
hold on 
plot(t,R2D(XHist(7,:)));
subtitle("Roll")
legend("Cmd", "State")

subplot(3,1,2);
plot(t,R2D(XCmdHist(8,:)))
hold on 
plot(t,R2D(XHist(8,:)));
subtitle("Pitch")
legend("Cmd", "State")

subplot(3,1,3);
plot(t,R2D(XCmdHist(9,:)))
hold on 
plot(t,R2D(XHist(9,:)));
subtitle("Yaw")
legend("Cmd", "State")

figure(4)
title('Rate')
subplot(3,1,1);
plot(t,R2D(XCmdHist(10,:)))
hold on 
plot(t,R2D(XHist(10,:)));
subtitle("p")
legend("Cmd", "State")

subplot(3,1,2);
plot(t,R2D(XCmdHist(11,:)))
hold on 
plot(t,R2D(XHist(11,:)));
subtitle("q")
legend("Cmd", "State")

subplot(3,1,3);
plot(t,R2D(XCmdHist(12,:)))
hold on 
plot(t,XHist(12,:));
subtitle("r")
legend("Cmd", "State")

figure(5) 
title('Thrust') 
plot(t,UHist(1,:))
hold on;
plot(t,UHist(2,:))
legend("T1","T2");

figure(6)
title("Fin Angle")
plot(t,R2D(UHist(3,:)))
hold on;
plot(t,R2D(UHist(4,:)))
plot(t,R2D(UHist(5,:)))
plot(t,R2D(UHist(6,:)))
legend("Fin1","Fin2","Fin3","Fin4")

%% Functions 
function radian = D2R(degree)
    radian = degree * (pi / 180);
end

function degree = R2D(radian)
    degree = radian * (180 / pi);
end

