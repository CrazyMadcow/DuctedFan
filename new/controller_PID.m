
function [U, BaCmd, InnerCmd] = controller_PID(Xcmd, X, I_term, dt, m, g, Ix,Iy,Iz)
    U = zeros(6,1);
    BaCmd = zeros(2,1);
    InnerCmd = zeros(4,1);
    %%%%%% Command %%%%%%
    N_cmd       = Xcmd(1);
    E_cmd       = Xcmd(2);
    D_cmd       = Xcmd(3);
    
    roll_cmd     = Xcmd(7);
    pitch_cmd    = Xcmd(8);
    yaw_cmd      = Xcmd(9);
    
    %%%%%% State %%%%%%
    N                   =   X(1);
    E                   =   X(2);
    D                   =   X(3);
    u                   =   X(4);
    v                   =   X(5);
    w                   =   X(6);
    roll                =   X(7);
    pitch               =   X(8);
    yaw                 =   X(9);
    p                   =   X(10);
    q                   =   X(11);
    r                   =   X(12);
    uvw                 =  [u;v;w];
    velNED              =  EulerDCM(roll, pitch, yaw) * uvw;
    
    Ndot                = velNED(1);
    Edot                = velNED(2);
    Ddot                = velNED(3);
    
    %%%%%%% PID Gain %%%%%%
    tau_N               = 4;
    tau_Ndot            = 1;
    tau_E               = 4;
    tau_Edot            = 1;
    tau_D               = 0.6;
    tau_Ddot            = 0.15;
    tau_roll            = 0.5;
    tau_pitch           = 0.5;
    tau_yaw             = 0.6;
    tau_p               = 0.15;
    tau_q               = 0.15;
    tau_r               = 0.3;

    K_N                 = 1 / tau_N;
    K_Ndot              = 1 / tau_Ndot;
    K_E                 = 1 / tau_E;
    K_Edot              = 1 / tau_Edot;
    K_D                 = 1 / tau_D;
    K_Ddot              = 1 / tau_Ddot;
    K_roll              = 1 / tau_roll;
    K_pitch             = 1 / tau_pitch;
    K_yaw               = 1 / tau_yaw;
    K_p                 = 1 / tau_p;
    K_q                 = 1 / tau_q;
    K_r                 = 1 / tau_r;
    
    I_Ndot              = 1 / tau_Ndot * 1;
    I_Edot              = 1 / tau_Edot * 1;
    I_Ddot              = 1 / tau_Ddot * 1;
    I_p                 = 1 / tau_p * 0.3;
    I_q                 = 1 / tau_q * 0.3;
    I_r                 = 1 / tau_r * 0.3;

    %%%%%% Position(NE) Controller(PD) %%%%%%
%     IvCmd(1,1)           = K_N * (N_cmd - N);
%     IvCmd(2,1)           = K_E * (E_cmd - E);
%     
%     BvCmd(1,1)           = IvCmd(1,1) * cos(yaw) + IvCmd(2,1) * sin(yaw);
%     BvCmd(2,1)           = -IvCmd(1,1) * sin(yaw) + IvCmd(2,1) * cos(yaw);
%     
%     BaCmd(1,1)           = K_Ndot * (BvCmd(1,1) - u);
%     BaCmd(2,1)           = K_Edot * (BvCmd(2,1) - v);
%     
%     pitch_cmd            = -BaCmd(1,1);
%     roll_cmd             = BaCmd(2,1);
%     
%     if (pitch_cmd > 60 * pi/180) 
%         pitch_cmd = 60 * pi/180;
%     elseif (pitch_cmd < -60 * pi/180)
%         pitch_cmd = -60 * pi/180;
%     end
%     
%     if (roll_cmd > 60 * pi/180) 
%         roll_cmd = 60 * pi/180;
%     elseif (roll_cmd < -60 * pi/180)
%         roll_cmd = -60 * pi/180;
%     end
    
    %%%%%% Attitude & Altitude Controller(P-PI) %%%%%%
    % Trim
    Ddot_trim           = 0;
    p_trim              = -q * sin(roll) * tan(pitch) - r * cos(roll) * tan(pitch);
    q_trim              = r * tan(pitch);
    r_trim              = -q * tan(roll);
    Fz_trim             =  - m * g / (cos(pitch) * cos(roll));
    L_trim              =  (Iz - Iy) * q * r;
    M_trim              =  (Ix - Iz) * p * r;
    N_trim              =  (Iy - Ix) * q * p;
    
    InnerCmd(1,1)       = K_D * (D_cmd - D) + Ddot_trim;
    InnerCmd(2,1)       = K_roll * (roll_cmd - roll) + p_trim;
    InnerCmd(3,1)       = K_pitch * (pitch_cmd - pitch) + q_trim;
    yaw_cmd
    InnerCmd(4,1)       = K_yaw * (yaw_cmd - yaw) + r_trim;

    I_term(3,1)         = I_term(3,1) + I_Ddot * (InnerCmd(1,1) - Ddot) * dt;
    I_term(4,1)         = I_term(4,1) + I_p * (InnerCmd(2,1) - p) * dt;
    I_term(5,1)         = I_term(5,1) + I_q * (InnerCmd(3,1) - q) * dt;
    I_term(6,1)         = I_term(6,1) + I_r * (InnerCmd(4,1) - r) * dt;

    Fz_control          = (m / (cos(pitch) * cos(roll))) * (K_Ddot * (InnerCmd(1,1) - Ddot) + I_term(3,1)) + Fz_trim;
    Mx_control          = Ix * (K_p * (InnerCmd(2,1) - p) + I_term(4,1)) + L_trim;
    My_control          = Iy * (K_q * (InnerCmd(3,1) - q) + I_term(5,1)) + M_trim;
    Mz_control          = Iz * (K_r * (InnerCmd(4,1) - r) + I_term(6,1)) + N_trim;

    U                   = [0; 0; Fz_control; Mx_control; My_control; Mz_control];
end 

function R = EulerDCM(roll, pitch, yaw)
    R(1,1)  = cos(yaw) * cos(pitch);
    R(1,2)  = -sin(yaw)*cos(roll) + cos(yaw)*sin(pitch)*sin(roll); 
    R(1,3)  = sin(yaw)*sin(roll) + cos(yaw)*sin(pitch)*cos(roll);
    R(2,1)  = sin(yaw)*cos(pitch);
    R(2,2)  = cos(yaw)*cos(roll) + sin(yaw)*sin(pitch)*sin(roll);
    R(2,3)  = -cos(yaw)*sin(roll) + sin(yaw)*sin(pitch)*cos(roll);
    R(3,1)  = -sin(pitch);
    R(3,2)  = cos(pitch)*sin(roll);
    R(3,3)  = cos(pitch)*cos(roll);
end

