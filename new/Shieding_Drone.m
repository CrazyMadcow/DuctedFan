classdef Shieding_Drone
    properties (Access = public) 
        nx                 % number of state variable
        nu                 % number of control variable
        m                  % mass
        g                  % gravity coefficient
        dz                 % center of gravity offset to control fin z direction
        d                  % center of gravity offset to control fin x,y direction
        finA               % area of control fin
        Ix                 % moment of inertia x
        Iy                 % moment of inertia y
        Iz                 % momnet of inertia z
        thrustMax          % propeller thrust max
        cT                 % propeller lift coefficient
        cM                 % propeller moment coefficient
        cL                 % control fin lift coefficient
        lprop              % distance between coaixal rotors
        rprop              % propeller radius
        finMax             % maximum fin angle deflection
    end

    methods
        function obj = Shieding_Drone(m, g, dz, d, finA, Ix, Iy, Iz, thrustMax, cT, cM, cL, lprop, rprop, finMax)
            obj.m               = m;
            obj.g               = g;
            obj.dz              = dz;
            obj.d               = d;
            obj.finA            = finA;
            obj.Ix              = Ix;
            obj.Iy              = Iy;
            obj.Iz              = Iz;
            obj.thrustMax       = thrustMax;
            obj.cT              = cT;
            obj.cM              = cM;
            obj.cL              = cL;
            obj.lprop           = lprop;
            obj.rprop           = rprop;
            obj.finMax          = finMax;
        end
        
        function f = StateDynamics(obj,X)
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
            
            uvw                 = [u; v; w];
            pqr                 = [p; q; r];
            velNED              = EulerDCM(roll, pitch, yaw) * uvw;
            eulerDot            = pqrDCM(roll, pitch, yaw) * pqr;
            
            f(1)                = velNED(1);
            f(2)                = velNED(2);
            f(3)                = velNED(3);
            f(4)                = v * r - w * q;
            f(5)                = w * p - u * r;
            f(6)                = u * q - v * p;
            f(7)                = eulerDot(1);
            f(8)                = eulerDot(2);
            f(9)                = eulerDot(3);
            f(10)               = ((obj.Iy - obj.Iz) / obj.Ix) * q * r;
            f(11)               = ((obj.Iz - obj.Ix) / obj.Iy) * r * p;
            f(12)               = ((obj.Ix - obj.Iy) / obj.Iz) * p * q;

        end
        
        function FM= ForceMomentGen(obj, X, U)
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
            
            T1Cmd               =   U(1);
            T2Cmd               =   U(2);
            Del1Cmd             =   U(3);
            Del2Cmd             =   U(4);
            Del3Cmd             =   U(5);
            Del4Cmd             =   U(6);
            
            % Gravity Zone
            Grav                =   [0;0;obj.g];
            R                   =   EulerDCM(roll, pitch, yaw);
            FGrav               =   R' * (obj.m * Grav);
            FxGrav              =   FGrav(1);
            FyGrav              =   FGrav(2);
            FzGrav              =   FGrav(3);
            LGrav               =   0;
            MGrav               =   0;
            NGrav               =   0;
            
            % Thrust Zone 
            FxThrust            =   0;
            FyThrust            =   0;
            FzThrust            =   -T1Cmd - T2Cmd * (1- 0.2 * (T1Cmd/obj.thrustMax));                                              
            LThrust             =   0;
            MThrust             =   0;
            NThrust             =   -T1Cmd * (obj.cM / obj.cT) + T2Cmd * (obj.cM / obj.cT) * (1- 0.2 * (T1Cmd/obj.thrustMax));      
            
            % Aerodynamic Zone 
            Vs                  =  sqrt(T1Cmd/(2 * 1.225 * pi * obj.rprop^2)) * (1 + (obj.lprop/obj.rprop) / sqrt(1+(obj.lprop/obj.rprop)^2));         
            q                   =  0.5 * 1.225 * (2*Vs)^2;                                                                                              
            FxAero              =  obj.cL * obj.finA * q * (Del2Cmd - Del4Cmd);                                                                        
            FyAero              =  obj.cL * obj.finA * q * (Del1Cmd - Del3Cmd);                                                                         
            FzAero              =  0;
            LAero               =  obj.cL * obj.finA * q * (-Del1Cmd + Del3Cmd) * obj.dz;                                                             
            MAero               =  obj.cL * obj.finA * q * (Del2Cmd - Del4Cmd) * obj.dz;                                                               
            NAero               =  obj.cL * obj.finA * q * (Del1Cmd + Del2Cmd + Del3Cmd + Del4Cmd) * obj.d;
            
            % Groundeffect Zone
            FxGround            = 0;
            FyGround            = 0;
            FzGround            = 0;%GroundEffectModel(obj.rprop, D, FzThrust, 1);
            LGround             = 0;
            MGround             = 0;
            NGround             = 0;
            
            
            
            Fx                  =  FxGrav + FxAero + FxThrust + FxGround;
            Fy                  =  FyGrav + FyAero + FyThrust + FyGround;
            Fz                  =  FzGrav + FzAero + FzThrust + FzGround;
            L                   =  LGrav + LAero + LThrust + LGround;
            M                   =  MGrav + MAero + MThrust + MGround;
            N                   =  NGrav + NAero + NThrust + NGround;
            
            FM                  =  [Fx; Fy; Fz; L; M; N];
        end
        
        function g = ControlDynamics(obj, X, U)
            FM               = ForceMomentGen(obj, X, U);
            
            g(4)             = FM(1) / obj.m;
            g(5)             = FM(2) / obj.m;
            g(6)             = FM(3) / obj.m;
            g(10)            = FM(4) / obj.Ix;
            g(11)            = FM(5) / obj.Iy;
            g(12)            = FM(6) / obj.Iz;
            
        end
        
        function XDot = Dynamics(obj,X, U)
            XDot = StateDynamics(obj, X) + ControlDynamics(obj, X, U); % noise term + (3 * pi/180)^2 * randn(1,12);
        end

        function Vs = Get_Vs(obj, U)
            T1Cmd               =   U(1);
            Vs                  =  sqrt(T1Cmd/(2 * 1.225 * pi * obj.rprop^2)) * (1 + (obj.lprop/obj.rprop) / sqrt(1+(obj.lprop/obj.rprop)^2)); 
        end
        
        function allocation = ControlAllocation(obj, FM_des, Vs)
            q                   =  0.5 * 1.225 * (2*Vs)^2;
            cf                  =  obj.cL * obj.finA * q;
            cm1                 =  cf * obj.dz;
            cm2                 =  cf * obj.d;
            cm3                 =  obj.cM / obj.cT;

            CA                  =  [0   0   0   cf  0   -cf;
                                    0   0   cf  0   -cf  0;
                                    -1  -1  0   0   0    0;
                                    0   0   -cm1 0  cm1  0;
                                    0   0   0   cm1 0   -cm1;
                                    0  0 cm2 cm2 cm2 cm2];
            
            allocation          = pinv(CA) * FM_des;
            
            % Constraint Thrust
            allocation(1)       = min(max(allocation(1), 0), obj.thrustMax);
            allocation(2)       = min(max(allocation(2), 0), obj.thrustMax);
            % Constraint fin angle 
            allocation(3)       = finSaturation(obj.finMax, allocation(3));
            allocation(4)       = finSaturation(obj.finMax, allocation(4));
            allocation(5)       = finSaturation(obj.finMax, allocation(5));
            allocation(6)       = finSaturation(obj.finMax, allocation(6));
        end
    end
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

function W = pqrDCM(roll, pitch, yaw)
    W(1,1)  = 1;
    W(1,2)  = sin(roll)*tan(pitch);
    W(1,3)  = cos(roll)*tan(pitch);
    W(2,1)  = 0;
    W(2,2)  = cos(roll);
    W(2,3)  = -sin(roll);
    W(3,1)  = 0;
    W(3,2)  = sin(roll)/cos(pitch);
    W(3,3)  = cos(roll)/cos(pitch);
end

function angle = finSaturation(finMax, alpha)
            if (alpha >= finMax)
                angle = finMax;

            elseif (alpha <= finMax)
                angle = -finMax;
            else
                angle = alpha;
            end
end

function effect = GroundEffectModel(rprop, z, thrust, model_number)
    if model_number == 1
        effect = (1 / ( 1 - 3 * (rprop / (4 * max(-z, 0.3)))^2)) * thrust;
    end
end
