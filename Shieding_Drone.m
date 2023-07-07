classdef Shieding_Drone
    properties (Access = public) 
        nx                  
        nu                  
        m                   
        g                   
        dz                 
        d                   
        finA                
        Ix                  
        Iy                 
        Iz                  
        rpmMax              
        thrustMax       
        cT                 
    end
    methods
        function obj = Shieding_Drone(m, g, dz, d, finA, Ix, Iy, Iz, rpmMax, thrustMax, cT)
            obj.m  = m;
            obj.g  = g;
            obj.dz = dz;
            obj.d  = d;
            obj.finA = finA;
            obj.Ix   = Ix;
            obj.Iy   = Iy;
            obj.Iz   = Iz;
            obj.rpmMax = rpmMax;
            obj.thrustMax = thrustMax;
            obj.cT   = cT;
        end
        
        function f = StateDynamics(obj,X)
            N           =   X(1);
            E           =   X(2);
            D           =   X(3);
            u           =   X(4);
            v           =   X(5);
            w           =   X(6);
            roll        =   X(7);
            pitch       =   X(8);
            yaw         =   X(9);
            p           =   X(10);
            q           =   X(11);
            r           =   X(12);
            
            uvw         = [u; v; w];
            pqr         = [p; q; r];
            velNED      = EulerDCM(roll, pitch, yaw) * uvw;
            eulerDot    = pqrDCM(roll, pitch, yaw) * pqr;
            
            f(1)      = velNED(1);
            f(2)      = velNED(2);
            f(3)      = velNED(3);
            f(4)      = v*r - w*q;
            f(5)      = w*p - u*r;
            f(6)      = u*q - v*p;
            f(7)      = eulerDot(1);
            f(8)      = eulerDot(2);
            f(9)      = eulerDot(3);
            f(10)      = ((obj.Iy - obj.Iz) / obj.Ix) * q * r;
            f(11)      = ((obj.Iz - obj.Ix) / obj.Iy) * r * p;
            f(12)      = ((obj.Ix - obj.Iy) / obj.Iz) * p * q;

        end
        
        function FM= ForceMomentGen(obj, X, U)
            N                =   X(1);
            E                =   X(2);
            D                =   X(3);
            u                =   X(4);
            v                =   X(5);
            w                =   X(6);
            roll             =   X(7);
            pitch            =   X(8);
            yaw              =   X(9);
            p                =   X(10);
            q                =   X(11);
            r                =   X(12);
            
            T1Cmd            =   U(1);
            T2Cmd            =   U(2);
            Del1Cmd          =   U(3);
            Del2Cmd          =   U(4);
            Del3Cmd          =   U(5);
            Del4Cmd          =   U(6);
            
            % Gravity Zone
            Grav             =   [0;0;obj.g];
            R                =   EulerDCM(roll, pitch, yaw);
            FGrav            =   R' * (obj.m * Grav);
            FxGrav           =   FGrav(1);
            FyGrav           =   FGrav(2);
            FzGrav           =   FGrav(3);
            LGrav            =   0;
            MGrav            =   0;
            NGrav            =   0;
            
            % Thrust Zone 
            FxThrust         =   0;
            FyThrust         =   0;
            FzThrust         =   -T1Cmd - T2Cmd * (1- 0.2 * (T1Cmd/obj.thrustMax));
            LThrust          =   0;
            MThrust          =   0;
            NThrust          =   T1Cmd * obj.cT - T2Cmd * (1-0.2 * (T1Cmd/obj.thrustMax)) * obj.cT;
            
            % Aerodynamic Zone 
            V                =  sqrt(T2Cmd/obj.thrustMax) * 17.0;
            q                =  0.5 * 1.225 * obj.finA * V^2;
            FxAero           =  q * 2.0 * pi * (Del2Cmd - Del4Cmd);
            FyAero           =  q * 2.0 * pi * (Del1Cmd - Del3Cmd);
            FzAero           =  0;
            LAero            =  q * obj.dz * 2.0 * pi * (-Del1Cmd + Del3Cmd);
            MAero            =  q * obj.dz * 2.0 * pi * (Del2Cmd - Del4Cmd);
            NAero            =  q * obj.d * 2.0 * pi * (Del1Cmd + Del2Cmd + Del3Cmd + Del4Cmd);
            
            Fx               =  FxGrav + FxAero + FxThrust;
            Fy               =  FyGrav + FyAero + FyThrust;
            Fz               =  FzGrav + FzAero + FzThrust;
            L                =  LGrav + LAero + LThrust;
            M                =  MGrav + MAero + MThrust;
            N                =  NGrav + NAero + NThrust;
            
            FM               =  [Fx; Fy; Fz; L; M; N];
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
            XDot = StateDynamics(obj, X) + ControlDynamics(obj, X, U);
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
