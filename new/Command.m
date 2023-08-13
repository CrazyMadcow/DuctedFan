function xCmd = Command(i, delT) 
    % Command array
    xCmd = zeros(12,1);  % posX, posY, posZ, velX, velY, velZ, roll, pitch, yaw, p, q, r 
    
    % Command 
    if (i <= 2 / delT)   % 2sec input command
        xCmd(1,1)       = 0;
        xCmd(2,1)       = 0;
        xCmd(3,1)       = -3; 
        xCmd(7,1)       = D2R(0);
        xCmd(8,1)       = D2R(0);
        xCmd(9,1)       = D2R(0);
    
    elseif (((2 / delT) < i) && (i <= (4 / delT)))
        xCmd(1,1)       = 2;
        xCmd(2,1)       = 0;
        xCmd(3,1)       = -3; 
        xCmd(7,1)       = D2R(20);
        xCmd(8,1)       = D2R(0);
        xCmd(9,1)       = D2R(0);
    
    
    elseif (((4 / delT) < i) && (i <= (6 / delT)))
        xCmd(1,1)       = 2;
        xCmd(2,1)       = 0;
        xCmd(3,1)       = -3; 
        xCmd(7,1)       = D2R(0);
        xCmd(8,1)       = D2R(30);
        xCmd(9,1)       = D2R(0);
    
    
    elseif (((6 / delT) < i) && (i <= (8 / delT)))
        xCmd(1,1)       = 2;
        xCmd(2,1)       = 3;
        xCmd(3,1)       = -0.3; 
        xCmd(7,1)       = D2R(0);
        xCmd(8,1)       = D2R(0);
        xCmd(9,1)       = D2R(0);
    
    
    elseif (((8 / delT) < i) && (i <= (10 / delT)))
        xCmd(1,1)       = 2;
        xCmd(2,1)       = 3;
        xCmd(3,1)       = -0.3; 
        xCmd(7,1)       = D2R(0);
        xCmd(8,1)       = D2R(0);
        xCmd(9,1)       = D2R(0);
    
    
    elseif (((10 / delT) < i) && (i <= (12 / delT)))
        xCmd(1,1)       = 3;
        xCmd(2,1)       = 3;
        xCmd(3,1)       = -0.5; 
        xCmd(7,1)       = R2D(0);
        xCmd(8,1)       = R2D(0);
        xCmd(9,1)       = R2D(0);
    
    
    elseif (((12 / delT) < i) && (i <= (14 / delT)))
        xCmd(1,1)       = 3;
        xCmd(2,1)       = 3;
        xCmd(3,1)       = -0.5; 
        xCmd(7,1)       = D2R(0);
        xCmd(8,1)       = D2R(0);
        xCmd(9,1)       = D2R(0);
    
    
    elseif (((14 / delT) < i) && (i <= (16 / delT)))
        xCmd(1,1)       = 0;
        xCmd(2,1)       = 0;
        xCmd(3,1)       = -3; 
        xCmd(7,1)       = D2R(0);
        xCmd(8,1)       = D2R(0);
        xCmd(9,1)       = D2R(0);
    
    else
        xCmd(1,1)       = 0;
        xCmd(2,1)       = 0;
        xCmd(3,1)       = 0; 
        xCmd(7,1)       = D2R(0);
        xCmd(8,1)       = D2R(0);
        xCmd(9,1)       = D2R(0);
        
    end
   
end

function radian = D2R(degree)
    radian = degree * (pi / 180);
end

function degree = R2D(radian)
    degree = radian * (180 / pi);
end