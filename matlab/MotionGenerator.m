
classdef MotionGenerator  < handle % matlab.System
    
    properties (SetAccess = private)
        maxVel = 0;
        maxAcc = 0;
        pos = 0;
        vel = 0;
        acc = 0;
        oldPos = 0;
        oldPosRef = 0;
        oldVel = 0;
        
        dBrk = 0;
        dAcc = 0;
        dVel = 0;
        dDec = 0;
        dTot = 0;
        
        tBrk = 0;
        tAcc = 0;
        tVel = 0;
        tDec = 0;
        
        velSt = 0;
        
        oldTime = 0;
        lastTime = 0;
        deltaTime = 0;
        
        signM = 1;      % 1 = positive change, -1 = negative change
        shape = 1;      % 1 = trapezoidal, 0 = triangular
        
    end
    
    methods
        % =================================================================
        % Constructor
        function MG = MotionGenerator(aVelMax,aAccMax,aPosInit)
            MG.maxVel       = aVelMax;
            MG.maxAcc       = aAccMax;
            MG.oldPos       = aPosInit;
            MG.oldPosRef    = aPosInit;
            MG.pos          = aPosInit;
        end
        
        % =================================================================
        % Public methods
        function update(MG, posRef, time)
            
            if (MG.oldPosRef ~= posRef)  % reference changed
                % Shift state variables
                MG.oldPosRef = posRef;
                MG.oldPos = MG.pos;
                MG.oldVel = MG.vel;
                MG.oldTime = MG.lastTime;
                
                % Calculate braking time and distance (in case is neeeded)
                MG.tBrk = abs(MG.oldVel) / MG.maxAcc; % same as for shape = 0
                MG.dBrk = MG.tBrk * abs(MG.oldVel) / 2;
                
                % Caculate Sign of motion
                MG.signM = sign(posRef - (MG.oldPos + sign(MG.oldVel)*MG.dBrk));
                
                if (MG.signM ~= sign(MG.oldVel))  % means brake is needed
                    MG.tAcc = (MG.maxVel / MG.maxAcc);
                    MG.dAcc = MG.tAcc * (MG.maxVel / 2);
                else
                    MG.tBrk = 0;
                    MG.dBrk = 0;
                    MG.tAcc = (MG.maxVel - abs(MG.oldVel)) / MG.maxAcc;
                    MG.dAcc = MG.tAcc * (MG.maxVel + abs(MG.oldVel)) / 2;
                end
                
                % Calculate total distance to go after braking
                MG.dTot = abs(posRef - MG.oldPos + MG.signM*MG.dBrk);
                
                MG.tDec = MG.maxVel / MG.maxAcc;
                MG.dDec = MG.tDec * (MG.maxVel) / 2;
                MG.dVel = MG.dTot - (MG.dAcc + MG.dDec);
                MG.tVel = MG.dVel / MG.maxVel;
                
                if (MG.tVel > 0)    % trapezoidal shape
                    MG.shape = 1;
                else                % triangular shape
                    MG.shape = 0;
                    % Recalculate distances and periods
                    if (MG.signM ~= sign(MG.oldVel))  % means brake is needed
                        MG.velSt = sqrt(MG.maxAcc*(MG.dTot));
                        MG.tAcc = (MG.velSt / MG.maxAcc);
                        MG.dAcc = MG.tAcc * (MG.velSt / 2);
                    else
                        MG.tBrk = 0;
                        MG.dBrk = 0;
                        MG.dTot = abs(posRef - MG.oldPos);      % recalculate total distance
                        MG.velSt = sqrt(0.5*MG.oldVel^2 + MG.maxAcc*MG.dTot);
                        MG.tAcc = (MG.velSt - abs(MG.oldVel)) / MG.maxAcc;
                        MG.dAcc = MG.tAcc * (MG.velSt + abs(MG.oldVel)) / 2;
                    end
                    MG.tDec = MG.velSt / MG.maxAcc;
                    MG.dDec = MG.tDec * (MG.velSt) / 2;
                end
                
            end
            
            % Calculate time since last set-point change
            MG.deltaTime = (time - MG.oldTime);
            % Calculate new setpoint
            MG.calculateTrapezoidalProfile(posRef);
            % Update last time
            MG.lastTime = time;
            
        end
        % =================================================================
        function calculateTrapezoidalProfile(MG, posRef)
            
            t = MG.deltaTime;
            
            if (MG.shape)   % trapezoidal shape
                if (t <= (MG.tBrk+MG.tAcc))
                    MG.pos = MG.oldPos + MG.oldVel*t + MG.signM * 0.5*MG.maxAcc*t^2;
                    MG.vel = MG.oldVel + MG.signM * MG.maxAcc*t;
                    MG.acc = MG.signM * MG.maxAcc;
                elseif (t > (MG.tBrk+MG.tAcc) && t < (MG.tBrk+MG.tAcc+MG.tVel))
                    MG.pos = MG.oldPos + MG.signM * (-MG.dBrk + MG.dAcc + MG.maxVel*(t-MG.tBrk-MG.tAcc));
                    MG.vel = MG.signM * MG.maxVel;
                    MG.acc = 0;
                elseif (t >= (MG.tBrk+MG.tAcc+MG.tVel) && t < (MG.tBrk+MG.tAcc+MG.tVel+MG.tDec))
                    MG.pos = MG.oldPos + MG.signM * (-MG.dBrk + MG.dAcc + MG.dVel + MG.maxVel*(t-MG.tBrk-MG.tAcc-MG.tVel) - 0.5*MG.maxAcc*(t-MG.tBrk-MG.tAcc-MG.tVel)^2);
                    MG.vel = MG.signM * (MG.maxVel - MG.maxAcc*(t-MG.tBrk-MG.tAcc-MG.tVel));
                    MG.acc = - MG.signM * MG.maxAcc;
                else
                    MG.pos = posRef;
                    MG.vel = 0;
                    MG.acc = 0;
                end
            else            % triangular shape
                if (t <= (MG.tBrk+MG.tAcc))
                    MG.pos = MG.oldPos + MG.oldVel*t + MG.signM * 0.5*MG.maxAcc*t^2;
                    MG.vel = MG.oldVel + MG.signM * MG.maxAcc*t;
                    MG.acc = MG.signM * MG.maxAcc;
                elseif (t > (MG.tBrk+MG.tAcc) && t < (MG.tBrk+MG.tAcc+MG.tDec))
                    MG.pos = MG.oldPos + MG.signM * (-MG.dBrk + MG.dAcc + MG.velSt*(t-MG.tBrk-MG.tAcc) - 0.5*MG.maxAcc*(t-MG.tBrk-MG.tAcc)^2);
                    MG.vel = MG.signM * (MG.velSt - MG.maxAcc*(t-MG.tBrk-MG.tAcc));
                    MG.acc = - MG.signM * MG.maxAcc;
                else
                    MG.pos = posRef;
                    MG.vel = 0;
                    MG.acc = 0;
                end
                
            end
            
        end
        
    end
end