function dydt = Robotic_Convoy_Kinematics_XY(t, y)
    %-------------------------------------------------------------------------------
    % Kinematic diffectial equations of N robots forming a convoy
    % Three differential equations per robot (X_dot/Y_dot/Alpha_dot)
    %-------------------------------------------------------------------------------
    
    dydt = zeros(size(y));
    
    %-------------------------------------------------------------------------------
    % Required global and local variables declaration;
    %-------------------------------------------------------------------------------
    global LeadMaxDeviationAngle;
    %global LeadMeanAngle;
    global TotalNoOfConvoys;
    global Delta;
    global V;
    global K1;
    global LeadRobotTurnRate;
    global IsNoiseAdded;
    global ThetaNoiseLimit;
    global ManeuverType;
    global LeadMeanAngle;
    
    XPos = zeros(TotalNoOfConvoys,1);
    YPos = zeros(TotalNoOfConvoys,1);
    Alpha = zeros(TotalNoOfConvoys,1);
    Theta = zeros(TotalNoOfConvoys,1);
    R = zeros(TotalNoOfConvoys,1);
    
    
    %-------------------------------------------------------------------------------
    % Variables initialization;
    %-------------------------------------------------------------------------------
    XPos(1) = y(0*TotalNoOfConvoys + 1);
    YPos(1) = y(1*TotalNoOfConvoys + 1);
    
    Theta(1) = 0;
    R(1) = 0;
    MaxError = ThetaNoiseLimit;
    MinError = -ThetaNoiseLimit;
    
    if ManeuverType==4
        Alpha(1) = LeadMeanAngle + mod(floor(t/10),4)*pi/2;
    else
        Alpha(1) = y(2*TotalNoOfConvoys + 1);
    end
    
    for iCounter = 2:1:TotalNoOfConvoys
        XPos(iCounter) = y(0*TotalNoOfConvoys + iCounter);
        YPos(iCounter) = y(1*TotalNoOfConvoys + iCounter);
        Alpha(iCounter) = y(2*TotalNoOfConvoys + iCounter);
        Theta(iCounter) = taninverse((XPos(iCounter-1)-XPos(iCounter)),(YPos(iCounter-1)-YPos(iCounter)));
        Theta(iCounter) = Theta(iCounter) + IsNoiseAdded * (((MaxError-MinError).*rand(1,1) + MinError)/180*pi) ;
        R(iCounter) = sqrt((XPos(iCounter-1)-XPos(iCounter))^2+(YPos(iCounter-1)-YPos(iCounter))^2);
    end
    
    
    %-------------------------------------------------------------------------------
    % Slope computation / Differential Equations;
    %-------------------------------------------------------------------------------
    dydt(0*TotalNoOfConvoys + 1) = V(1) * cos(Alpha(1));    % X_dot for Lead Robot
    dydt(1*TotalNoOfConvoys + 1) = V(1) * sin(Alpha(1));    % Y_dot for Lead Robot
    
    % Alpha_dot for Lead Robot
    if(mod(floor(t/400),2))
        C=1;
    else
        C=1;
    end
    
    if ManeuverType==1
        dydt(2*TotalNoOfConvoys + 1) = LeadRobotTurnRate; % For circular motion of the lead robot of the convoy
    elseif ManeuverType==2
        dydt(2*TotalNoOfConvoys + 1) = 1.25 * LeadMaxDeviationAngle * sin(0.15 * t); % For sinusoidal motion of the lead robot of the convoy
    elseif ManeuverType==3
        dydt(2*TotalNoOfConvoys + 1) = -4.0 / (0.5*t + 20); % For spiral motion of the lead robot of the convoy
    else
        dydt(2*TotalNoOfConvoys + 1) = 0;
    end
    
    for iCounter = 2:1:TotalNoOfConvoys        
        if K1==1
            V(iCounter) = V(iCounter-1) * cos(Alpha(iCounter-1)- Theta(iCounter)) / cos(Alpha(iCounter)- Theta(iCounter)); %cos(Delta);
        else
            V(iCounter) = V(iCounter-1) * cos(Alpha(iCounter-1)- Theta(iCounter)) / cos(Alpha(iCounter)- Theta(iCounter));
            %V(iCounter) = V(iCounter-1) * cos(Alpha(iCounter-1)-Theta(iCounter)) / cos((K1-1)*Theta(iCounter));
        end
        
        dydt(0*TotalNoOfConvoys + iCounter) = V(iCounter) * cos(Alpha(iCounter));
        dydt(1*TotalNoOfConvoys + iCounter) = V(iCounter) * sin(Alpha(iCounter));
        Theta_dot = ((V(iCounter-1)* sin(Alpha(iCounter-1)-Theta(iCounter))) - (V(iCounter)* sin(Alpha(iCounter)-Theta(iCounter)))) / R(iCounter);
        
        if K1==1
            dydt(2*TotalNoOfConvoys + iCounter) = K1 *  Theta_dot;
        else
            %dydt(2*TotalNoOfConvoys + iCounter) = K1 *  V(iCounter-1) * sin(Alpha(iCounter)-(K1-1)*Theta(iCounter)) / cos((K1-1)*Theta(iCounter));
            dydt(2*TotalNoOfConvoys + iCounter) = K1 *  Theta_dot;
        end
    end
    
end

