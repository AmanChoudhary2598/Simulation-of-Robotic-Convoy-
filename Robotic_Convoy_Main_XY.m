%-----------------------------------------------------------------------------
% Program : Control of robotic convoy using PP/DPP/PN guidance
%-----------------------------------------------------------------------------
% Description : To simulate the robotic convoy with PP/DPP/PN guidance
%-----------------------------------------------------------------------------

clear all;
clear global;
close all;
clc;

%-----------------------------------------------------------------------------
% Constants declaration/intialization
%-----------------------------------------------------------------------------
% Simulation parameters
%-----------------------------------------------------------------------------
t_simulation = 15;                             % Total duration of scenario to be simulated (in seconds)
Maneuver = 1;                                   % 1-Circular / 2-Sinusoidal / 3-Spiral / 4-Square
NoiseAdded = 0;                                 % 0-No / 1-Yes
InitialAngle = 0 * pi/4;

t_step = 0.01;                                  % Simulation time step (in seconds)
t_0 = 0;                                        % Initial time for siumlation
markerColours = ['r' 'y' 'm' 'c' 'b' 'g'];      % To support plotting data up to 6 robots

PlotMargin = 10;
R_PlotRange = 5;
Theta_PlotRange = 2*pi;
Theta_dot_PlotRange = pi;
%-----------------------------------------------------------------------------
% Engagement parameters
%-----------------------------------------------------------------------------
global V;
global LeadMaxDeviationAngle;
global LeadMeanAngle;
global TotalNoOfConvoys;
global NoOfFollowers;
global Delta;
global K1;
global Rc;                                      % Turn radius of the lead robot
global LeadRobotTurnRate;                       % Turn rate of the lead robot
global ManeuverType;
global ThetaNoiseLimit;
global IsNoiseAdded;

x_0 = [5 4 3 2 1 0]'*00;                        % Initial X cordinates of each robot
y_0 = [5 4 3 2 1 0]'*2;                         % Initial Y cordinates of each robot
d = sqrt((x_0(1)-x_0(2))^2+(y_0(1)-y_0(2))^2);  
V = [5 5 5 5 5 5]'*1;                           % Velocities of each robot
Delta = -0.0 * pi/36;                            % Delta in steps of 5 degrees
K1 = 1.05;                                      % K1 = 1 for PP and DPP guidance
ThetaNoiseLimit = 10;

IsNoiseAdded = NoiseAdded;
ManeuverType = Maneuver;

TotalNoOfConvoys = size(V,1);
NoOfFollowers = TotalNoOfConvoys - 1;
LeadMaxDeviationAngle = 1 * pi/18;
LeadMeanAngle = InitialAngle;

Rc_d_ratio = 6;
Rc = Rc_d_ratio * d;
LeadRobotTurnRate = V(1)/Rc;



if ManeuverType==1
    textPosition = -10;
elseif ManeuverType==2
    textPosition = -20;
elseif ManeuverType==3
    textPosition = -40;
else
    textPosition = 30;
end

if K1==1
    if Delta==0
        TextGuidanceName = 'PP Guidance';
    else
        TextGuidanceName = 'DPP Guidance';
    end
else
    TextGuidanceName = 'PN Guidance';
end

R_0 = zeros(size(V));
Theta_0 = zeros(size(V));
Alpha_0 = zeros(size(V));
% V_R = zeros(size(V));
% V_Theta = zeros(size(V));
R_dot_0 = zeros(size(V));
Theta_dot_0 = zeros(size(V));

R_t = zeros(size(V));
Theta_t = zeros(size(V));
Alpha_t = zeros(size(V));
R_dot_t = zeros(size(V));
Theta_dot_t = zeros(size(V));

%-----------------------------------------------------------------------------
% Other calculated parameters and Variables initialization
%-----------------------------------------------------------------------------

for iCounter = 1:1:TotalNoOfConvoys
    
    if iCounter==1
        R_0(iCounter) = 0;
        Theta_0(iCounter) = 0;
        Alpha_0(iCounter) = LeadMeanAngle;
        R_dot_0(iCounter) = 0;
        Theta_dot_0(iCounter) = 0;
        
        R_t(iCounter) = R_0(iCounter);
        Theta_t(iCounter)= Theta_0(iCounter);
        Alpha_t(iCounter) = Alpha_0(iCounter);
        R_dot_t(iCounter) = R_dot_0(iCounter);
        Theta_dot_t(iCounter) = Theta_dot_0(iCounter);
        
    else
        
        R_0(iCounter) = sqrt((x_0(iCounter-1)-x_0(iCounter))^2+(y_0(iCounter-1)-y_0(iCounter))^2);        
        Theta_0(iCounter) = taninverse((x_0(iCounter-1)-x_0(iCounter)),(y_0(iCounter-1)-y_0(iCounter)));  
        Alpha_0(iCounter) = Theta_0(iCounter) + Delta;
        R_dot_0(iCounter) = V(iCounter-1)*cos(Alpha_0(iCounter-1)-Theta_0(iCounter)) - V(iCounter)*cos(Delta);        
        Theta_dot_0(iCounter) = (V(iCounter-1)*sin(Alpha_0(iCounter-1)-Theta_0(iCounter)) - V(iCounter)*sin(Delta))/R_0(iCounter);        
        
        R_t(iCounter) = R_0(iCounter);
        Theta_t(iCounter)= Theta_0(iCounter);
        Alpha_t(iCounter) = Alpha_0(iCounter);
        R_dot_t(iCounter) = R_dot_0(iCounter);
        Theta_dot_t(iCounter) = Theta_dot_0(iCounter);
        
    end
end

%-----------------------------------------------------------------------------
% Kinematic equations for pure pusuit
%-----------------------------------------------------------------------------
% V_R(t)     = V_T * cos(Alpha_T - Theta(t)) - V_P * cos(Delta)
% V_Theta(t) = V_T * sin(Alpha_T - Theta(t)) - V_P * sin(Delta)
%-----------------------------------------------------------------------------

%-----------------------------------------------------------------------------
% Initial values vector creation
%-----------------------------------------------------------------------------
InitialValues = zeros(3*TotalNoOfConvoys,1);

for iCounter = 1:1:TotalNoOfConvoys
    InitialValues(0*TotalNoOfConvoys+iCounter) = x_0(iCounter);
    InitialValues(1*TotalNoOfConvoys+iCounter) = y_0(iCounter);
    InitialValues(2*TotalNoOfConvoys+iCounter) = Alpha_0(iCounter);
end

InitialValues = InitialValues';

%-----------------------------------------------------------------------------
% Main program
%-----------------------------------------------------------------------------
options = odeset('Events', @ODETerminateEvent);
tspan = t_0:t_step:t_simulation;
[t,y_out] = ode45(@Robotic_Convoy_Kinematics_XY,tspan, InitialValues, options);

stepCount = size(y_out,1);

dy = zeros(size(y_out,1),size(y_out,2));

Xpos = zeros(stepCount,TotalNoOfConvoys);
Ypos = zeros(stepCount,TotalNoOfConvoys);
Theta = zeros(stepCount,TotalNoOfConvoys);
R = zeros(stepCount,TotalNoOfConvoys);
Theta_dot = zeros(stepCount,TotalNoOfConvoys);
Alpha = zeros(stepCount,TotalNoOfConvoys);

for iCounter = 1:1:TotalNoOfConvoys
    Xpos(:,iCounter) = y_out(:,0*TotalNoOfConvoys+iCounter);
    Ypos(:,iCounter) = y_out(:,1*TotalNoOfConvoys+iCounter);
    Alpha(:,iCounter) = y_out(:,2*TotalNoOfConvoys+iCounter);
end

for iCounter = 1:1:stepCount
    
    Initial_Y_Val = y_out(iCounter,:);
    dy(iCounter,:) = Robotic_Convoy_Kinematics_XY (t(iCounter), Initial_Y_Val);
    R(iCounter,1) = 0;
    Theta(iCounter,1) = 0;
    for jCounter = 2:1:TotalNoOfConvoys
        R(iCounter,jCounter) = sqrt((Xpos(iCounter,jCounter-1)-Xpos(iCounter,jCounter))^2+(Ypos(iCounter,jCounter-1)-Ypos(iCounter,jCounter))^2);
        Theta(iCounter,jCounter) = taninverse((Xpos(iCounter,jCounter-1)-Xpos(iCounter,jCounter)),(Ypos(iCounter,jCounter-1)-Ypos(iCounter,jCounter)));
        Theta_dot(iCounter,jCounter) = ((V(jCounter-1)*sin(Alpha(iCounter,jCounter-1)-Theta(iCounter,jCounter)))-(V(jCounter)*sin(Alpha(iCounter,jCounter)-Theta(iCounter,jCounter))))/R(iCounter,jCounter);
    end
end


%-----------------------------------------------------------------------------
% Plots
%-----------------------------------------------------------------------------
figure(1);
hold on;
set(gca,'Color','k')
set(gca, 'XColor', [0.1 0.1 0.1]);
set(gca, 'YColor', [0.1 0.1 0.1]);
[X_min, X_minIndex] = min(Xpos(:,1));
[Y_min, Y_minIndex] = min(Ypos(:,1));
[X_max, X_maxIndex] = max(Xpos(:,1));
[Y_max, Y_maxIndex] = max(Ypos(:,1));
MinValue = min(X_min,Y_min);
MaxValue = max(X_max,Y_max);
X_Range = X_max - X_min;
Y_Range = Y_max - Y_min;
axis([MinValue-PlotMargin MaxValue+PlotMargin MinValue-PlotMargin MaxValue+PlotMargin]);
for iCounter=1:1:TotalNoOfConvoys
    plot(Xpos(:,iCounter),Ypos(:,iCounter),'Color',markerColours(iCounter));
end
for iCounter=1:1:TotalNoOfConvoys
    plot(Xpos(1,iCounter),Ypos(1,iCounter),'r*');
    plot(Xpos(stepCount,iCounter),Ypos(stepCount,iCounter),'mo');
end
title('Robot''s position');
xlabel('X position (m)');
ylabel('Y position (m)');
legend('{\color{gray}Lead robot''s trajectory}','{\color{gray}2^{nd} robot''s trajectory}', '{\color{gray}3^{rd} robot''s trajectory}', '{\color{gray}4^{th} robot''s trajectory}', '{\color{gray}5^{th} robot''s trajectory}','{\color{gray}Last robot''s trajectory}'); % To be updated manually if the number of robots are changed
TextHandle = text(double(textPosition), double(textPosition), sprintf('%15s \n V_{Lead} = %3.0f m/s, \n Distance = %3.2f m, \n N = %3.3f, \n Delta = %3.1f deg',TextGuidanceName,V(1),d,K1,Delta*180/pi), 'FontSize',10);
set(TextHandle, 'Color',[1, 1 ,1])
grid on;
pbaspect([1 1 1])
%axis normal;

figure(2);
plot1 = subplot(1,2,1);
hold on;
set(gca,'Color','k');
set(gca, 'XColor', [0.1 0.1 0.1]);
set(gca, 'YColor', [0.1 0.1 0.1]);
for iCounter=2:1:TotalNoOfConvoys
    plot(plot1,t,R(:,iCounter),'Color',markerColours(iCounter));
end
title('Distance between robots');
ylabel('Distance R_{i/i+1}(m)');
xlabel('Time (s)');
% LegendObj = legend('show');
% LegendObj.Color = 'red';
legend('{\color{gray}R_{12}}','{\color{gray}R_{23}}', '{\color{gray}R_{34}}', '{\color{gray}R_{45}}', '{\color{gray}R_{56}}'); % To be updated manually if the number of robots are changed
grid on;
axis([0 t_simulation 0 R_PlotRange])
axis normal;

plot2 = subplot(1,2,2);
hold on;
set(gca,'Color','k');
set(gca, 'XColor', [0.1 0.1 0.1]);
set(gca, 'YColor', [0.1 0.1 0.1]);
for iCounter=2:1:TotalNoOfConvoys
    plot(plot2,t,Theta(:,iCounter),'Color',markerColours(iCounter));
end
title('LOS angle between robots');
ylabel('LOS angle \theta_{i/i+1}(rad)');
xlabel('Time (s)');
% LegendObj = legend('show');
% LegendObj.Color = 'red';
legend('{\color{gray}\theta_{12}}','{\color{gray}\theta_{23}}', '{\color{gray}\theta_{34}}', '{\color{gray}\theta_{45}}', '{\color{gray}\theta_{56}}'); % To be updated manually if the number of robots are changed
grid on;
axis([0 t_simulation -1*Theta_PlotRange Theta_PlotRange])
axis normal;

figure(3);
hold on;
set(gca,'Color','k');
set(gca, 'XColor', [0.1 0.1 0.1]);
set(gca, 'YColor', [0.1 0.1 0.1]);
for iCounter=2:1:TotalNoOfConvoys
    plot(t,Theta_dot(:,iCounter),'Color',markerColours(iCounter));
end
title('d\theta(t)/dt V/s Time');
ylabel('d\theta(t)/dt (rad/s)');
xlabel('Time (s)');
% LegendObj = legend('show');
% LegendObj.Color = 'red';
legend('{\color{gray}d\theta_{12}(t)/dt}','{\color{gray}d\theta_{23}(t)/dt}', '{\color{gray}d\theta_{34}(t)/dt}', '{\color{gray}d\theta_{45}(t)/dt}', '{\color{gray}d\theta_{56}(t)/dt}'); % To be updated manually if the number of robots are changed
grid on;
axis([0 t_simulation -1*Theta_dot_PlotRange Theta_dot_PlotRange])
axis normal;
