clc;   
close all; 
clear;
workspace; 
format long g;
format compact;
fontSize = 20;

simTime = 40; % simulated seconds after burnout 
%% Initial Conditions (are at burnout)
angle = 86.95; % [Deg]
x0 = 57.93; % [ft]
y0 = 1732.1; % [ft]
v0 = 671.5; % [ft/s]
targetAlt=4000;
% Time
timeStep = 0.05;
time = timeStep:timeStep:simTime+timeStep; % time vector 

% Find perfect ideal curve:
apogeeIdeal=0;
CD=1;
while targetAlt-apogeeIdeal >10 || targetAlt-apogeeIdeal < -10 
    % Ideal Curve
    [~,yIdeal,apogeeIdeal,~,~,dragIdeal]=getTrajectory(time, timeStep, angle, x0, y0, v0, 0, 0, CD, 0);
    if targetAlt-apogeeIdeal > 0
        CD=CD-.01;
    elseif targetAlt-apogeeIdeal < 0
        CD=CD+.01;
    end
end
[~,I] = max(yIdeal);
timeToIdealApogee= (I*timeStep);
% No Airbrake
[xNoAB,yNoAB,apogeeNoAB,~,VelNoAB,drag]=getTrajectory(time, timeStep, angle, x0, y0, v0, 0, 1,0,0);

% With Airbrake
[xAB,yAB,apogeeAB,delta,VelAB,dragAB]=getTrajectory(time, timeStep, angle, x0, y0, v0, 1, dragIdeal,0,timeToIdealApogee);

% Display apogees
disp("Apogee:")
disp("No AB: "); disp(apogeeNoAB);
disp("AB: "); disp(apogeeAB);
disp("Ideal: "); disp(apogeeIdeal);

xPlot=3.5:timeStep:simTime+3.5+timeStep;
hold on
figure(1);
plot([0 50], [5000 5000],'LineWidth', 1) % 5000 ft target line
plot([0 3.5],[0 y0],'LineWidth', 1) % burn line
plot(3.5,y0,"*",'MarkerSize', 10) % burn *

plot(xPlot, yNoAB, 'r-', 'LineWidth', 1); % With Drag line
plot(xPlot, yAB, 'b-', 'LineWidth', 1); % With Drag & AB line
plot(xPlot, yIdeal, 'g-', 'LineWidth', 1); % With Drag & AB line
grid on;
title("Altitude vs Time")
legend("Target","Burn","Burn End","No AB","With Drag & AB", "Ideal",'Location','southeast')

xlim([0 simTime])
ylim([0 7000])
xlabel("Time (s)")
ylabel("Altitude (ft)")

figure(2);
% Drag Plot
subplot(1,2,1) 
hold on;
plot(xPlot, drag, 'r-', 'LineWidth', 1); 
plot(xPlot, dragAB, 'b-', 'LineWidth', 1); 
plot(xPlot, dragIdeal, 'g-', 'LineWidth', 1); 
grid on;

legend("No AB","AB","Ideal",'Location','northwest')
title("Drag Produced")
xlim([2 20])
ylim([-5 50])
xlabel("Time (s)")
ylabel("Drag (lbf)")

% Delta Plot
subplot(1,2,2) 
hold on;
plot(xPlot, delta, 'r-', 'LineWidth', 1); 
grid on;

title("Airbrake Deflection Angles")
xlim([2 20])
ylim([0 90])
xlabel("Time (s)")
ylabel("Deflection Angle (deg)")

%% Functions
function [x,y,apogee,delta,Velocity, Drag]=getTrajectory(time, step, angle, x0, y0, v0, withAB, dragIdeal,CD,timeToIdealApogee)
    g = -32.2; % [ft/s^2]
    %% Rocket Data
    diameter = 4/12; % [ft]
    width_AB = 2/12; % [ft]
    length_AB = 3/12; % [ft]
    num_AB = 4;
    weight = 15.83; % [lb] at burnout
     
    % Preallocate Matracies
    ax=zeros(1,length(time));
    ay=zeros(1,length(time));
    delta=zeros(1,length(time)+1);
    Drag = zeros(1,length(time)+1);
    y = zeros(1,length(time)); y(1)=y0;
    x = zeros(1,length(time)); x(1)=x0;
    Velocity = zeros(1,length(time)); Velocity(1) = v0;
    
    % Initial x & y velocities
    v0x=v0*cosd(angle);
    v0y=v0*sind(angle);
    vx = zeros(1,length(time)); vx(1)=v0x;
    vy = zeros(1,length(time)); vy(1)=v0y;
    for t = 1:length(time)
        [Drag(t),Cd(t),S(t),rho(t)] = getDrag(y(t), Velocity(t),delta(t), dragIdeal, CD);
        if isnan(Drag(t)) % Fix for when Drag is NaN when y=0
            Drag(t) = 0;
        end
        % Acceleration in x & y at time t
        ay(t) = -(((0-Drag(t))*sind(angle))-weight)/(weight/g);
        ax(t) = -((0-Drag(t))*cosd(angle))/(weight/g);
        vx(t+1) = vx(t) + ax(t)*step; 
        vy(t+1) = vy(t) + ay(t)*step;
        if vy(t) < 1
            delta(t+1) = 0;
            
        end
        Velocity(t+1) = sqrt(vx(t).^2 + vy(t).^2);
        x(t+1) = x(t) + vx(t+1)*step;
        y(t+1) = y(t) + vy(t+1)*step;
        
        
        % For airbrakes check ideal velocity
        if withAB && y(t+1)-y(t) >0 
            if timeToIdealApogee-time(t) > .5
                delta_Dreq = dragIdeal(t) - Drag(t);
                delta_Cd = (2*delta_Dreq)/(rho(t)*vy(t)^2*S(t));
                Cd_new = Cd(t)+delta_Cd;
                delta_S = (2*delta_Dreq)/(rho(t)*vy(t)^2*Cd_new);
                if delta_S < 0
                    mult=-1;
                else
                    mult=1;
                end
                deltaAngle = mult*asind(abs(delta_S)/(num_AB*width_AB*length_AB));
                delta(t+1) = delta(t) + deltaAngle;
            end
            if delta(t+1) > 80
                delta(t+1) = 80;
            end
        end
   
    end
    apogee=max(y);
end
%%
function [Drag, CD_total, Sref, rho] = getDrag(altitude, velocity, delta,dragIdeal, CD)
    
    %% Interpolation Calculations at SL and 5000 ft
    rho_low = 0.002377; rho_high = 0.002048; % slug/ft 
    alt_low = 0; alt_high = 5000; % ft
    
    %% Flight Data
    rho = rho_low+(altitude-alt_low)*(rho_high-rho_low)/(alt_high-alt_low); % slug/ft
    
    %% Rocket Data
    diameter = 4/12; % ft
    width_AB = 2/12; % ft
    length_AB = 3/12; % ft
    num_AB = 4;

    %% Wetted Areas
    Swet_tube = pi*(0.5*diameter)^2;
    Swet_AB = width_AB*length_AB*sind(delta)*num_AB; % ft^2
    Sref = Swet_tube + Swet_AB; % ft^2

    % Drag Coefficients
    Table_CD = zeros(1, 81);
    Table_CD(1) = 0.38;
    Table_CD(6) = 0.395;
    Table_CD(11) = 0.421;
    Table_CD(16) = 0.465;
    Table_CD(21) = 0.495;
    Table_CD(26) = 0.528;
    Table_CD(31) = 0.56;
    Table_CD(36) = 0.59;
    Table_CD(41) = 0.644;
    Table_CD(46) = 0.649;
    Table_CD(51) = 0.689;
    Table_CD(56) = 0.69;
    Table_CD(61) = 0.685;
    Table_CD(66) = 0.698;
    Table_CD(71) = 0.69;
    Table_CD(76) = 0.71;
    Table_CD(81) = 0.729;
    d0=0;
    d1=10;
    if delta ~= 0
        for i=1:7
            if delta >= d0 && delta <= d1
                break
            end
            d0 = d0 + 10;
            d1 = d1 + 10;
        end
        CD_total = Table_CD(d0+1) + (delta-d0)*((Table_CD(d1+1)-Table_CD(d0+1))/(d1-d0));
    else
        CD_total = Table_CD(1);%(Cd_BT+Cd_fin) % with correction
    end
    if dragIdeal == 0
        CD_total=CD;
    end
    Drag = 0.5*rho*(velocity^2)*CD_total*Sref; % lbf
end





