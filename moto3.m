clear
close all
clc

options = optimoptions('fsolve','Display','none');

% Script for a gripper with the base plate fixed and a top mobile plate
% Kinematics of the gripper

% DoF: yA

% I modeled the finger as a rectangular base as a trapezoid 

% The object gripper can be useful to mantain all functions, giving
% grippers of different dimensions, the one here is the starting one

% From last experiments:
gripper3.r_e = 50; % mm - OF - radius of the base plate
gripper3.r_i = 40; % mm - AB - radius of the moving plate
gripper3.b = 30; % mm - BC - length of the connecting rod
gripper3.c = 41.9124;  % mm - FD - length of the external link, between finger and base plate
% Once imposed x_E(y_A=0), gripper.c can be found in the next
% section
gripper3.a = 5; % mm - OP - distance of moving plate, with y_A = 0, respect to fixed one

gripper3.d = 10.50; % mm - GH == DI - left and right length of rectangular base of the finger 
gripper3.h = 120; % mm - height of the finger triangle
gripper3.n = 29.43; % mm - GD == HI - bases of the rectangular base of finger
gripper3.m = 7.32; % mm - CG - height of triangle in finger base
gripper3.f = sqrt(gripper3.h^2+(gripper3.n/2)^2); % mm - HE == IE - Side of finger
gripper3.k = sqrt(gripper3.n^2+gripper3.m^2); % mm - CD - hypotenuse
gripper3.alpha = atan (gripper3.n/gripper3.m); % rad - GCD - angle of the triangle of the base of the finger
gripper3.beta = atan(gripper3.h/(gripper3.n/2)); % rad - IHE == HIE - base angle of the finger
gripper3.l = 40; % mm - finger width

gripper3.motion =@(y_A, x) [ % x = [theta; gamma; x_E; y_E]
    gripper3.r_e-gripper3.r_i-gripper3.b*cos(x(1))-gripper3.k*cos(x(2));
    gripper3.c-gripper3.a-y_A-gripper3.b*sin(x(1))-gripper3.k*sin(x(2));
    x(3)-gripper3.r_i-gripper3.b*cos(x(1))-(gripper3.m+gripper3.d)*cos(gripper3.alpha+x(2))-gripper3.f*cos(gripper3.alpha+gripper3.beta+x(2)-pi/2);
    x(4)-gripper3.a-y_A-gripper3.b*sin(x(1))-(gripper3.m+gripper3.d)*sin(gripper3.alpha+x(2))-gripper3.f*sin(gripper3.alpha+gripper3.beta+x(2)-pi/2)
]; % Equation of Motion of the mechanism. DoF y_A, the unknowns are position of thoretical
% peak point of the finger, angle of the connecting rod respect to
% horizontal axis and angle of the hypotenuse of the triangle respect to
% horizontal

gripper3.x0 = [30*pi/180; 20*pi/180; 62; 148]; % randomly chosen initial vector
% for the solver iterations

gripper3.x0 = fsolve (@(x)gripper3.motion(0, x), gripper3.x0, options);

gripper3.motion(0, gripper3.x0) % This should print [0;0;0;0] or almost
% on the console, for the dimensions of the gripper being compatible with
% the real mechanism

% Original Model
gripperOG.r_e = 96; % mm - OF - radius of the base plate
gripperOG.r_i = 61; % mm - AB - radius of the moving plate
gripperOG.b = 15; % mm - BC - length of the connecting rod
gripperOG.c = 31.0917;  % mm - FD - length of the external link, between finger and base plate
% Once imposed x_E(y_A=0), gripper.c can be found in the next
% section
gripperOG.a = 5; % mm - OP - distance of moving plate, with y_A = 0, respect to fixed one

gripperOG.d = 10.50; % mm - GH == DI - left and right length of rectangular base of the finger 
gripperOG.h = 120; % mm - height of the finger triangle
gripperOG.n = 29.43; % mm - GD == HI - bases of the rectangular base of finger
gripperOG.m = 7.32; % mm - CG - height of triangle in finger base
gripperOG.f = sqrt(gripperOG.h^2+(gripperOG.n/2)^2); % mm - HE == IE - Side of finger
gripperOG.k = sqrt(gripper3.n^2+gripperOG.m^2); % mm - CD - hypotenuse
gripperOG.alpha = atan (gripperOG.n/gripperOG.m); % rad - GCD - angle of the triangle of the base of the finger
gripperOG.beta = atan(gripperOG.h/(gripperOG.n/2)); % rad - IHE == HIE - base angle of the finger
gripperOG.l = 40; % mm - finger width

gripperOG.motion =@(y_A, x) [ % x = [theta; gamma; x_E; y_E]
    gripperOG.r_e-gripperOG.r_i-gripperOG.b*cos(x(1))-gripperOG.k*cos(x(2));
    gripperOG.c-gripperOG.a-y_A-gripperOG.b*sin(x(1))-gripperOG.k*sin(x(2));
    x(3)-gripperOG.r_i-gripperOG.b*cos(x(1))-(gripperOG.m+gripperOG.d)*cos(gripperOG.alpha+x(2))-gripperOG.f*cos(gripperOG.alpha+gripperOG.beta+x(2)-pi/2);
    x(4)-gripperOG.a-y_A-gripperOG.b*sin(x(1))-(gripperOG.m+gripperOG.d)*sin(gripperOG.alpha+x(2))-gripperOG.f*sin(gripperOG.alpha+gripperOG.beta+x(2)-pi/2)
]; % Equation of Motion of the mechanism. DoF y_A, the unknowns are position of thoretical
% peak point of the finger, angle of the connecting rod respect to
% horizontal axis and angle of the hypotenuse of the triangle respect to
% horizontal

gripperOG.x0 = [30*pi/180; 20*pi/180; 62; 148]; % randomly chosen initial vector
% for the solver iterations

gripperOG.x0 = fsolve (@(x)gripperOG.motion(0, x), gripperOG.x0, options);

gripperOG.motion(0, gripperOG.x0) % This should print [0;0;0;0] or almost
% on the console, for the dimensions of the gripper being compatible with
% the real mechanism

% To find theta, gamma, x_E and y_E for a certain y_A:
% x_sol = fsolve (@(x) gripper_original.motion(y_A, x), x0)
% x0 has to be a solution, possibly near the one u're trying to find, to be
% sure u have convergence in the right point

% gripper elisa
% From last experiments:
gripperE.r_e = 70; % mm - OF - radius of the base plate
gripperE.r_i = 40; % mm - AB - radius of the moving plate
gripperE.b = 30; % mm - BC - length of the connecting rod
gripperE.c = 76;  % mm - FD - length of the external link, between finger and base plate
% Once imposed x_E(y_A=0), gripper.c can be found in the next
% section
gripperE.a = 50; % mm - OP - distance of moving plate from fixed base (motor height = )
% Measured from real piece, constraint, if we put the motor on the base
% plate, we could make a hole on base plate and lower this value

gripperE.d = 20; % mm - GH == DI - left and right length of rectangular base of the finger 
gripperE.h = 120; % mm - height of the finger triangle
gripperE.n = 34; % mm - GD == HI - bases of the rectangular base of finger
gripperE.m = 0; % mm - CG - height of triangle in finger base ( = 0, rectangular base)
gripperE.f = sqrt(gripperE.h^2+(gripperE.n/2)^2); % mm - HE == IE - Side of finger
gripperE.k = sqrt(gripperE.n^2+gripperE.m^2); % mm - CD - hypotenuse
gripperE.alpha = atan (gripperE.n/gripperE.m); % rad - GCD - angle of the triangle of the base of the finger
gripperE.beta = atan(gripperE.h/(gripperE.n/2)); % rad - IHE == HIE - base angle of the finger
gripperE.l = 40; % mm - finger width

gripperE.motion =@(y_A, x) [ % x = [theta; gamma; x_E; y_E]
    gripperE.r_e-gripperE.r_i-gripperE.b*cos(x(1))-gripperE.k*cos(x(2));
    gripperE.c-gripperE.a-y_A-gripperE.b*sin(x(1))-gripperE.k*sin(x(2));
    x(3)-gripperE.r_i-gripperE.b*cos(x(1))-(gripperE.m+gripperE.d)*cos(gripperE.alpha+x(2))-gripperE.f*cos(gripperE.alpha+gripperE.beta+x(2)-pi/2);
    x(4)-gripperE.a-y_A-gripperE.b*sin(x(1))-(gripperE.m+gripperE.d)*sin(gripperE.alpha+x(2))-gripperE.f*sin(gripperE.alpha+gripperE.beta+x(2)-pi/2)
]; % Equation of Motion of the mechanism. DoF y_A, the unknowns are position of thoretical
% peak point of the finger, angle of the connecting rod respect to
% horizontal axis and angle of the hypotenuse of the triangle respect to
% horizontal

gripperE.x0 = [120*pi/180; 20*pi/180; 62; 148]; % randomly chosen initial vector
% for the solver iterations

gripperE.x0 = fsolve (@(x)gripperE.motion(0, x), gripperE.x0, options);

gripperE.motion(0, gripperE.x0) % This should print [0;0;0;0] or almost
% on the console, for the dimensions of the gripper being compatible with
% the real mechanism

gripperE.motor_h = 33.8+19.05; % Height of the motor+plastic thing
y_A_zero = gripperE.motor_h - gripperE.a % real y_A = 0 for our mechanism


% gripper ottimizzato
% From last experiments:
g_opt.r_e = 40; % mm - OF - radius of the base plate
g_opt.r_i = 20; % mm - AB - radius of the moving plate
g_opt.b = 10; % mm - BC - length of the connecting rod
g_opt.c = 67.6166;  % mm - FD - length of the external link, between finger and base plate
% Once imposed x_E(y_A=0), gripper.c can be found in the next
% section
g_opt.a = 56; % mm - OP - distance of moving plate from fixed base (motor height = )
% Measured from real piece, constraint, if we put the motor on the base
% plate, we could make a hole on base plate and lower this value

g_opt.d = 20; % mm - GH == DI - left and right length of rectangular base of the finger 
g_opt.h = 120; % mm - height of the finger triangle
g_opt.n = 25; % mm - GD == HI - bases of the rectangular base of finger
g_opt.m = 0; % mm - CG - height of triangle in finger base ( = 0, rectangular base)
g_opt.f = sqrt(g_opt.h^2+(g_opt.n/2)^2); % mm - HE == IE - Side of finger
g_opt.k = sqrt(g_opt.n^2+g_opt.m^2); % mm - CD - hypotenuse
g_opt.alpha = atan (g_opt.n/g_opt.m); % rad - GCD - angle of the triangle of the base of the finger
g_opt.beta = atan(g_opt.h/(g_opt.n/2)); % rad - IHE == HIE - base angle of the finger
g_opt.l = 40; % mm - finger width

g_opt.motion =@(y_A, x) [ % x = [theta; gamma; x_E; y_E]
    g_opt.r_e-g_opt.r_i-g_opt.b*cos(x(1))-g_opt.k*cos(x(2));
    g_opt.c-g_opt.a-y_A-g_opt.b*sin(x(1))-g_opt.k*sin(x(2));
    x(3)-g_opt.r_i-g_opt.b*cos(x(1))-(g_opt.m+g_opt.d)*cos(g_opt.alpha+x(2))-g_opt.f*cos(g_opt.alpha+g_opt.beta+x(2)-pi/2);
    x(4)-g_opt.a-y_A-g_opt.b*sin(x(1))-(g_opt.m+g_opt.d)*sin(g_opt.alpha+x(2))-g_opt.f*sin(g_opt.alpha+g_opt.beta+x(2)-pi/2)
]; % Equation of Motion of the mechanism. DoF y_A, the unknowns are position of thoretical
% peak point of the finger, angle of the connecting rod respect to
% horizontal axis and angle of the hypotenuse of the triangle respect to
% horizontal

g_opt.x0 = [120*pi/180; 20*pi/180; 62; 148]; % randomly chosen initial vector
% for the solver iterations

g_opt.x0 = fsolve (@(x)g_opt.motion(0, x), g_opt.x0, options);

g_opt.motion(0, g_opt.x0) % This should print [0;0;0;0] or almost
% on the console, for the dimensions of the gripper being compatible with
% the real mechanism


%% Testing and simulations
close all
clc
gripper = gripperE; % Gripper that u choose to test

[y_As, sols] = unknowns_plot(gripper, 30); % function which plots for all possible
% y_A, theta, gamma, x_E, y_E with the given gripper dimensions
dxE_dyA = (sols(3,end)-sols(3,1))/(y_As(end)-y_As(1))
pausa = 0.1; % seconds of pause between the change of the plot
dimensions = animate_gripper(gripper, y_As, sols, pausa); % Animates the
% gripper for the increasing y_A. If the last input is omitted, the 
% animation will go ahead when the users push a button of the keyboard

figure
hold on
plot (y_As, dimensions.C(1,:)) % Plot of X of the C point
plot (y_As, dimensions.H(1,:)) % Plot of X of the H point
grid on
legend ('X_C','X_H')
% This 2 plots should give an idea of the possibility of the gripper to
% contain a rock, when it has gripped one. I don't really know what would
% be necessary for a rock of 15 cm, maybe at mid of the finger like 12 cm
% of diameter complexivily
plot_gripper (gripper, 0)  % Gripper when closed
axis equal

thetas = sols(1,:);

aa = polyfit(y_As, thetas, 2)

theta_pol = @(x) aa(1).*x.^2 + aa(2).*x + aa(3);
figure
plot(y_As, thetas)
hold on
plot(y_As, theta_pol(y_As))
grid on
legend ('theta', 'poly-theta')

%% Going to find c, to have x_E_min at y_A = 0
% I want x_E = x_E_min at y_A = 0
% Variables: theta, gamma, y_E and c
gripper = g_opt;
x_E_min = gripper.l*sqrt(3)/6;

gripper.motion_c =@(y_A, x) [ % x = [theta; gamma; c; y_E]
    gripper.r_e-gripper.r_i-gripper.b*cos(x(1))-gripper.k*cos(x(2));
    x(3)-gripper.a-y_A-gripper.b*sin(x(1))-gripper.k*sin(x(2));
    x_E_min-gripper.r_i-gripper.b*cos(x(1))-(gripper.m+gripper.d)*cos(gripper.alpha+x(2))-gripper.f*cos(gripper.alpha+gripper.beta+x(2)-pi/2);
    x(4)-gripper.a-y_A-gripper.b*sin(x(1))-(gripper.m+gripper.d)*sin(gripper.alpha+x(2))-gripper.f*sin(gripper.alpha+gripper.beta+x(2)-pi/2)
]; % x_E_min becomes a constant and c a variable
gripper.x0 = [30*pi/180; 20*pi/180; 62; 148];



sol = fsolve (@(x)gripper.motion_c(0, x), gripper.x0) % Closed position
if norm(gripper.motion_c(0, sol)) <= 1e-6
    fprintf('SUCCESSO\n\n');
    gripper.c = sol(3)
    theta = sol(1);
    gamma = sol(2);
    x_E = x_E_min;
    y_E = sol(4);
    y_A = 0;
    
    O = [0;0];
    F = [gripper.r_e; 0];
    P = [0; gripper.a];
    A = [0; gripper.a+y_A]; 
    B = [gripper.r_i; gripper.a+y_A];
    C = B + gripper.b*[cos(theta); sin(theta)];
    D = C + gripper.k * [cos(gamma); sin(gamma)];
    G = C + gripper.m * [cos(gripper.alpha+gamma); sin(gripper.alpha+gamma)];
    E = [x_E; y_E];
    H = C + (gripper.m+gripper.d) * [cos(gripper.alpha+gamma); sin(gripper.alpha+gamma)];
    I = D + gripper.d * [cos(gripper.alpha+gamma); sin(gripper.alpha+gamma)];
    figure
    hold on
    link_plot(O, P, 'k');
    link_plot(P, A, 'r');
    link_plot(A, B, 'k');
    link_plot(B, C, 'k');
    link_plot(O, F, 'k');
    link_plot(F, D, 'k');
    
    link_plot(H, I, 'b');
    link_plot(G, D, 'b');
    link_plot(C, D, 'b');
    link_plot(C, G, 'b');
    link_plot(C, H, 'b');
    link_plot(C, D, 'b');
    link_plot(D, I, 'b');
    link_plot(H, E, 'b');
    link_plot(I, E, 'b');
    grid on
else
    fprintf('FALLIMENTO\n\n')
end
axis equal
%% going to find a, having to satisfy the existance of the gripper at y_A = 0

% I want x_E = x_E_min at y_A = 0
% Variables: theta, gamma, y_E and c

x_E_min = gripper.l*sqrt(3)/6;

gripper.motion_a =@(y_A, x) [ % x = [theta; gamma; a; y_E]
    gripper.r_e-gripper.r_i-gripper.b*cos(x(1))-gripper.k*cos(x(2));
    gripper.c-x(3)-y_A-gripper.b*sin(x(1))-gripper.k*sin(x(2));
    x_E_min-gripper.r_i-gripper.b*cos(x(1))-(gripper.m+gripper.d)*cos(gripper.alpha+x(2))-gripper.f*cos(gripper.alpha+gripper.beta+x(2)-pi/2);
    x(4)-x(3)-y_A-gripper.b*sin(x(1))-(gripper.m+gripper.d)*sin(gripper.alpha+x(2))-gripper.f*sin(gripper.alpha+gripper.beta+x(2)-pi/2)
]; % x_E_min becomes a constant and c a variable
gripper.x0 = [30*pi/180; 20*pi/180; 40; 148];

sol = fsolve (@(x)gripper.motion_a(0, x), gripper.x0) % Closed position
if norm(gripper.motion_a(0, sol)) <= 1e-6
    fprintf('SUCCESSO\n\n');
    gripper.a = sol(3)
    theta = sol(1);
    gamma = sol(2);
    x_E = x_E_min;
    y_E = sol(4);
    y_A = 0;
    
    O = [0;0];
    F = [gripper.r_e; 0];
    P = [0; gripper.a];
    A = [0; gripper.a+y_A]; 
    B = [gripper.r_i; gripper.a+y_A];
    C = B + gripper.b*[cos(theta); sin(theta)];
    D = C + gripper.k * [cos(gamma); sin(gamma)];
    G = C + gripper.m * [cos(gripper.alpha+gamma); sin(gripper.alpha+gamma)];
    E = [x_E; y_E];
    H = C + (gripper.m+gripper.d) * [cos(gripper.alpha+gamma); sin(gripper.alpha+gamma)];
    I = D + gripper.d * [cos(gripper.alpha+gamma); sin(gripper.alpha+gamma)];
    figure
    hold on
    link_plot(O, P, 'k');
    link_plot(P, A, 'r');
    link_plot(A, B, 'k');
    link_plot(B, C, 'k');
    link_plot(O, F, 'k');
    link_plot(F, D, 'k');
    
    link_plot(H, I, 'b');
    link_plot(G, D, 'b');
    link_plot(C, D, 'b');
    link_plot(C, G, 'b');
    link_plot(C, H, 'b');
    link_plot(C, D, 'b');
    link_plot(D, I, 'b');
    link_plot(H, E, 'b');
    link_plot(I, E, 'b');
    grid on
else
    fprintf('FALLIMENTO\n\n')
end
%% Functions

function [y_As, sols] = unknowns_plot(gripper, y_Af)
    options = optimoptions('fsolve','Display','none');
    if nargin == 1
        y_Af = 20;
    end
    if norm(gripper.motion(0, gripper.x0)) <= 1e-5 % else the gripper doesn't exists
        y_As = linspace (0, y_Af, 50); % All possible y_As
        sols = zeros (4, length(y_As)); % Initialization Matrix of solutions
        x0 = gripper.x0; % x0 for the numeric method fsolve()
        for ii = 1:length(y_As) % This cycle solves the equations of motion each time
            sols(:, ii) = fsolve(@(x)gripper.motion(y_As(ii), x), x0, options);  
            x0 = sols(:, ii);
            if norm(gripper.motion(y_As(ii), x0)) >= 1e-6
                fprintf ('This gripper has a problem at y_A = %f\n\n', y_As(ii));
                break;
            end
        end
        % Plotting what i found
        figure
        plot (y_As, sols(3, :), LineWidth=1.2) % X_E plot
        hold on
        plot (y_As, sols(4, :), LineWidth=1.2) % Y_E plot
        grid on
        legend ('x_E', 'y_E')
        figure
        plot(y_As, 180/pi*sols(1,:), LineWidth=1.2) % theta plot, gradi
        hold on
        plot(y_As, 180/pi*sols(2,:), LineWidth=1.2) % gamma plot, gradi
        grid on
        legend('theta', 'gamma')
    
    else
        y_As = 0;
        sols = 0;
        disp('Non esiste sto gripper bruh')
    end
end

function dimensions = animate_gripper(gripper, y_As, sols, pausa)
% Function which animates the gripper, for the range of y_As given
% if input "pausa" is omitted, the animation will go ahead if the user
% pushes a key
% dimensions is a variable useful for checking the feasability and
% compatibility with the task. It contains all the lenght of the links
% found (if at any point it's not the nominal, something's wrong) and all 
% the positions of the points at any instant

    y_A = y_As(1);
    theta = sols(1,1);
    gamma = sols(2,1);
    x_E = sols(3,1);
    y_E = sols(4,1);

    O = [0;0];
    F = [gripper.r_e; 0];
    P = [0; gripper.a];
    A = [0; gripper.a+y_A]; 
    B = [gripper.r_i; gripper.a+y_A];
    C = B + gripper.b*[cos(theta); sin(theta)];
    D = C + gripper.k * [cos(gamma); sin(gamma)];
    G = C + gripper.m * [cos(gripper.alpha+gamma); sin(gripper.alpha+gamma)];
    E = [x_E; y_E];
    H = C + (gripper.m+gripper.d) * [cos(gripper.alpha+gamma); sin(gripper.alpha+gamma)];
    I = D + gripper.d * [cos(gripper.alpha+gamma); sin(gripper.alpha+gamma)];
    
    figure
    hold on
    OP = link_plot(O, P, 'k');
    PA = link_plot(P, A, 'r');
    AB = link_plot(A, B, 'k');
    BC = link_plot(B, C, 'k');
    OF = link_plot(O, F, 'k');
    FD = link_plot(F, D, 'k');

    HI = link_plot(H, I, 'b');
    GD = link_plot(G, D, 'b');
    CD = link_plot(C, D, 'b');
    CG = link_plot(C, G, 'b');
    CH = link_plot(C, H, 'b');
    DI = link_plot(D, I, 'b');
    HE = link_plot(H, E, 'b');
    IE = link_plot(I, E, 'b');
    
    dimensions.O = zeros(2,1);
    dimensions.O(:,1) = O;
    dimensions.F = zeros(2,1);
    dimensions.F(:,1) = F;
    dimensions.P = zeros(2,1);
    dimensions.P(:,1) = P;
    dimensions.A = zeros(2,1);
    dimensions.A(:,1) = A;
    dimensions.B = zeros(2,1);
    dimensions.B(:,1) = B;
    dimensions.C = zeros(2,1);
    dimensions.C(:,1) = C;
    dimensions.D = zeros(2,1);
    dimensions.D(:,1) = D;
    dimensions.G = zeros(2,1);
    dimensions.G(:,1) = G;
    dimensions.E = zeros(2,1);
    dimensions.E(:,1) = E;
    dimensions.H = zeros(2,1);
    dimensions.H(:,1) = H;
    dimensions.I = zeros(2,1);
    dimensions.I(:,1) = I;

    dimensions.OP = zeros(1,length(y_As));
    dimensions.OP(:,1) = norm(P-O);
    dimensions.PA = zeros(1,length(y_As));
    dimensions.PA(:,1) = norm(P-A);
    dimensions.AB = zeros(1,length(y_As));
    dimensions.AB(:,1) = norm(A-B);
    dimensions.BC = zeros(1,length(y_As));
    dimensions.BC(:,1) = norm(B-C);
    dimensions.OF = zeros(1,length(y_As));
    dimensions.OF(:,1) = norm(F-O);
    dimensions.FD = zeros(1,length(y_As));
    dimensions.FD(:,1) = norm(F-D);
    dimensions.HI = zeros(1,length(y_As));
    dimensions.HI(:,1) = norm(H-I);
    dimensions.GD = zeros(1,length(y_As));
    dimensions.GD(:,1) = norm(G-D);
    dimensions.CD = zeros(1,length(y_As));
    dimensions.CD(:,1) = norm(C-D);
    dimensions.CG = zeros(1,length(y_As));
    dimensions.CG(:,1) = norm(C-G);
    dimensions.CH = zeros(1,length(y_As));
    dimensions.CH(:,1) = norm(C-H);
    dimensions.DI = zeros(1,length(y_As));
    dimensions.DI(:,1) = norm(H-E);
    dimensions.HE = zeros(1,length(y_As));
    dimensions.HE(:,1) = norm(H-E);
    dimensions.IE = zeros(1,length(y_As));
    dimensions.IE(:,1) = norm(I-E);
    grid on
    if nargin == 4
        pause(pausa)
    else
        pause();
    end
   for ii = 2:length(sols(1,:))
       if norm(gripper.motion(y_As(ii), sols(:,ii))) <= 1e-6
        y_A = y_As(ii);
        theta = sols(1,ii);
        gamma = sols(2,ii);
        x_E = sols(3,ii);
        y_E = sols(4,ii);

        O = [0;0];
        F = [gripper.r_e; 0];
        P = [0; gripper.a];
        A = [0; gripper.a+y_A]; 
        B = [gripper.r_i; gripper.a+y_A];
        C = B + gripper.b*[cos(theta); sin(theta)];
        D = C + gripper.k * [cos(gamma); sin(gamma)];
        G = C + gripper.m * [cos(gripper.alpha+gamma); sin(gripper.alpha+gamma)];
        E = [x_E; y_E];
        H = C + (gripper.m+gripper.d) * [cos(gripper.alpha+gamma); sin(gripper.alpha+gamma)];
        I = D + gripper.d * [cos(gripper.alpha+gamma); sin(gripper.alpha+gamma)];
        
        link_animate (O, P, OP);
        link_animate (P, A, PA);
        link_animate (A, B, AB);
        link_animate (B, C, BC);
        link_animate (O, F, OF);
        link_animate (O, P, OP);
        link_animate (F, D, FD);
        link_animate (H, I, HI);
        link_animate (G, D, GD);
        link_animate (C, D, CD);
        link_animate (C, G, CG);
        link_animate (C, H, CH);
        link_animate (D, I, DI);
        link_animate (H, E, HE);
        link_animate (I, E, IE);

        dimensions.O(:,ii) = O;
        dimensions.F(:,ii) = F;
        dimensions.P(:,ii) = P;
        dimensions.A(:,ii) = A;
        dimensions.B(:,ii) = B;
        dimensions.C(:,ii) = C;
        dimensions.D(:,ii) = D;
        dimensions.G(:,ii) = G;
        dimensions.E(:,ii) = E;
        dimensions.H(:,ii) = H;
        dimensions.I(:,ii) = I;

        dimensions.OP(:,ii) = norm(P-O);
        dimensions.PA(:,ii) = norm(P-A);
        dimensions.AB(:,ii) = norm(A-B);
        dimensions.BC(:,ii) = norm(B-C);
        dimensions.OF(:,ii) = norm(F-O);
        dimensions.FD(:,ii) = norm(F-D);
        dimensions.HI(:,ii) = norm(H-I);
        dimensions.GD(:,ii) = norm(G-D);
        dimensions.CD(:,ii) = norm(C-D);
        dimensions.CG(:,ii) = norm(C-G);
        dimensions.CH(:,ii) = norm(C-H);
        dimensions.DI(:,ii) = norm(H-E);
        dimensions.HE(:,ii) = norm(H-E);
        dimensions.IE(:,ii) = norm(I-E);
        axis equal
        if nargin == 4
            pause(pausa)
        else
            pause();
        end
       else
           fprintf('Animazione terminata a y_A = %f\n\n', y_As(ii));
           break;
       end
   end
end

function [sol] = plot_gripper (gripper, y_A)
% This function plots in red the vertical coordinate, in blue the
% finger and in black the rest of the links, of the gripper, given a
% certain gripper object and a vertical coordinate.
% It also gives as output the solution vector x = [theta; gamma; x_E; y_E]

    sol = fsolve (@(x)gripper.motion(y_A,x), gripper.x0);
    if norm(gripper.motion(y_A,sol)) <= 1e-6
        theta = sol(1);
        gamma = sol(2);
        x_E = sol(3);
        y_E = sol(4);
    
        O = [0;0];
        F = [gripper.r_e; 0];
        P = [0; gripper.a];
        A = [0; gripper.a+y_A]; 
        B = [gripper.r_i; gripper.a+y_A];
        C = B + gripper.b*[cos(theta); sin(theta)];
        D = C + gripper.k * [cos(gamma); sin(gamma)];
        G = C + gripper.m * [cos(gripper.alpha+gamma); sin(gripper.alpha+gamma)];
        E = [x_E; y_E];
        H = C + (gripper.m+gripper.d) * [cos(gripper.alpha+gamma); sin(gripper.alpha+gamma)];
        I = D + gripper.d * [cos(gripper.alpha+gamma); sin(gripper.alpha+gamma)];
        figure
        hold on
        link_plot(O, P, 'k');
        link_plot(P, A, 'r');
        link_plot(A, B, 'k');
        link_plot(B, C, 'k');
        link_plot(O, F, 'k');
        link_plot(F, D, 'k');
    
        link_plot(H, I, 'b');
        link_plot(G, D, 'b');
        link_plot(C, D, 'b');
        link_plot(C, G, 'b');
        link_plot(C, H, 'b');
        link_plot(D, I, 'b');
        link_plot(H, E, 'b');
        link_plot(I, E, 'b');
        grid on
        % finger plot,
        % figure
        % hold on
        % link_plot (C, H, 'k');
        % link_plot (C, G, 'k');
        % link_plot (G, I, 'k');
        % link_plot (H, I, 'k');
        % link_plot (G, E, 'k');
        % link_plot (I, E, 'k');
    else
        disp('bruh non esiste sto gripper')
    end
end

function IJ = link_plot (I, J, color)
% Function which, given coordinates of point I and point J, plots a beam of
% color 
    IJ = plot([I(1);J(1)],[I(2);J(2)],color, LineWidth=1.2);
end

function link_animate (I, J, IJ)
    set(IJ, 'XData', [I(1), J(1)], 'YData', [I(2), J(2)]);
end