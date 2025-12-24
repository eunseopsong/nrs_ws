%% This example is made for KUKA iiwa 7

% Update Date: 06/23/2025
% Prudcted by: Jaeyun Sim
% E-mail: wodbs1118@g.skku.edu

% Description:
% - This code generate HTM & Jacobian using DH-parameter.
% - And you can use this matrices in C++ by copying.
% - NOTE! This code support robots consisting rotational joints only.
%         And it supports craig's version

%% System Setup (Normally.. Do Not Touch)
% Description: it only supports the 10-axis.

clear; clc; close all;
addpath("Func_Common")
syms th1 th2 th3 th4 th5 th6 th7 th8 th9 th10
syms a1 a2 a3 a4 a5 a6 a7 a8 a9 a10
syms d1 d2 d3 d4 d5 d6 d7 d8 d9 d10


%% DH-Parameter Setup (Write the proper DH-parameter)
% - Type the syms only (except for pi, 0)

% Symblic value setting to generate HTM
DH_al = [   0, -pi/2, pi/2, pi/2, -pi/2, -pi/2, pi/2];
DH_a  = [   0,     0,    0,    0,     0,     0,    0];
DH_d  = [  d1,     0,   d3,    0,    d5,     0,   d7];
DH_th = [ th1,   th2,  th3,  th4,   th5,   th6,  th7];

% Actual value setting to verify (HTM & Jacobian)
VER_a  = [   0,     0,    0,    0,     0,     0,    0]; % Unit: mm
VER_d  = [ 340,     0,  400,    0,   400,     0,  126]; % Unit: mm
VER_th = [   0,     0,    0,    0,     0,     0,    0]; % Unit: rad
VER_tq = [   0,     1,    0,    0,     0,     0,    0]; % Used in jacobian veri. (Unit: Nm)

%% HTM Generation (Do Not Touch)
% - This construct the HTM
% - The arguments to Forward_kinemaics are composed sequencially of alpha, a, d, theta.

for i = 1:length(DH_al)
    Tp(:,:,i) = Forward_kinemaics(DH_al(i), DH_a(i), DH_d(i), DH_th(i));
end


%% Process Notification (Do Not Touch)
To(:,:,1) = Tp(:,:,1); % T01
for i = 2:size(Tp,3)
    To(:,:,i) = simplify(To(:,:,i-1)*Tp(:,:,i));
    fprintf("FK %d th iter finished \n",i);
end
disp("Forward kinematics step done")
disp(To(:,:,size(Tp,3))) % Display the data


%% Jacobian calculation (Do Not Touch)
% 1) Find origin points
for i = 1:size(Tp,3)
    Or(:,:,i) = To(1:3,4,i);
    Axis_Z(:,:,i) = To(1:3,3,i);
end

% 2) Calculate the jacobians
Jv = simplify(cross(Axis_Z(:,:,1), Or(:,:,size(Tp,3)) - Or(:,:,1)));
Jw = Axis_Z(:,:,1);
for i = 2:size(Tp,3)
    Jv = [Jv, simplify(cross(Axis_Z(:,:,i), Or(:,:,size(Tp,3)) - Or(:,:,i)))];
    Jw = [Jw, Axis_Z(:,:,i)];
end

% 3) Merging
J = [Jv;Jw]; % Jacobian

disp("Jacobian step done")
disp(J)  % Display the data


%% Verification of Forward kinematics & Jacobian

% < Forward kinematics >

for i=1:length(DH_al)
To(:,:,size(Tp,3)) = subs(To(:,:,size(Tp,3)),{DH_a(i),DH_d(i),DH_th(i)},[VER_a(i),VER_d(i),VER_th(i)]);
J = subs(J,{DH_a(i),DH_d(i),DH_th(i)},[VER_a(i),VER_d(i),VER_th(i)]);
end
Total_force = J*VER_tq';

fprintf("X(mm): %.1f, Y(mm): %.1f, Z(mm): %.1f \n",To(1:3,4,size(Tp,3)));
fprintf("Fx(N): %.2f, Fy(N): %.2f, Fz(N): %.2f \n", Total_force(1:3)/1000);