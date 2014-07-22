function [estimates] = kinematics_exact_solution(U,P)

%     5.9670    4.5424    0.0370    6.1132
%
%   Columns 5 through 8
%
%     0.9011    4.4560    4.3789    5.9497
%
%   Columns 9 through 12
%
%     5.8731    4.5456    0.0196    6.1164

% MCP Chain
exo_t1_rel = U(1);
exo_t6_rel = U(2);
exo_t9_rel = U(3);

% Solving kinematics of MCP chain
exo_t1 = 2*pi-exo_t1_rel;

% Geometric Parameters
% Lengths
x_A = P(1); % m
y_A = P(2); % m
l_BC = P(3); % m
l_CD = P(4); % m

% Kinematics Equations
x = solve_quad_eqn(x_A-l_BC*cos(exo_t1), l_BC*sin(exo_t1)-y_A, -l_CD);

% (x_A-l_BC*cos(exo_t1))*sin(x(1))+(l_BC*sin(exo_t1)-y_A)*cos(x(1))-l_CD

t_mcp = x(2);
exo_t2 = t_mcp - pi/2;
exo_x3 = sqrt((l_BC*cos(exo_t1)+l_CD*sin(t_mcp)-x_A)^2+(l_BC*sin(exo_t1)-l_CD*cos(t_mcp)-y_A)^2);

%% PIP Chain
% Geometric Parameters
% Lengths
l_AH = P(5);
l_CE = P(6);
l_EF = P(7);
l_FG = P(8);
l_GH = P(9);

% Evaluating model parameters
% PIP Chain
l_HF = sqrt(l_GH*l_GH+l_FG*l_FG);
t_HFG = atan(l_GH/l_FG);
l_DH = l_AH-exo_x3;
l_CH = sqrt(l_CD*l_CD+l_DH*l_DH);
t_HFG = atan(l_GH/l_FG);
t_DCH = atan(l_DH/l_CD);

% Kinematics Equations
x = solve_quad_eqn(2*l_CH*l_EF*sin(exo_t6_rel),2*l_CH*(l_EF*cos(exo_t6_rel)-l_CE),...
    l_CH^2+l_CE^2+l_EF^2-l_HF^2-2*l_CE*l_EF*cos(exo_t6_rel));

% 2*l_CH*l_EF*sin(exo_t6_rel)*sin(x(1))+...
%     2*l_CH*(l_EF*cos(exo_t6_rel)-l_CE)*cos(x(1))+...
% l_CH^2+l_CE^2+l_EF^2-l_HF^2-2*l_CE*l_EF*cos(exo_t6_rel);

exo_t5 = exo_t2+t_DCH-x(2)-2*pi;
exo_t7 = atan2(l_CH*sin(exo_t2+t_DCH)-l_CE*sin(exo_t5)+l_EF*sin(exo_t5+exo_t6_rel),...
    l_CH*cos(exo_t2+t_DCH)-l_CE*cos(exo_t5)+l_EF*cos(exo_t5+exo_t6_rel))+t_HFG;

exo_t7 = wrapToPi(exo_t7);
exo_t6 = exo_t5+pi+exo_t6_rel;
% if(exo_t5<0)
    
if(exo_t7<0)
    t_pip = 2*pi+exo_t7+pi/2;
else
    t_pip = exo_t7+pi/2;
end

if(t_pip<pi)
    disp('t_pip less!');
end
%% DIP Chain
% Solving kinematics of DIP chain
% Geometric Parameters
% Lengths
l_FG = P(8);
l_GK = P(10);
l_FI = P(11);
l_IJ = P(12);

t_GFK = atan(l_GK/l_FG);
l_KF = sqrt(l_GK*l_GK+l_FG*l_FG);
exo_t9 = pi + exo_t6 - exo_t9_rel;

% Kinematics Equations
x = solve_quad_eqn(l_FI*cos(exo_t9)-l_KF*cos(exo_t7+t_GFK),...
    -l_FI*sin(exo_t9)+l_KF*sin(exo_t7+t_GFK),l_IJ);
% (l_FI*cos(exo_t9)-l_KF*cos(exo_t7+t_GFK))*sin(x(1))+...
%     (-l_FI*sin(exo_t9)+l_KF*sin(exo_t7+t_GFK))*cos(x(1))+l_IJ;

if(isnan(x))
    t_dip = 2*pi;
elseif(x(1)<0)
    t_dip = x(1)+2*pi;
else
    t_dip = x(1);
end
    

exo_t10 = t_dip-pi/2;

exo_x11 = sqrt((l_FI*cos(exo_t9)+l_IJ*cos(exo_t10)-l_KF*cos(exo_t7+t_GFK))^2+...
    (l_FI*sin(exo_t9)+l_IJ*sin(exo_t10)-l_KF*sin(exo_t7+t_GFK))^2);

exo_x11 = 0.005;
t_dip = (t_mcp+t_pip+10*pi/180);
exo_t10 = t_dip-pi/2;
%%
estimates = [exo_t1, exo_t2, exo_x3, t_mcp, exo_t5, exo_t6, exo_t7, t_pip,...
    exo_t9, exo_t10, exo_x11, t_dip];
