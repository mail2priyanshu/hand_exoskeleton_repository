function [tau_finger,...
    tau_finger_dot, tau_exo,tau_exo_dot,...
    J,J_dot,K_1,K_2,K1,K2,Jn,Jn_dot] = statics_solution(estimates,P,exo_t_dot)

% Position input
exo_t1 = estimates(1);
exo_x3 = estimates(3);
t_mcp = estimates(4);
exo_t5 = estimates(5);
exo_t6 = estimates(6);
exo_t7 = estimates(7);
t_pip = estimates(8);
% t9 = estimates(9);
% exo_t10 = estimates(10);
% x11 = estimates(11);
% exo_t12 = estimates(12);

exo_t6r = exo_t6-exo_t5-pi;

% Velocity input
exo_t1_dot = exo_t_dot(1);
exo_t6r_dot = exo_t_dot(2);

% Geometric Parameters
% Lengths
x_A = P(1); % m
y_A = P(2); % m
l_BC = P(3); % m
l_CD = P(4); % m
l_AH = P(5);
l_CE = P(6);
l_EF = P(7);
l_FG = P(8);
l_GH = P(9);
l_GK = P(10);
l_FI = P(11);
l_IJ = P(12);

% t_GFK = atan(l_GK/l_FG);
% l_KF = sqrt(l_GK*l_GK+l_FG*l_FG);
l_FH = sqrt(l_GH*l_GH+l_FG*l_FG);

t_HFG = atan(l_GH/l_FG);
l_DH = l_AH-exo_x3;
l_CH = sqrt(l_CD*l_CD+l_DH*l_DH);
t_HFG = atan(l_GH/l_FG);
t_DCH = atan(l_DH/l_CD);

J = index_finger_J(exo_t1,exo_t5,exo_x3,exo_t6r,l_AH,l_BC,l_CD,...
    l_CE,l_EF,l_FH,t_HFG,t_mcp,t_pip);

X_dot = J*exo_t_dot;
exo_x3_dot = X_dot(1);
t_mcp_dot = X_dot(2);
exo_t5_dot = X_dot(3);
t_pip_dot = X_dot(4);

J_dot = index_finger_J_dot(exo_t1,exo_t5,exo_x3,exo_t6r,exo_t1_dot,...
    exo_t5_dot,exo_x3_dot,exo_t6r_dot,l_AH,l_BC,l_CD,l_CE,l_EF,l_FH,...
    t_HFG,t_mcp,t_mcp_dot,t_pip,t_pip_dot);

% Acceleration calculation
% X_ddot = J_dot*theta_dot+J*theta_ddot;

[tau_finger,tau_finger_dot,K1,K2] = evaluate_joint_torque(t_mcp,t_pip,...
    t_mcp_dot,t_pip_dot);
f3 = 0;
tau_5 = 0;

tau_mcp = tau_finger(1);
tau_pip = tau_finger(2);
tau_mcp_dot = tau_finger_dot(1);
tau_pip_dot = tau_finger_dot(2);

F = [f3; tau_mcp; tau_5; tau_pip];
tau_exo = J'*F; % reaction force on the exoskeleton due to torque at the finger
F_dot = [0; tau_mcp_dot; 0; tau_pip_dot];
tau_exo_dot = (J_dot'*F+J'*F_dot);

Jn = [J(2,1), J(2,2);
    J(4,1), J(4,2)];

Jn_dot = [J_dot(2,1), J_dot(2,2);
          J_dot(4,1), J_dot(4,2)];

K_1 = K1*Jn+K2*Jn_dot;
K_2 = K2*Jn;

% -K(1,1)*(J(2,1)*exo_t_dot(1)+J(2,2)*exo_t_dot(2))-K(1,2)*(J(2,1)*(-0.2326)+J(2,2)*0.6571+J_dot(2,1)*exo_t_dot(1)+J_dot(2,2)*exo_t_dot(2))
% -K(2,1)*(J(4,1)*exo_t_dot(1)+J(4,2)*exo_t_dot(2))-K(2,2)*(J(4,1)*(-0.2326)+J(4,2)*0.6571+J_dot(4,1)*exo_t_dot(1)+J_dot(4,2)*exo_t_dot(2))
% 
% K1 = [K(1,2)*J(2,1), K(1,2)*J(2,2);
%     K(2,2)*J(4,1), K(2,2)*J(4,2)];
% K2 = [K(1,1)*J(2,1)+K(1,2)*J_dot(2,1),...
%     K(1,1)*J(2,2)+K(1,2)*J_dot(2,2);
%     K(2,1)*J(4,1)+K(2,2)*J_dot(4,1),...
%     K(2,1)*J(4,2)+K(2,2)*J_dot(4,2)];
% K1\(-yd_ddot-K2*exo_t_dot)
% % 
% % exo_t_ddot =
% % 
% %     0.2068
% %    -0.5841
% %    
% yd_ddot =   1.0e-03*[-0.2756;-0.5512]