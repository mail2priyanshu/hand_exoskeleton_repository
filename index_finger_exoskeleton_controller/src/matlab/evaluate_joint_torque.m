function [tau_finger,tau_finger_dot,K1,K2] = evaluate_joint_torque(t4,t8,...
    t4_dot,t8_dot)

%% MCP Joint torque evaluation
% MCP (Nm)
A_mcp = 1.01;
B_mcp = 0.05;
C_mcp = 3.39;
D_mcp = 0.05;
E_mcp = 70.96;
F_mcp = 13.68;

t4 = wrapTo2Pi(t4);

% if(t4<3*pi/2+0.5)
%     disp('mcp exceeding!');
% end

if(t4>pi)
    t_mcp = (2*pi- t4)*180/pi;
elseif(t4<pi/2)
    t_mcp = -t4*180/pi;
else
    disp('t_mcp exceeding limits!');
    t_mcp = (2*pi- t4)*180/pi;
end


% b_mcp = 0.005; % damping coefficient at MCP joint (Nm/rad/s)
b_mcp = 0.00; % damping coefficient at MCP joint (Nm/rad/s)
tau_mcp = ((A_mcp*(exp(-B_mcp*(t_mcp-E_mcp))-1) -...
    C_mcp*(exp(D_mcp*(t_mcp-F_mcp))-1))*10^(-3))-b_mcp*t4_dot; % (Nm)
%% NOTE - tau_pip_dot does not include the derivative of damping term
tau_mcp_dot = (A_mcp*B_mcp*exp(-B_mcp*(t_mcp-E_mcp)) +...
    C_mcp*D_mcp*exp(D_mcp*(t_mcp-F_mcp)))*10^(-3)*t4_dot; % t4_dot = -t_mcp_dot

% adapting for torque sign convention (couterclockwise positive)
tau_mcp = -tau_mcp; 
tau_mcp_dot = -tau_mcp_dot;

% Expressions for second derivative of finger torque at MCP joint
K_1m = (-A_mcp*B_mcp^2*exp(-B_mcp*(t_mcp-E_mcp)) +...
    C_mcp*D_mcp^2*exp(D_mcp*(t_mcp-F_mcp)))*10^(-3);
K_2m = (A_mcp*B_mcp*exp(-B_mcp*(t_mcp-E_mcp)) +...
    C_mcp*D_mcp*exp(D_mcp*(t_mcp-F_mcp)))*10^(-3);
%% PIP Joint torque evaluation
% Proximal (Nm)
A_pip = 0.6992;
B_pip = 0.05;
C_pip = 2.3469;
D_pip = 0.05;
E_pip = 36.2987;
F_pip = 23.3078;

t8_wrapped = wrapTo2Pi(t8);

if(t8_wrapped>pi)
    t_pip = (2*pi-t8_wrapped)*180/pi-t_mcp;
elseif(t8_wrapped<pi/2)
    t_pip = -t8_wrapped*180/pi-t_mcp;
else
    disp('t_pip exceeding limits!');
    t_pip = (2*pi-t8_wrapped)*180/pi-t_mcp;
end

% t_pip = (2*pi-t8)*180/pi-t_mcp;
% b_pip = 0.005; % damping coefficient at PIP joint (Nm/rad/s)
b_pip = 0.00; % damping coefficient at PIP joint (Nm/rad/s)
% tau_pip = -(A_pip*(exp(-B_pip*(t_pip-E_pip))-1) -...
%     C_pip*(exp(D_pip*(t_pip-F_pip))-1))*10^(-3)-b_pip*(t4_dot-t8_dot); % (Nm)
% tau_pip_dot = A_pip*B_pip*exp(B_pip*(t_pip-E_pip)) +...
%     C_pip*D_pip*exp(D_pip*(t_pip-F_pip));

tau_pip = (A_pip*(exp(-B_pip*(t_pip-E_pip))-1) -...
    C_pip*(exp(D_pip*(t_pip-F_pip))-1))*10^(-3)-b_pip*(t8_dot-t4_dot); % (Nm)
%% NOTE - tau_pip_dot does not include the derivative of damping term
tau_pip_dot = (A_pip*B_pip*exp(-B_pip*(t_pip-E_pip)) +...
    C_pip*D_pip*exp(D_pip*(t_pip-F_pip)))*10^(-3)*t8_dot; % t8_dot = -t_pip_dot

% adapting for torque sign convention (couterclockwise positive)
tau_pip = -tau_pip;
tau_pip_dot = -tau_pip_dot;

% Expressions for second derivative of finger torque at PIP joint
K_1p = (-A_mcp*B_mcp^2*exp(-B_mcp*(t_mcp-E_mcp)) +...
    C_mcp*D_mcp^2*exp(D_mcp*(t_mcp-F_mcp)))*10^(-3);
K_2p = (A_mcp*B_mcp*exp(-B_mcp*(t_mcp-E_mcp)) +...
    C_mcp*D_mcp*exp(D_mcp*(t_mcp-F_mcp)))*10^(-3);

tau_finger = [tau_mcp; tau_pip];
tau_finger_dot = [tau_mcp_dot; tau_pip_dot];
K1 = [K_1m, 0;
    0, K_1p];
K2 = [K_2m, 0;
     0, K_2p];
% [tau_mcp tau_mcp_dot]