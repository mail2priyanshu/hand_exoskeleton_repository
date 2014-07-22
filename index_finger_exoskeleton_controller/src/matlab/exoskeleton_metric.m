clc
clear

%% INITIALISATION AND PARAMETER DEFINITION:
% =========================================

if(exist('../../../../../../MATLAB/export_fig','dir'))
    addpath('../../../../../../MATLAB/export_fig');
    print_flag = 1; % print figure PDF
else
    print_flag = 0; % do not print figure PDF
end

line_thickness = 2;
fontsize = 20;
pattern_str = {'-b','--r','-+k','-.c','-*y','.m','--b','--k'};
legend_location = 'NorthEast';

if(print_flag)
    set(0,'DefaultAxesPosition',[0.1900    0.1300    0.7250    0.7950])
    set(0, 'DefaultFigurePosition', [100,200,500,400]);
    set(0,'DefaultAxesFontSize',fontsize-2);
    set(0,'DefaultTextFontSize',fontsize);
%     set(0,'DefaultLegendInterpreter','latex');
    close all;
end

% System Parameters
global r_m r_j start_ind end_ind
r_m = 1.09375*0.0254;
r_j = 0.4635*0.0254;

control_type = {'FF_Exo_MCP','FF_PID_Exo_MCP','FF_MCP','FF_PID_MCP',...
    'FF_Exo_PIP','FF_PID_Exo_PIP','FF_PIP','FF_PID_PIP',...
    'FF_Exo_MCP_PIP','FF_PID_Exo_MCP_PIP','FF_MCP_PIP',...
    'FF_PID_MCP_PIP','DT_MCP_PIP'};

%% File Override
% filename ='../data/gain_test/sensor_data_FF_PID_PIP_Exo_f_0.500000_tau_pip_0.060000_kp_11_0.csv';

% filename ='../data/sensor_data_FF_PID_MCP_PIP_Exo_f_0.500000_tau_mcp_0.250000_tau_pip_0.040000_0.csv';
% end_ind = 1000;
% ct = 10;

% filename ='../data/sensor_data_FF_PID_MCP_f_0.500000_tau_mcp_0.200000_tau_pip_0.000000_0.csv';
% ct = 3;

% filename ='../data/sensor_data_FF_PID_PIP_f_0.500000_tau_mcp_0.000000_tau_pip_0.010000_0.csv';
% end_ind = 3000;
% ct = 7;

% filename ='../data/sensor_data_FF_PID_PIP_f_0.500000_tau_mcp_0.000000_tau_pip_0.025000_0.csv';
% end_ind = 5000;
% ct = 7;

% filename ='../data/sensor_data_FF_PID_PIP_f_0.500000_tau_mcp_0.000000_tau_pip_0.030000_0.csv';
% % filename ='../data/sensor_data_FF_PID_MCP_PIP_f_0.500000_tau_mcp_0.150000_tau_pip_0.020000_0.csv';
% % filename ='../data/sensor_data_FF_PID_MCP_PIP_f_0.500000_tau_mcp_0.200000_tau_pip_0.035000_0.csv';
% end_ind = 1000;
% ct = 12;

% offset = 30;
% f = 0.5;
% start_ind = 1;
% end_ind = 195;
% start_ind_long = 1;
% end_ind_long = 1000;
% print_flag = 0;

%% Exoskeleton Joint Torque Control
% % filename ='../data/sensor_data_FF_PID_MCP_PIP_Exo_f_0.200000_tau_mcp_0.250000_tau_pip_0.050000_0.csv';

% filename ='../data/ff_pid_exo/sensor_data_FF_PID_MCP_PIP_Exo_f_0.500000_tau_mcp_0.250000_tau_pip_0.060000_0.csv';
% ct = 10;
% % offset = 30;
% f = 0.5;
% start_ind = 1;
% end_ind = 3500;
% print_flag = 1;
% plot_dir = 'figures/MCP_PIP_exo'; % directory to export plot PDF
% legend_location = {'NorthOutside','NorthEast','SouthEast','NorthOutside',...
%     'NorthEast','SouthEast','NorthOutside','NorthOutside','NorthOutside','NorthOutside',...
%     'NorthEast','NorthEast','NorthEast','EastOutside','NorthEast','NorthEast'};

%% Finger Joint Torque Control
% filename ='../data/ff_pid_fin/sensor_data_FF_PID_MCP_PIP_f_0.500000_tau_mcp_0.200000_tau_pip_0.035000_0.csv';
% ct = 12;
% f = 0.5;
% start_ind = 1;
% end_ind = 3500;
% plot_dir = 'figures/MCP_PIP_finger'; % directory to export plot PDF
% print_flag = 1;
% % end_ind = size(data,1);
% fontsize = 20;
% 
% 
% % set(0,'DefaultAxesPosition',[0.1400    0.1100    0.7450    0.8150])
% legend_location = {'NorthOutside','NorthEast','NorthOutside','NorthOutside',...
% 'NorthOutside','NorthOutside','NorthEast','NorthEast','SouthEast','NorthEast',...
%     'NorthEast','NorthEast','NorthEast','EastOutside','NorthEast','NorthEast'};
% ylim_tau_mcp = [-0.2 0.25];

%% Dynamic Transparency
% % MCP
% filename ='../data/sensor_data_DT_MCP_PIP_f_0.500000_tau_mcp_0.000000_tau_pip_0.000000_0.csv';
% ct = 13;
% f = 0.5;
% start_ind = 3500;
% end_ind = 4700;
% print_flag = 0;

% % PIP
% filename ='../data/DT/sensor_data_DT_MCP_PIP_f_0.500000_tau_mcp_0.000000_tau_pip_0.000000_0.csv';
% ct = 13;
% f = 0.5;
% start_ind = 1;
% end_ind = 5000;
% print_flag = 0;

% MCP + PIP
filename ='../data/DT/sensor_data_DT_MCP_PIP_f_0.500000_tau_mcp_0.000000_tau_pip_0.000000_0.csv';
ct = 13;
f = 0.5;
% start_ind = 3500;
% end_ind = 4700;
start_ind = 1;
end_ind = 7000;
print_flag = 1;
plot_dir = 'figures/MCP_PIP_DT'; % directory to export plot PDF
% set(0,'DefaultAxesPosition',[0.1800    0.1100    0.7450    0.8050])
legend_location = {'NorthOutside','NorthOutside','NorthOutside','NorthOutside',...
'NorthOutside','NorthOutside','NorthOutside','NorthOutside','SouthEast','NorthEast',...
    'NorthEast','NorthEast','NorthEast','EastOutside','NorthEast','NorthEast'};
ylimit_tau_mcp = [-0.4 0.3];
ylimit_tau_pip = [-0.1 0.06];

% legend_location = {'NorthOutside','NorthEast','NorthEast','NorthEast',...
% 'NorthEast','NorthEast','NorthEast','NorthEast','NorthEast','NorthEast',...
%     'NorthEast','NorthEast','NorthEast','NorthEast','NorthEast','NorthEast'};
%% Metric Evaluation
% if(print_flag)
%     diary([filename(1:end-4) '.txt']);
% end

data = dlmread(filename);

% end_ind = length(data);

time = data(:,1); % time in stored s
% exo_t1 exo_t2 exo_x3 t_mcp exo_t5 exo_t6 exo_t7 t_pip exo_t9 exo_t10
% exo_x11 t_dip
Estimates_RT = [data(:,2:3)*180/pi,data(:,4),data(:,5:13)*180/pi];

Theta_r = data(:,14:17)*180/pi;
Theta_r_dot = data(:,18:19)*180/pi;
Theta_m_u = data(:,20:21);
Theta_m = data(:,22:23);
Theta_m_dot = data(:,24:25);
Tau_finger_d = data(:,26:27);
Tau_finger_dot_d = data(:,28:29);
Tau_finger = data(:,30:31);
Tau_finger_dot = data(:,32:33);
Tau_exo_d = data(:,34:35);
Tau_exo_dot_d = data(:,36:37);
Tau_exo = data(:,38:39);
% Tau_exo(:,2) = Tau_exo(:,2);
Tau_exo_dot = data(:,40:41);

% k = 1488.57*0.5;
% rj = 0.4635*0.0254;
% rm = 1.09375*0.0254;
% Theta_r_0 = 83.44;
% Tau_exo_pip = 2*k*rj*(rm*Theta_m_u(:,2)*pi/180 - rj*(Theta_r(:,3)-Theta_r_0)*pi/180);

Theta_finger_RT(:,1) = (2*180-Estimates_RT(:,4));
Theta_finger_RT(:,2) = (2*180-Estimates_RT(:,8))-Theta_finger_RT(:,1);

try
    Jn(:,1,1) = data(:,42);
    Jn(:,1,2) = data(:,43);
    Jn(:,2,1) = data(:,44);
    Jn(:,2,2) = data(:,45);
    Jn_dot(:,1,1) = data(:,46);
    Jn_dot(:,1,2) = data(:,47);
    Jn_dot(:,2,1) = data(:,48);
    Jn_dot(:,2,2) = data(:,49);
end

JR(:,1) = abs(Tau_finger(:,1)./Estimates_RT(:,3));

l_EF = 0.035;
l_FG = 0.0290;
l_GH = 0.0290;
l_FH = sqrt(l_GH*l_GH+l_FG*l_FG);
% JR(:,2) = Tau_exo(:,2)/l_EF;
JR_pip_y = -(Tau_finger(:,1).*sin(Estimates_RT(:,1)*pi/180)+...
    Tau_exo(:,2).*sin(3*pi/2-Estimates_RT(:,7)*pi/180))./...
    (l_FH*cos(3*pi/2-Estimates_RT(:,7)*pi/180).*sin(Estimates_RT(:,6)*pi/180)+...
    l_EF*sin(3*pi/2-Estimates_RT(:,7)*pi/180).*cos(Estimates_RT(:,6)*pi/180));

JR_pip_x = (Tau_finger(:,1)+...
    JR_pip_y.*l_FH.*cos(3*pi/2-Estimates_RT(:,7)*pi/180))./...
    (l_FH*sin(3*pi/2-Estimates_RT(:,7)*pi/180));

JR(:,2) = (JR_pip_x.^2+JR_pip_y.^2).^(1/2);


% JR(:,1) = Tau_finger(:,1)./Estimates_RT(:,3)+JR(:,2);
%% Measured Geometric Parameters
% Priyanshu
% x_A = 0.008;
% y_A = -0.03;
% l_BC = 0.047;
% l_CD = 0.025;
% l_AH = 0.050;
% l_CE = 0.04;
% l_EF = 0.035;
% l_FG = 0.018;
% l_GH = 0.015;
% l_GK = 0.007;
% l_FI = 0.02617;
% l_IJ = 0.01968;

%% Optimized Geometric Parameters
% x_A = 0.0000;
% y_A = -0.0357;
%
% l_BC = 0.0424;
%
% l_CD = 0.0350;
% l_AH = 0.0400;
%
% l_CE = 0.04;
% l_EF = 0.035;
% l_FG = 0.0290;
% l_GH = 0.0290;
% l_GK = 0.007;
% l_FI = 0.02617;
% l_IJ = 0.01968;

% J_final = zeros(end_ind-start_ind+1,4,2);
% Estimates = zeros(end_ind-start_ind+1,12);
% for i=start_ind:end_ind
%     exo_t_rel = [Theta_r(i,1), Theta_r(i,3), Theta_r(i,4)]'*pi/180;
%     [tau_finger,tau_finger_dot,...
%         tau_exo,tau_exo_dot,J,J_dot,...
%         K_1,K_2,K1,K2,~,~,estimates] = index_finger_exoskeleton_statics(exo_t_rel,...
%         Theta_r_dot(i,:)',0);
%
%     %     % Position input
%     %     exo_t1 = estimates(1);
%     %     exo_x3 = estimates(3);
%     %     t_mcp = estimates(4);
%     %     exo_t5 = estimates(5);
%     %     exo_t6 = estimates(6);
%     %     exo_t7 = estimates(7);
%     %     t_pip = estimates(8);
%     %     exo_t6r = exo_t_rel(2);
%     %
%     %     l_FH = sqrt(l_GH*l_GH+l_FG*l_FG);
%     %
%     %     t_HFG = atan(l_GH/l_FG);
%     %     l_DH = l_AH-exo_x3;
%     %     l_CH = sqrt(l_CD*l_CD+l_DH*l_DH);
%     %     t_HFG = atan(l_GH/l_FG);
%     %     t_DCH = atan(l_DH/l_CD);
%     %
%     %     J = index_finger_J(exo_t1,exo_t5,exo_x3,exo_t6r,...
%     %         l_AH,l_BC,l_CD,l_CE,l_EF,l_FH,t_HFG,t_mcp,t_pip);
%     %
%     %     J_final(i,:,:) = J;
%     Estimates(i,:) = estimates*180/pi;
% end

% Theta_finger(:,1) = wrapTo180((2*180-Estimates(:,4)));
% Theta_finger(:,2) = wrapTo180((2*180-Estimates(:,8))-Theta_finger(:,1));
%% Plotting and Exporting figures
fig_array = zeros(17,1);


if(ct ~= 12)
    figure(1);
    fig_array(1) = 1;
    clf;
    h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
        Theta_r(start_ind:end_ind,1),'-b','LineWidth',line_thickness);
    hold on;
    h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
        Theta_r(start_ind:end_ind,3),'--r','LineWidth',line_thickness);
    legend(h,{'$\theta_{1r}$','$\theta_{6r}$'},'location',legend_location{1},...
        'Interpreter','latex',...
        'Box','off','EdgeColor',[1 1 1],'Color','none','Orientation',...
        'Horizontal');
    if(~print_flag)
        theTitle = ['Exoskeleton Joint Angles Vs Time'];
        title(theTitle,'Interpreter','latex',...
            'FontName','Times New Roman','FontSize',fontsize);
    end
    xlabel([ 'Time (s) $\rightarrow$'],'Interpreter','latex','FontName',...
        'Times New Roman','FontSize',fontsize);
    ylabel('Exo Joint Angles (deg) $\rightarrow$','Interpreter','latex',...
        'FontName','Times New Roman','FontSize',fontsize);
    grid on
    hold off
end

if(ct == 12)
    figure(1);
    fig_array(1) = 1;
    clear h ;
    clf;
    h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
        Theta_r(start_ind:end_ind,1),'-b','LineWidth',line_thickness);
    hold on;
    h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
        Theta_r(start_ind:end_ind,2),'-.k','LineWidth',line_thickness);
    h(3) = plot(time(start_ind:end_ind)-time(start_ind),...
        Theta_r(start_ind:end_ind,3),'--r','LineWidth',line_thickness);
    legend(h,{'$\theta_{1r}$','$\theta_{5r}$','$\theta_{6r}$'},'Interpreter',...
        'latex','location',...
        legend_location{1},'Orientation','Horizontal',...
        'Box','off','EdgeColor',[1 1 1],'Color','none');
    if(~print_flag)
        theTitle = ['Exoskeleton Joint Angles Vs Time'];
        title(theTitle,'Interpreter','latex',...
            'FontName','Times New Roman','FontSize',fontsize);
    end
    xlabel([ 'Time (s) $\rightarrow$'],'Interpreter','latex','FontName',...
        'Times New Roman','FontSize',fontsize);
    ylabel('Exo Joint Angles (deg) $\rightarrow$','Interpreter','latex',...
        'FontName','Times New Roman','FontSize',fontsize);
    grid on
    hold off
end

figure(2);
fig_array(2) = 1;
clear h ;
clf;
h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
    Theta_finger_RT(start_ind:end_ind,1),'-b','LineWidth',line_thickness);
hold on;
h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
    Theta_finger_RT(start_ind:end_ind,2),'--r','LineWidth',line_thickness);
legend(h,{'$\theta_{mcp}$','$\theta_{pip}$'},'Interpreter',...
        'latex','location',legend_location{2},...
    'Box','off','EdgeColor',[1 1 1],'Color','none','Orientation','Horizontal');
if(~print_flag)
    theTitle = ['Finger Joint Angles Real-time Vs Time'];
    title(theTitle,'Interpreter','latex',...
        'FontName','Times New Roman','FontSize',fontsize);
end
xlabel([ 'Time (s) $\rightarrow$'],'Interpreter','latex','FontName',...
    'Times New Roman','FontSize',fontsize);
ylabel('Finger Joint Angles (deg) $\rightarrow$','Interpreter','latex',...
    'FontName','Times New Roman','FontSize',fontsize);
grid on
hold off

if(ct~=5 && ct~=6 && ct~=7 && ct~=8)
    figure(3);
    fig_array(3) = 1;
    clear h ;
    clf;
    h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
        Theta_m_u(start_ind:end_ind,1),'-b','LineWidth',line_thickness);
    hold on;
    h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
        Theta_m(start_ind:end_ind,1),'--r','LineWidth',line_thickness);
    leg = legend(h,{'Desired','Measured'},'Location',legend_location{3},...
        'Box','off','EdgeColor',[1 1 1],'Color','none','Orientation',...
        'Horizontal','Interpreter','latex');
    rect = [0.3, 0.85, .55, .25];
    set(leg, 'Position', rect);
    if(~print_flag)
        theTitle = ['MCP Motor Angle Vs Time'];
        title(theTitle,'Interpreter','latex',...
            'FontName','Times New Roman','FontSize',fontsize);
    end
    xlabel([ 'Time (s) $\rightarrow$'],'Interpreter','latex','FontName',...
        'Times New Roman','FontSize',fontsize);
    ylabel('MCP Motor Angle (deg) $\rightarrow$','Interpreter','latex',...
        'FontName','Times New Roman','FontSize',fontsize);
    grid on
    hold off
end

if(ct~=1 && ct~=2 && ct~=3 && ct~=4)
    figure(4);
    fig_array(4) = 1;
    clear h ;
    clf;
    h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
        Theta_m_u(start_ind:end_ind,2),'-b','LineWidth',line_thickness);
    hold on;
    h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
        Theta_m(start_ind:end_ind,2),'--r','LineWidth',line_thickness);
    legend(h,{'Desired','Measured'},'location',legend_location{4},...
        'Box','off','EdgeColor',[1 1 1],'Color','none','Orientation',...
        'Horizontal','Interpreter','latex');
    if(~print_flag)
        theTitle = ['PIP Motor Angle Vs Time'];
        title(theTitle,'Interpreter','latex',...
            'FontName','Times New Roman','FontSize',fontsize);
    end
    xlabel([ 'Time (s) $\rightarrow$'],'Interpreter','latex','FontName',...
        'Times New Roman','FontSize',fontsize);
    ylabel('PIP Motor Angle (deg) $\rightarrow$','Interpreter','latex',...
        'FontName','Times New Roman','FontSize',fontsize);
    grid on
    hold off
end

if(ct==1 || ct==2 || ct==9 || ct==10 || ct==13)
    figure(5);
    fig_array(5) = 1;
    plot(time(start_ind:end_ind)-time(start_ind),...
        Tau_finger(start_ind:end_ind,1),'--r','LineWidth',line_thickness);
    if(~print_flag)
        theTitle = ['MCP Finger Joint Torque Vs Time'];
        title(theTitle,'Interpreter','latex',...
            'FontName','Times New Roman','FontSize',fontsize);
    end
    xlabel([ 'Time (s) $\rightarrow$'],'Interpreter','latex','FontName',...
        'Times New Roman','FontSize',fontsize);
    ylabel('MCP Joint Torque (Nm) $\rightarrow$','Interpreter','latex',...
        'FontName','Times New Roman','FontSize',fontsize);
    grid on
    hold off
    try
    ylim(ylim_tau_mcp);
    end
end

if(ct==5 || ct==6 || ct==9 || ct==10 || ct==13)
    figure(6);
    fig_array(6) = 1;
    plot(time(start_ind:end_ind)-time(start_ind),...
        Tau_finger(start_ind:end_ind,2),'--r','LineWidth',line_thickness);
    if(~print_flag)
        theTitle = ['PIP Finger Joint Torque Vs Time'];
        title(theTitle,'Interpreter','latex',...
            'FontName','Times New Roman','FontSize',fontsize);
    end
    xlabel([ 'Time (s) $\rightarrow$'],'Interpreter','latex','FontName',...
        'Times New Roman','FontSize',fontsize);
    ylabel('PIP Joint Torque (Nm) $\rightarrow$','Interpreter','latex',...
        'FontName','Times New Roman','FontSize',fontsize);
    grid on
    hold off
end


if(ct==3 || ct==4 ||ct==11 || ct==12)
    figure(5);
    fig_array(5) = 1;
    clear h ;
    clf
    h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
        Tau_finger_d(start_ind:end_ind,1),'-b','LineWidth',line_thickness);
    hold on;
    h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
        Tau_finger(start_ind:end_ind,1),'--r','LineWidth',line_thickness);
    legend(h,{'Desired','Estimated'},'location',legend_location{5},...
        'Box','off','EdgeColor',[1 1 1],'Color','none','Orientation',...
        'Horizontal','Interpreter','latex');
    if(~print_flag)
        theTitle = ['MCP Finger Joint Torque Vs Time'];
        title(theTitle,'Interpreter','latex',...
            'FontName','Times New Roman','FontSize',fontsize);
    end
    xlabel([ 'Time (s) $\rightarrow$'],'Interpreter','latex','FontName',...
        'Times New Roman','FontSize',fontsize);
    ylabel('MCP Joint Torque (Nm) $\rightarrow$','Interpreter','latex',...
        'FontName','Times New Roman','FontSize',fontsize);
    grid on
    hold off
    try
        ylim(ylim_tau_mcp);
    end
end

if(ct==7 || ct==8 || ct==11 || ct==12)
    figure(6);
    fig_array(6) = 1;
    clear h ;
    clf
    h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
        Tau_finger_d(start_ind:end_ind,2),'-b','LineWidth',line_thickness);
    hold on;
    h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
        Tau_finger(start_ind:end_ind,2),'--r','LineWidth',line_thickness);
    legend(h,{'Desired','Estimated'},'location',legend_location{6},...
        'Box','off','EdgeColor',[1 1 1],'Color','none','Orientation',...
        'Horizontal','Interpreter','latex');
    if(~print_flag)
        theTitle = ['PIP Finger Joint Torque Vs Time'];
        title(theTitle,'Interpreter','latex',...
            'FontName','Times New Roman','FontSize',fontsize);
    end
    xlabel([ 'Time (s) $\rightarrow$'],'Interpreter','latex','FontName',...
        'Times New Roman','FontSize',fontsize);
    ylabel('PIP Joint Torque (Nm) $\rightarrow$','Interpreter','latex',...
        'FontName','Times New Roman','FontSize',fontsize);
    grid on
    hold off
end

if(ct==1 || ct==2 || ct==9 || ct==10 || ct==13)
    figure(7);
    fig_array(7) = 1;
    clear h ;
    clf;
    h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
        Tau_exo_d(start_ind:end_ind,1),'-b','LineWidth',line_thickness);
    hold on;
    h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
        Tau_exo(start_ind:end_ind,1),'--r','LineWidth',line_thickness);
    legend(h,{'Desired','Estimated'},'Location',legend_location{7},...
        'Box','off','EdgeColor',[1 1 1],'Color','none','Orientation',...
        'Horizontal','Interpreter','latex');
    if(~print_flag)
        theTitle = ['Exo MCP Torque Vs Time'];
        title(theTitle,'Interpreter','latex',...
            'FontName','Times New Roman','FontSize',fontsize);
    end
    xlabel([ 'Time (s) $\rightarrow$'],'Interpreter','latex','FontName',...
        'Times New Roman','FontSize',fontsize);
    ylabel('Exo MCP Joint Torque (Nm) $\rightarrow$','Interpreter','latex',...
        'FontName','Times New Roman','FontSize',fontsize);
    % try
    %     ylim(ylimits);
    % end
    grid on
    hold off
    try
        ylim([ylimit_tau_mcp]);
    end
end

if(ct==3 || ct==4 || ct==11 || ct==12 )
    figure(7);
    fig_array(7) = 1;
    clear h ;
    clf;
    plot(time(start_ind:end_ind)-time(start_ind),...
        Tau_exo(start_ind:end_ind,1),'--r','LineWidth',line_thickness);
    if(~print_flag)
        theTitle = ['Exoskeleton MCP Joint Torque Vs Time'];
        title(theTitle,'Interpreter','latex',...
            'FontName','Times New Roman','FontSize',fontsize);
    end
    xlabel([ 'Time (s) $\rightarrow$'],'Interpreter','latex','FontName',...
        'Times New Roman','FontSize',fontsize);
    ylabel('Exo MCP Joint Torque (Nm) $\rightarrow$','Interpreter','latex',...
        'FontName','Times New Roman','FontSize',fontsize);
    grid on
    hold off
end

if(ct==5 || ct==6 || ct==9 || ct==10 || ct==13)
    figure(8);
    fig_array(8) = 1;
    clear h ;
    clf;
    h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
        Tau_exo_d(start_ind:end_ind,2),'-b','LineWidth',line_thickness);
    hold on;
    h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
        Tau_exo(start_ind:end_ind,2),'--r','LineWidth',line_thickness);
    %     h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
    %         Tau_exo_pip(start_ind:end_ind),'--r','LineWidth',line_thickness);
    legend(h,{'Desired','Estimated'},'Location',legend_location{8},...
        'Box','off','EdgeColor',[1 1 1],'Color','none','Orientation',...
        'Horizontal','Interpreter','latex');
    if(~print_flag)
        theTitle = ['Exoskeleton PIP Joint Torque Vs Time'];
        title(theTitle,'Interpreter','latex',...
            'FontName','Times New Roman','FontSize',fontsize);
    end
    xlabel([ 'Time (s) $\rightarrow$'],'Interpreter','latex','FontName',...
        'Times New Roman','FontSize',fontsize);
    ylabel('Exo PIP Joint Torque (Nm) $\rightarrow$','Interpreter','latex',...
        'FontName','Times New Roman','FontSize',fontsize);
    grid on
    hold off
    try
        ylim([ylimit_tau_pip]);
    end
end

if(ct==7 || ct==8 || ct==11 || ct==12)
    figure(8);
    fig_array(8) = 1;
    clear h ;
    clf;
    h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
        Tau_exo(start_ind:end_ind,2),'-b','LineWidth',line_thickness);
    if(~print_flag)
        theTitle = ['Exoskeleton PIP Joint Torque Vs Time'];
        title(theTitle,'Interpreter','latex',...
            'FontName','Times New Roman','FontSize',fontsize);
    end
    xlabel([ 'Time (s) $\rightarrow$'],'Interpreter','latex','FontName',...
        'Times New Roman','FontSize',fontsize);
    ylabel('Exo PIP Joint Torque (Nm) $\rightarrow$','Interpreter','latex',...
        'FontName','Times New Roman','FontSize',fontsize);
    grid on
    hold off
end

figure(9);
fig_array(9) = 1;
clear h ;
clf;
h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
    Theta_r_dot(start_ind:end_ind,1),'-b','LineWidth',line_thickness);
hold on;
h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
    Theta_r_dot(start_ind:end_ind,2),'--r','LineWidth',line_thickness);
legend(h,{'$\omega_{1r}$','$\omega_{6r}$'},'Interpreter',...
        'latex','Location',legend_location{9},...
    'Box','off','EdgeColor',[1 1 1],'Color','none','Orientation','Horizontal');
if(~print_flag)
    theTitle = ['Exoskeleton Joint Velocities Vs Time'];
    title(theTitle,'Interpreter','latex',...
        'FontName','Times New Roman','FontSize',fontsize);
end
xlabel([ 'Time (s) $\rightarrow$'],'Interpreter','latex','FontName',...
    'Times New Roman','FontSize',fontsize);
ylabel('Exo Joint Velocity (deg/s) $\rightarrow$','Interpreter','latex',...
    'FontName','Times New Roman','FontSize',fontsize);
grid on
hold off

figure(10);
fig_array(10) = 1;
clear h ;
clf;
h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
    Theta_m_dot(start_ind:end_ind,1),'-b','LineWidth',line_thickness);
hold on;
h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
    Theta_m_dot(start_ind:end_ind,2),'--r','LineWidth',line_thickness);
legend(h,{'$\omega_{m,mcp}$','$\omega_{m,pip}$'},'Interpreter',...
        'latex','location',legend_location{10},...
    'Box','off','EdgeColor',[1 1 1],'Color','none','Orientation','Horizontal');
if(~print_flag)
    theTitle = ['Motor Velocities Vs Time'];
    title(theTitle,'Interpreter','latex',...
        'FontName','Times New Roman','FontSize',fontsize);
end
xlabel([ 'Time (s) $\rightarrow$'],'Interpreter','latex','FontName',...
    'Times New Roman','FontSize',fontsize);
ylabel('Motor Velocity (rad/s) $\rightarrow$','Interpreter','latex',...
    'FontName','Times New Roman','FontSize',fontsize);
grid on
hold off

% figure(11);
% fig_array(11) = 0;
% clear h ;
% clf;
% h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
%     J_final(start_ind:end_ind,2,1),'-g','LineWidth',line_thickness);
% hold on;
% h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
%     J_final(start_ind:end_ind,2,2),'-k','LineWidth',line_thickness);
% h(3) = plot(time(start_ind:end_ind)-time(start_ind),...
%     J_final(start_ind:end_ind,4,1),'--g','LineWidth',line_thickness);
% h(4) = plot(time(start_ind:end_ind)-time(start_ind),...
%     J_final(start_ind:end_ind,4,2),'--k','LineWidth',line_thickness);
% legend(h,'J_{21}','J_{22}',...
%     'J_{41}','J_{42}','location','NorthEast');
% theTitle = ['Jacobian Entries Vs Time'];
% title(theTitle,'Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% xlabel([ 't (s) $\rightarrow$'],'Interpreter','latex','FontName',...
%     'Times New Roman','FontSize',fontsize);
% ylabel('J $\rightarrow$','Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% grid on
% hold off

figure(12);
fig_array(12) = 1;
clear h ;
clf;
h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
    Jn(start_ind:end_ind,1,1),'-b','LineWidth',line_thickness);
hold on;
h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
    Jn(start_ind:end_ind,1,2),'-r','LineWidth',line_thickness);
h(3) = plot(time(start_ind:end_ind)-time(start_ind),...
    Jn(start_ind:end_ind,2,1),'-g','LineWidth',line_thickness);
h(4) = plot(time(start_ind:end_ind)-time(start_ind),...
    Jn(start_ind:end_ind,2,2),'-k','LineWidth',line_thickness);
legend(h,{'$J_{n,11}$','$J_{n,12}$','$J_{n,21}$','$J_{n,22}$'},'Interpreter',...
        'latex','Location',legend_location{12},...
    'Box','off','EdgeColor',[1 1 1],'Color','none');
if(~print_flag)
    theTitle = ['Jacobian Real-time Entries Vs Time'];
    title(theTitle,'Interpreter','latex',...
        'FontName','Times New Roman','FontSize',fontsize);
end
xlabel([ 'Time (s) $\rightarrow$'],'Interpreter','latex','FontName',...
    'Times New Roman','FontSize',fontsize);
ylabel('$J_n$ $\rightarrow$','Interpreter','latex',...
    'FontName','Times New Roman','FontSize',fontsize);
grid on
hold off

figure(13);
fig_array(13) = 1;
clear h ;
clf;
h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
    Jn_dot(start_ind:end_ind,1,1),'-b','LineWidth',line_thickness);
hold on;
h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
    Jn_dot(start_ind:end_ind,1,2),'-r','LineWidth',line_thickness);
h(3) = plot(time(start_ind:end_ind)-time(start_ind),...
    Jn_dot(start_ind:end_ind,2,1),'-g','LineWidth',line_thickness);
h(4) = plot(time(start_ind:end_ind)-time(start_ind),...
    Jn_dot(start_ind:end_ind,2,2),'-k','LineWidth',line_thickness);
legend(h,{'$\dot{J}_{n11}$','$\dot{J}_{n,12}$','$\dot{J}_{n,21}$',...
          '$\dot{J}_{n,22}$'},'Interpreter',...
        'latex','location',legend_location{13},...
    'Box','off','EdgeColor',[1 1 1],'Color','none');
if(~print_flag)
    theTitle = ['Jacobian derivative Real-time Entries Vs Time'];
    title(theTitle,'Interpreter','latex',...
        'FontName','Times New Roman','FontSize',fontsize);
end
xlabel([ 'Time (s) $\rightarrow$'],'Interpreter','latex','FontName',...
    'Times New Roman','FontSize',fontsize);
ylabel('$\dot{J}_n$ $\rightarrow$','Interpreter','latex',...
    'FontName','Times New Roman','FontSize',fontsize);
grid on
hold off

figure(14);
fig_array(14) = 1;
clear h ;
clf;
ax1 = axes('Position',[0.1500    0.1200    0.7350    0.8050]);
for i=1:8
    if(i~=3)
        h(i) = plot(time(start_ind:end_ind)-time(start_ind),...
            Estimates_RT(start_ind:end_ind,i),pattern_str{i},'LineWidth',line_thickness);
    end
    hold on;
end

if(~print_flag)
    theTitle = ['Angle Estimates Real-time Vs Time'];
    title(theTitle,'Interpreter','latex',...
        'FontName','Times New Roman','FontSize',fontsize);
end
xlabel([ 'Time (s) $\rightarrow$'],'Interpreter','latex','FontName',...
    'Times New Roman','FontSize',fontsize);
% set(get(ax1,'YLabel'),'String',...
%     'Joint displacement (deg)','FontSize',fontsize,...
%     'FontName','Times New Roman');
ylabel('Joint displacement (deg) $\rightarrow$','Interpreter','latex',...
    'FontName','Times New Roman','FontSize',fontsize);
grid on


ax2 = axes('Position',get(ax1,'Position'),...
    'XAxisLocation','bottom',...
    'YAxisLocation','right',...
    'Color','none',...
    'XColor','k','YColor','k');%,...
% set(get(ax2,'YLabel'),'String',...
%     'Joint displacement (m)','FontSize',fontsize,...
%     'FontName','Times New Roman'); %,'YLim',[0 0.04]
ylabel(ax2,'Joint displacement (mm) $\rightarrow$','Interpreter','latex',...
    'FontName','Times New Roman','FontSize',fontsize);
h(3) = line(time(start_ind:end_ind)-time(start_ind),...
    Estimates_RT(start_ind:end_ind,3)*1000,'Color','k',...
    'LineWidth',line_thickness,'Parent',ax2);
legend(h,{'$\theta_1$','$\theta_2$','$x_3$','$\theta_4$','$\theta_5$',...
    '$\theta_6$','$\theta_7$','$\theta_8$'},'Interpreter',...
        'latex','Box','off','EdgeColor',[1 1 1],...
    'Color','none','Location',legend_location{14});

hold off
% return;


% figure(15);
% fig_array(15) = 0;
% clear h ;
% clf;
% for i=1:7
%     h(i) = plot(time(start_ind:end_ind)-time(start_ind),...
%         Estimates(start_ind:end_ind,i),pattern_str{i},'LineWidth',line_thickness);
%     hold on;
% end
% legend(h,'\theta_1','\theta_2','x_3','\theta_{MCP}','\theta_5',...
%     '\theta_6','\theta_7');
% theTitle = ['Angle Estimates Vs Time'];
% title(theTitle,'Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% xlabel([ 't (s) $\rightarrow$'],'Interpreter','latex','FontName',...
%     'Times New Roman','FontSize',fontsize);
% ylabel('$\theta$ / $x$ $\rightarrow$','Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% grid on
% hold off

figure(16);
fig_array(16) = 1;
clear h ;
clf;
h(1) = plot(Theta_finger_RT(start_ind:end_ind,1),...
    Tau_finger(start_ind:end_ind,1),pattern_str{1},'LineWidth',line_thickness);
hold on;
h(2) = plot(Theta_finger_RT(start_ind:end_ind,2),...
    Tau_finger(start_ind:end_ind,2),pattern_str{2},'LineWidth',line_thickness);
legend(h,{'$\theta_{MCP}$','$\theta_{PIP}$'},'Interpreter',...
        'latex','Location',legend_location{16},...
    'Box','off','EdgeColor',[1 1 1],'Color','none');
if(~print_flag)
    theTitle = ['Finger Torque Vs Angle'];
    title(theTitle,'Interpreter','latex',...
        'FontName','Times New Roman','FontSize',fontsize);
end
xlabel([ '$\theta$ (deg) $\rightarrow$'],'Interpreter','latex','FontName',...
    'Times New Roman','FontSize',fontsize);
ylabel('$\tau$ $\rightarrow$','Interpreter','latex',...
    'FontName','Times New Roman','FontSize',fontsize);
grid on
hold off

% MCP Joint Reaction Force
figure(17);
fig_array(17) = 1;
clear h ;
clf;
h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
    JR(start_ind:end_ind,1),pattern_str{1},'LineWidth',line_thickness);
hold on;
h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
    JR(start_ind:end_ind,2),pattern_str{2},'LineWidth',line_thickness);
legend(h,{'MCP','PIP'},'Location',legend_location{16},...
    'Box','off','EdgeColor',[1 1 1],'Color','none','Interpreter','latex');
if(~print_flag)
    theTitle = ['Finger Joint Reaction Vs Time'];
    title(theTitle,'Interpreter','latex',...
        'FontName','Times New Roman','FontSize',fontsize);
end
xlabel([ 'Time (s) $\rightarrow$'],'Interpreter','latex','FontName',...
    'Times New Roman','FontSize',fontsize);
ylabel('Joint Reaction Magnitude (N) $\rightarrow$','Interpreter','latex',...
    'FontName','Times New Roman','FontSize',fontsize);
grid on
hold off

plot_str = {'exo_rel_angles','finger_angles',...
    'motor_angle_mcp','motor_angle_pip','finger_torque_mcp',...
    'finger_torque_pip','exoskeleton_torque_mcp',...
    'exoskeleton_torque_pip','exo_rel_velocity','motor_velocity',...
    'jacobian_offline','jacobian_real_time',...
    'jacobian_derivative_real_time','angle_estimates_real_time',...
    'angle_estimates_offline','finger_torque_angle','finger_joint_reaction'};

try
    % Exporting PDF of plot
    if(print_flag)
        for i=1:length(fig_array)
            if(fig_array(i))
                figure(i);
                %                 set(gca, 'LooseInset', get(gca, 'TightInset'));
                %                 figuresize(300,200,'points')
                %                 print(gcf, '-dpdf',[plot_dir '/IFE_' control_type{ct} '_'...
                %                     plot_str{i} '_f_' num2str(floor(f))])
                export_fig([plot_dir '/IFE_' control_type{ct} '_'...
                    plot_str{i} '_f_' num2str(floor(f))],'-pdf',...
                    '-transparent');
            end
        end
    end
end

if(print_flag)
    pause;
    close all;
end