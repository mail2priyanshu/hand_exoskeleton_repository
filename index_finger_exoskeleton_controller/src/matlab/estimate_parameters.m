clc
clear

%% INITIALISATION AND PARAMETER DEFINITION:
% =========================================
addpath('../../../../../../MATLAB/export_fig');
plot_dir = 'figures'; % directory to export plot PDF
print_flag = 0; % print figure PDF
line_thickness = 2;
fontsize = 16;
pattern_str = {'-b','--r','-+k','-.c','-*y','.m','--b'};

if(print_flag)
    close all;
end

global tau_j theta_m theta_j r_m r_j start_ind end_ind offset
r_m = 1.09375*0.0254;
r_j = 0.4635*0.0254;

% test_type='../data/sensor_data_accuracy';
% f = 1;
% ct = 4;
% start_ind = 1000;
% end_ind = 4000;
%
control_type = {'FF_Exo_MCP','FF_PID_Exo_MCP','FF_MCP','FF_PID_MCP',...
    'FF_Exo_PIP','FF_PID_Exo_PIP','FF_PIP','FF_PID_PIP',...
    'FF_Exo_MCP_PIP','FF_PID_Exo_MCP_PIP','FF_MCP_PIP',...
    'FF_PID_MCP_PIP'};
% k = 680;
%
% test_type='../data/straight/chosen/sensor_data_step';
% f = 0;
% ct = 1;
% start_ind = 960;
% end_ind = 1930;
%
% switch(control_type{ct})
%     case 'open_loop'
%         filename = sprintf('%s_%s_k_%0.6f_f_%0.6f.csv',test_type,'open_loop',k,f);
%     case 'proportional'
%         kp = 3;
%         filename = sprintf('%s_%s_k_%0.6f_f_%0.6f_kp_%0.6f.csv',...
%             test_type,'proportional',k,f,kp);
%     case 'proportional_derivative'
%         kp = 3;
%         kd = 0.1;
%         filename = sprintf('%s_%s_k_%0.6f_f_%0.6f_kp_%0.6f_kd_%0.6f.csv',...
%             test_type,'proportional_derivative',k,f,kp,kd);
%     case 'proportional_derivative_integral'
%         kp = 3;
%         kd = 0.1;
%         ki = 0.1;
%         filename = sprintf('%s_%s_k_%0.6f_f_%0.6f_kp_%0.6f_kd_%0.6f_ki_%0.6f.csv',...
%             test_type,'proportional_derivative_integral',k,f,kp,kd,ki);
% end

%% File Override
% filename ='../data/gain_test/sensor_data_FF_PID_PIP_Exo_f_0.500000_tau_pip_0.060000_kp_11_0.csv';
filename ='../data/sensor_data_FF_PID_MCP_PIP_Exo_f_0.500000_tau_mcp_0.250000_tau_pip_0.040000_0.csv';
end_ind = 3000;
ct = 10;

% filename ='../data/sensor_data_FF_PID_MCP_f_0.500000_tau_mcp_0.200000_tau_pip_0.000000_0.csv';
% ct = 3;

% filename ='../data/sensor_data_FF_PID_PIP_f_0.500000_tau_mcp_0.000000_tau_pip_0.010000_0.csv';
% end_ind = 3000;
% ct = 7;

% filename ='../data/sensor_data_FF_PID_PIP_f_0.500000_tau_mcp_0.000000_tau_pip_0.025000_0.csv';
% end_ind = 5000;
% ct = 7;

% offset = 30;
f = 0.5;
% start_ind = 1;
% end_ind = 195;
% start_ind_long = 1;
% end_ind_long = 1000;
print_flag = 0;


% %% Torque Control
% filename ='../data/20140424/sensor_data_FF_PID_PIP_Exo_f_0.500000_tau_pip_0.070000.csv';
%
% ct = 5;
% % offset = 30;
% f = 0.5;
% start_ind = 1;
% end_ind = 1000;
% start_ind_long = 1;
% end_ind_long = 1000;
% print_flag = 0;


% %% Dynamic Transparency
%
% filename ='../data/20140424/sensor_data_FF_PIP_Exo_f_4.000000_tau_pip_0.000000.csv';
%
% ct = 5;
% % offset = 30;
% f = 0.5;
% start_ind = 3500;
% end_ind = 4700;
% start_ind_long = 1;
% end_ind_long = 1000;
% print_flag = 0;


%% Control Comparison 1 (Data)
% offset = 19;
% f = 0.5;
% start_ind_long = 1;
% end_ind_long = 10000;
% plot_dir = 'figures/control_comparison'; % directory to export plot PDF

% Open-loop
% filename ='../data/control_comparison/sensor_data_accuracy_open_loop_k_771.435000_f_0.500000.csv';
% start_ind = 960;
% end_ind = 1930;
% ct = 1;
% % accuracy_err_best = 0.0197
% % accuracy_err_actual = 0.0360
% % accuracy_err_est = 0.0197

% P Control
% filename ='../data/control_comparison/sensor_data_accuracy_proportional_k_771.435000_f_0.500000_kp_3.000000.csv';
% start_ind = 960;
% end_ind = 1930;
% ct = 2;
% % accuracy_err_best = 0.0177
% % accuracy_err_actual = 0.0196
% % accuracy_err_est = 0.0195

% % PD Control
% filename ='../data/control_comparison/sensor_data_accuracy_proportional_derivative_k_771.435000_f_0.500000_kp_3.000000_kd_0.100000.csv';
% start_ind = 960;
% end_ind = 1930;
% ct = 3;
% % accuracy_err_best = 0.0172
% % accuracy_err_actual = 0.0136
% % accuracy_err_est = 0.0224

% % PID Control
% filename ='../data/control_comparison/sensor_data_accuracy_proportional_derivative_integral_k_771.435000_f_0.500000_kp_3.000000_kd_0.100000_ki_0.010000.csv';
% start_ind = 960;
% end_ind = 1930;
% ct = 4;
% % accuracy_err_best = 0.0161
% % accuracy_err_actual = 0.0217
% % accuracy_err_est = 0.0247

%% Metric Evaluation
if(print_flag)
    diary([filename(1:end-4) '.txt']);
end

data = dlmread(filename);


% end_ind = size(data,1);

% k = 807; % 8.5*0.4535*9.81/(0.0254*0.919 (factor due to enclosures for compression spring))
% tau_offset = 0.01;

% Accomodating difference in file format.
time = data(:,1); % time in stored s

Estimates_RT = data(:,2:13)*180/pi; % exo_t1 exo_t2 exo_x3 t_mcp exo_t5 exo_t6 exo_t7
% t_pip exo_t9 exo_t10 exo_x11 t_dip
Theta_r = data(:,14:17)*180/pi;
Theta_r_dot = data(:,18:19);
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
Tau_exo_dot = data(:,40:41);

Jn(:,1,1) = data(:,42);
Jn(:,1,2) = data(:,43);
Jn(:,2,1) = data(:,44);
Jn(:,2,2) = data(:,45);
Jn_dot(:,1,1) = data(:,46);
Jn_dot(:,1,2) = data(:,47);
Jn_dot(:,2,1) = data(:,48);
Jn_dot(:,2,2) = data(:,49);

Theta_finger_RT(:,1) = (2*180-Estimates_RT(:,4));
Theta_finger_RT(:,2) = (2*180-Estimates_RT(:,8))-Theta_finger_RT(:,1);

% System Parameters
global rj rm;
rj = 0.4635*0.0254; %24.8*0.001/2; % m
rm = 1.09375*0.0254;%8.20*0.001/2; % m

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

x_A = 0.008;
y_A = -0.03;

l_BC = 0.047;

l_CD = 0.025;
l_AH = 0.047;

l_CE = 0.04;
l_EF = 0.035;
l_FG = 0.018;
l_GH = 0.015;
l_GK = 0.007;
l_FI = 0.02617;
l_IJ = 0.01968;
x0 = [x_A, y_A, l_BC, l_CD, l_AH, l_FG, l_GH];

opts = optimset('Algorithm','interior-point');
problem = createOptimProblem('fmincon','x0',x0,...
    'objective',@(x) evaluate_objective(x,Theta_r),'lb',...
    [0;-0.05;0.04;0.02;0.04;0.01;0.01],'ub',...
    [0.02;-0.01;0.06;0.035;0.06;0.03;0.03],...
    'options',opts);
gs = GlobalSearch;
[x,f] = run(gs,problem)


% %% Export_fig settings
% figure(1);
% clf;
% h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
%     Theta_r(start_ind:end_ind,1),'-b','LineWidth',line_thickness);
% hold on;
% h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
%     Theta_r(start_ind:end_ind,3),'--r','LineWidth',line_thickness);
% legend(h,'\theta_{1r}','\theta_{6r}','location','NorthEast');
% theTitle = ['Exoskeleton Joint Angles Vs Time'];
% title(theTitle,'Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% xlabel([ 't (s) $\rightarrow$'],'Interpreter','latex','FontName',...
%     'Times New Roman','FontSize',fontsize);
% ylabel('Exoskeleton Joint Angles (deg) $\rightarrow$','Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% grid on
% hold off
% 
% figure(2);
% clf;
% h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
%     Theta_finger(start_ind:end_ind,1),'-b','LineWidth',line_thickness);
% hold on;
% h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
%     Theta_finger(start_ind:end_ind,2),'--r','LineWidth',line_thickness);
% legend(h,'\theta_{MCP}','\theta_{PIP}','location','NorthEast');
% theTitle = ['Finger Joint Angles Vs Time'];
% title(theTitle,'Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% xlabel([ 't (s) $\rightarrow$'],'Interpreter','latex','FontName',...
%     'Times New Roman','FontSize',fontsize);
% ylabel('Finger Joint Angles (deg) $\rightarrow$','Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% grid on
% hold off
% 
% if(ct~=5 && ct~=6 && ct~=7 && ct~=8)
%     figure(3);
%     clf;
%     h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
%         Theta_m_u(start_ind:end_ind,1),'-b','LineWidth',line_thickness);
%     hold on;
%     h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
%         Theta_m(start_ind:end_ind,1),'--r','LineWidth',line_thickness);
%     legend(h,'\theta_{m,MCP,d}','\theta_{m,MCP}','location','NorthEast');
%     theTitle = ['MCP Motor Angle Vs Time'];
%     title(theTitle,'Interpreter','latex',...
%         'FontName','Times New Roman','FontSize',fontsize);
%     xlabel([ 't (s) $\rightarrow$'],'Interpreter','latex','FontName',...
%         'Times New Roman','FontSize',fontsize);
%     ylabel('Motor Angle (deg) $\rightarrow$','Interpreter','latex',...
%         'FontName','Times New Roman','FontSize',fontsize);
%     grid on
%     hold off
% end
% 
% if(ct~=1 && ct~=2 && ct~=3 && ct~=4)
%     figure(4);
%     clf;
%     h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
%         Theta_m_u(start_ind:end_ind,2),'-b','LineWidth',line_thickness);
%     hold on;
%     h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
%         Theta_m(start_ind:end_ind,2),'--r','LineWidth',line_thickness);
%     legend(h,'\theta_{m,pip,d}','\theta_{m,pip}','location','NorthEast');
%     theTitle = ['PIP Motor Angle Vs Time'];
%     title(theTitle,'Interpreter','latex',...
%         'FontName','Times New Roman','FontSize',fontsize);
%     xlabel([ 't (s) $\rightarrow$'],'Interpreter','latex','FontName',...
%         'Times New Roman','FontSize',fontsize);
%     ylabel('Motor Angle (deg) $\rightarrow$','Interpreter','latex',...
%         'FontName','Times New Roman','FontSize',fontsize);
%     grid on
%     hold off
% end
% 
% if(ct==1 || ct==2 || ct==9 || ct==10)
%     figure(5);
%     plot(time(start_ind:end_ind)-time(start_ind),...
%         Tau_finger(start_ind:end_ind,1),'--r','LineWidth',line_thickness);
%     theTitle = ['MCP Finger Joint Torque Vs Time'];
%     title(theTitle,'Interpreter','latex',...
%         'FontName','Times New Roman','FontSize',fontsize);
%     xlabel([ 't (s) $\rightarrow$'],'Interpreter','latex','FontName',...
%         'Times New Roman','FontSize',fontsize);
%     ylabel('$\tau_{MCP}$ (Nm) $\rightarrow$','Interpreter','latex',...
%         'FontName','Times New Roman','FontSize',fontsize);
%     grid on
%     hold off
% end
% 
% if(ct==5 || ct==6 || ct==9 || ct==10)
%     figure(6);
%     plot(time(start_ind:end_ind)-time(start_ind),...
%         Tau_finger(start_ind:end_ind,2),'--r','LineWidth',line_thickness);
%     theTitle = ['PIP Finger Joint Torque Vs Time'];
%     title(theTitle,'Interpreter','latex',...
%         'FontName','Times New Roman','FontSize',fontsize);
%     xlabel([ 't (s) $\rightarrow$'],'Interpreter','latex','FontName',...
%         'Times New Roman','FontSize',fontsize);
%     ylabel('$\tau_{PIP}$ (Nm) $\rightarrow$','Interpreter','latex',...
%         'FontName','Times New Roman','FontSize',fontsize);
%     grid on
%     hold off
% end
% 
% 
% if(ct==3 || ct==4 ||ct==11 || ct==12)
%     figure(5);
%     clf
%     h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
%         Tau_finger_d(start_ind:end_ind,1),'-b','LineWidth',line_thickness);
%     hold on;
%     h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
%         Tau_finger(start_ind:end_ind,1),'--r','LineWidth',line_thickness);
%     legend(h,'Desired','Estimated','location','NorthEast');
%     theTitle = ['MCP Finger Joint Torque Vs Time'];
%     title(theTitle,'Interpreter','latex',...
%         'FontName','Times New Roman','FontSize',fontsize);
%     xlabel([ 't (s) $\rightarrow$'],'Interpreter','latex','FontName',...
%         'Times New Roman','FontSize',fontsize);
%     ylabel('MCP Joint Torque (Nm) $\rightarrow$','Interpreter','latex',...
%         'FontName','Times New Roman','FontSize',fontsize);
%     grid on
%     hold off
% end
% 
% if(ct==7 || ct==8 || ct==11 || ct==12)
%     figure(6);
%     clf
%     h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
%         Tau_finger_d(start_ind:end_ind,2),'-b','LineWidth',line_thickness);
%     hold on;
%     h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
%         Tau_finger(start_ind:end_ind,2),'--r','LineWidth',line_thickness);
%     legend(h,'Desired','Estimated','location','NorthEast');
%     theTitle = ['PIP Finger Joint Torque Vs Time'];
%     title(theTitle,'Interpreter','latex',...
%         'FontName','Times New Roman','FontSize',fontsize);
%     xlabel([ 't (s) $\rightarrow$'],'Interpreter','latex','FontName',...
%         'Times New Roman','FontSize',fontsize);
%     ylabel('PIP Joint Torque (Nm) $\rightarrow$','Interpreter','latex',...
%         'FontName','Times New Roman','FontSize',fontsize);
%     grid on
%     hold off
% end
% 
% if(ct==1 || ct==2 || ct==9 || ct==10)
%     figure(7);
%     clf;
%     h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
%         Tau_exo_d(start_ind:end_ind,1),'-b','LineWidth',line_thickness);
%     hold on;
%     h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
%         Tau_exo(start_ind:end_ind,1),'--r','LineWidth',line_thickness);
%     legend(h,'Desired','Estimated','location','NorthEast');
%     theTitle = ['Exoskeleton MCP Joint Torque Vs Time'];
%     fontsize = 16;
%     title(theTitle,'Interpreter','latex',...
%         'FontName','Times New Roman','FontSize',fontsize);
%     xlabel([ 't (s) $\rightarrow$'],'Interpreter','latex','FontName',...
%         'Times New Roman','FontSize',fontsize);
%     ylabel('Exoskeleton MCP Joint Torque (Nm) $\rightarrow$','Interpreter','latex',...
%         'FontName','Times New Roman','FontSize',fontsize);
%     % try
%     %     ylim(ylimits);
%     % end
%     grid on
%     hold off
% end
% 
% if(ct==3 || ct==4 || ct==11 || ct==12)
%     figure(7);
%     clf;
%     plot(time(start_ind:end_ind)-time(start_ind),...
%         Tau_exo(start_ind:end_ind,1),'--r','LineWidth',line_thickness);
%     theTitle = ['Exoskeleton MCP Joint Torque Vs Time'];
%     fontsize = 16;
%     title(theTitle,'Interpreter','latex',...
%         'FontName','Times New Roman','FontSize',fontsize);
%     xlabel([ 't (s) $\rightarrow$'],'Interpreter','latex','FontName',...
%         'Times New Roman','FontSize',fontsize);
%     ylabel('Exoskeleton MCP Joint Torque (Nm) $\rightarrow$','Interpreter','latex',...
%         'FontName','Times New Roman','FontSize',fontsize);
%     % try
%     %     ylim(ylimits);
%     % end
%     grid on
%     hold off
% end
% 
% if(ct==5 || ct==6 || ct==9 || ct==10)
%     figure(8);
%     clf;
%     h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
%         Tau_exo_d(start_ind:end_ind,2),'-b','LineWidth',line_thickness);
%     hold on;
%     h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
%         Tau_exo(start_ind:end_ind,2),'--r','LineWidth',line_thickness);
%     legend(h,'Desired','Estimated','location','NorthEast');
%     theTitle = ['Exoskeleton PIP Joint Torque Vs Time'];
%     fontsize = 16;
%     title(theTitle,'Interpreter','latex',...
%         'FontName','Times New Roman','FontSize',fontsize);
%     xlabel([ 't (s) $\rightarrow$'],'Interpreter','latex','FontName',...
%         'Times New Roman','FontSize',fontsize);
%     ylabel('$\tau_{PIP,exo}$ (Nm) $\rightarrow$','Interpreter','latex',...
%         'FontName','Times New Roman','FontSize',fontsize);
%     % try
%     %     ylim(ylimits);
%     % end
%     grid on
%     hold off
% end
% 
% if(ct==7 || ct==8 || ct==11 || ct==12)
%     figure(8);
%     clf;
%     h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
%         Tau_exo_d(start_ind:end_ind,2),'-b','LineWidth',line_thickness);
%     hold on;
%     h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
%         Tau_exo(start_ind:end_ind,2),'--r','LineWidth',line_thickness);
%     legend(h,'Desired','Estimated','location','NorthEast');
%     theTitle = ['Exoskeleton PIP Joint Torque Vs Time'];
%     fontsize = 16;
%     title(theTitle,'Interpreter','latex',...
%         'FontName','Times New Roman','FontSize',fontsize);
%     xlabel([ 't (s) $\rightarrow$'],'Interpreter','latex','FontName',...
%         'Times New Roman','FontSize',fontsize);
%     ylabel('$\tau_{PIP,exo}$ (Nm) $\rightarrow$','Interpreter','latex',...
%         'FontName','Times New Roman','FontSize',fontsize);
%     % try
%     %     ylim(ylimits);
%     % end
%     grid on
%     hold off
% end
% 
% figure(9);
% clf;
% h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
%     Theta_r_dot(start_ind:end_ind,1),'-b','LineWidth',line_thickness);
% hold on;
% h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
%     Theta_r_dot(start_ind:end_ind,2),'--r','LineWidth',line_thickness);
% legend(h,'\theta_{1r}','\theta_{6r}','location','NorthEast');
% theTitle = ['Exoskeleton Joint Velocities Vs Time'];
% fontsize = 16;
% title(theTitle,'Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% xlabel([ 't (s) $\rightarrow$'],'Interpreter','latex','FontName',...
%     'Times New Roman','FontSize',fontsize);
% ylabel('Joint Velocity (rad/s) $\rightarrow$','Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% % try
% %     ylim(ylimits);
% % end
% grid on
% hold off
% 
% figure(10);
% clf;
% h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
%     Theta_m_dot(start_ind:end_ind,1),'-b','LineWidth',line_thickness);
% hold on;
% h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
%     Theta_m_dot(start_ind:end_ind,2),'--r','LineWidth',line_thickness);
% legend(h,'\theta_{m,MCP}','\theta_{m,PIP}','location','NorthEast');
% theTitle = ['Motor Velocities Vs Time'];
% fontsize = 16;
% title(theTitle,'Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% xlabel([ 't (s) $\rightarrow$'],'Interpreter','latex','FontName',...
%     'Times New Roman','FontSize',fontsize);
% ylabel('Motor Velocity (rad/s) $\rightarrow$','Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% % try
% %     ylim(ylimits);
% % end
% grid on
% hold off
% 
% figure(11);
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
% fontsize = 16;
% title(theTitle,'Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% xlabel([ 't (s) $\rightarrow$'],'Interpreter','latex','FontName',...
%     'Times New Roman','FontSize',fontsize);
% ylabel('J $\rightarrow$','Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% grid on
% hold off
% 
% 
% figure(12);
% clf;
% h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
%     Jn(start_ind:end_ind,1,1),'-b','LineWidth',line_thickness);
% hold on;
% h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
%     Jn(start_ind:end_ind,1,2),'-r','LineWidth',line_thickness);
% h(3) = plot(time(start_ind:end_ind)-time(start_ind),...
%     Jn(start_ind:end_ind,2,1),'-g','LineWidth',line_thickness);
% h(4) = plot(time(start_ind:end_ind)-time(start_ind),...
%     Jn(start_ind:end_ind,2,2),'-k','LineWidth',line_thickness);
% legend(h,'J_{11}','J_{12}','J_{21}','J_{22}','Location','NorthEast');
% theTitle = ['Jacobian Real-time Entries Vs Time'];
% fontsize = 16;
% title(theTitle,'Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% xlabel([ 't (s) $\rightarrow$'],'Interpreter','latex','FontName',...
%     'Times New Roman','FontSize',fontsize);
% ylabel('J $\rightarrow$','Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% grid on
% hold off
% 
% figure(13);
% clf;
% h(1) = plot(time(start_ind:end_ind)-time(start_ind),...
%     Jn_dot(start_ind:end_ind,1,1),'-b','LineWidth',line_thickness);
% hold on;
% h(2) = plot(time(start_ind:end_ind)-time(start_ind),...
%     Jn_dot(start_ind:end_ind,1,2),'-r','LineWidth',line_thickness);
% h(3) = plot(time(start_ind:end_ind)-time(start_ind),...
%     Jn_dot(start_ind:end_ind,2,1),'-g','LineWidth',line_thickness);
% h(4) = plot(time(start_ind:end_ind)-time(start_ind),...
%     Jn_dot(start_ind:end_ind,2,2),'-k','LineWidth',line_thickness);
% legend(h,'Jn_{11}','Jn_{12}','Jn_{21}','Jn_{22}',...
%     'location','NorthEast');
% theTitle = ['Jacobian derivative Real-time Entries Vs Time'];
% fontsize = 16;
% title(theTitle,'Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% xlabel([ 't (s) $\rightarrow$'],'Interpreter','latex','FontName',...
%     'Times New Roman','FontSize',fontsize);
% ylabel('$\dot{J}$ $\rightarrow$','Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% grid on
% hold off
% 
% figure(14);
% clf;
% for i=1:7
%     h(i) = plot(time(start_ind:end_ind)-time(start_ind),...
%         Estimates_RT(start_ind:end_ind,i),pattern_str{i},'LineWidth',line_thickness);
%     hold on;
% end
% legend(h,{'\theta_1','\theta_2','x_3','\theta_{MCP}','\theta_5',...
%     '\theta_6','\theta_7'});
% theTitle = ['Angle Estimates Real-time Vs Time'];
% fontsize = 16;
% title(theTitle,'Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% xlabel([ 't (s) $\rightarrow$'],'Interpreter','latex','FontName',...
%     'Times New Roman','FontSize',fontsize);
% ylabel('$\theta$ / x $\rightarrow$','Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% grid on
% hold off
% 
% 
% figure(15);
% clf;
% for i=1:7
%     h(i) = plot(time(start_ind:end_ind)-time(start_ind),...
%         Estimates(start_ind:end_ind,i),pattern_str{i},'LineWidth',line_thickness);
%     hold on;
% end
% legend(h,'\theta_1','\theta_2','x_3','\theta_{MCP}','\theta_5',...
%     '\theta_6','\theta_7');
% theTitle = ['Angle Estimates Vs Time'];
% fontsize = 16;
% title(theTitle,'Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% xlabel([ 't (s) $\rightarrow$'],'Interpreter','latex','FontName',...
%     'Times New Roman','FontSize',fontsize);
% ylabel('$\theta$ / $x$ $\rightarrow$','Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% grid on
% hold off
% 
% 
% figure(16);
% clf;
% h(1) = plot(Theta_finger(start_ind:end_ind,1),...
%     Tau_finger(start_ind:end_ind,1),pattern_str{1},'LineWidth',line_thickness);
% hold on;
% h(2) = plot(Theta_finger(start_ind:end_ind,2),...
%     Tau_finger(start_ind:end_ind,2),pattern_str{2},'LineWidth',line_thickness);
% legend(h,'\theta_{MCP}','\theta_{PIP}');
% theTitle = ['Finger Torque Vs Angle'];
% fontsize = 16;
% title(theTitle,'Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% xlabel([ '$\theta$ (deg) $\rightarrow$'],'Interpreter','latex','FontName',...
%     'Times New Roman','FontSize',fontsize);
% ylabel('$\tau$ $\rightarrow$','Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% grid on
% hold off
% 
% %
% % start_ind = 1000;
% % end_ind = 10000;
% % if(length(theta_m)>end_ind)
% %     figure(6);
% %     clf;
% %     plot((r_m*theta_m(start_ind:end_ind)-r_j*theta_j(start_ind:end_ind))*pi/180,tau_j(start_ind:end_ind),'-r','LineWidth',line_thickness);
% %     theTitle = ['Joint Torque Vs Motor Angle (k=' num2str(k) ' N/m)'];
% %     fontsize = 16;
% %     title(theTitle,'Interpreter','latex',...
% %         'FontName','Times New Roman','FontSize',fontsize);
% %     xlabel([ '$\Delta l$ (m) $\rightarrow$'],'Interpreter','latex','FontName',...
% %         'Times New Roman','FontSize',fontsize);
% %     ylabel('$\tau_j$ (Nm) $\rightarrow$','Interpreter','latex',...
% %         'FontName','Times New Roman','FontSize',fontsize);
% %     grid on
% %     hold off
% % end
% %
% % figure(7);
% % clf;
% % plot(time(start_ind:end_ind)-time(start_ind),kj(start_ind:end_ind),'-r','LineWidth',line_thickness);
% % theTitle = ['Joint Stiffness Vs Time (k=' num2str(k) ' N/m)'];
% % fontsize = 16;
% % title(theTitle,'Interpreter','latex',...
% %     'FontName','Times New Roman','FontSize',fontsize);
% % xlabel([ 't (s) $\rightarrow$'],'Interpreter','latex','FontName',...
% %     'Times New Roman','FontSize',fontsize);
% % ylabel('$k_j$ (Nm/degrees) $\rightarrow$','Interpreter','latex',...
% %     'FontName','Times New Roman','FontSize',fontsize);
% % grid on
% % hold off
% %
% % try
% %     plot_str = {'joint_torqe','joint_torque_error','joint_torque_zoomed',...
% %         'motor_angle','joint_torque_motor_angle','joint_angle'};
% %     % Exporting PDF of plot
% %     if(print_flag)
% %         for i=1:6
% %             figure(i);
% %             export_fig([plot_dir '/sea_accuracy_best_fit_' plot_str{i} ...
% %                 '_k_' num2str(floor(k)) '_f_' num2str(floor(f)) '_' ...
% %                 control_type{ct}],'-pdf',...
% %                 '-transparent');
% %         end
% %     end
% % end
% %
% % if(print_flag)
% %     diary off;
% % %     pause;
% %     close all;
% % end