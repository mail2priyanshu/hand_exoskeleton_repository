function [tau_finger,tau_finger_dot,...
    tau_exo,tau_exo_dot,J,J_dot,...
    K_1,K_2,K1,K2,Jn,Jn_dot,estimates] = index_finger_exoskeleton_statics(exo_t_rel,...
    exo_t_dot,mechanism_plot_flag)
% Kinematics and Statics model for four-bar index finger exoskeleton
% PURPOSE : To simulate the kinematics and statics of exoskeleton
% INPUTS  :
% OUTPUTS : Exoskeleton kinematics and statics
% AUTHOR  : Priyanshu Agarwal
% DATE     : August 29th, 2013
% Last Modified: April 10th, 2014
%
% To DOs
% 1.

% DONE

% global tau_mcp tau_pip;
% clear all

%% INITIALISATION AND PARAMETER DEFINITION:
% % =========================================
% % User-defined Parameters
% print_flag = 0; % all plots will be printed as pdf if 1
% addpath('../../../export_fig');
%
% % Directory to print the pdf files
% print_dir = 'figures';
% mechanism_plot_flag = 1; % simulate mechanism using plot
% fontsize = 18;

global rj rm;

% Geometric Parameters
% Lengths
% pixeltomm = 58.65/(130.32*1000);
% x_A = pixeltomm*6.15;%0.00378 % m
% 
% y_A = -pixeltomm*73.96;%-0.03364;%+0.005; % m
% 
% l_AB = norm([x_A y_A]); % m
% 
% l_BC = pixeltomm*110.67;%0.047;%+0.002; % m
% 
% l_CD = pixeltomm*50.89;%0.02283;%(0.03247-0.01983)+0.01335/2;%-0.002; % m
% l_AH = pixeltomm*95.32;%0.044;
% 
% l_CE = pixeltomm*90.6;%0.03886;
% l_EF = pixeltomm*86.33;%0.03554;
% l_FG = pixeltomm*47.05;%0.01960;
% l_GH = pixeltomm*24.7;%0.01029;
% l_GK = pixeltomm*22.64;%0.019-l_GH; % l_GK = 0.01985-l_GH;
% l_FI = pixeltomm*60.07;%pixeltomm*57.07 %0.02443;
% l_IJ = pixeltomm*35.5;%pixeltomm*46.23 %0.018; % 0.0195
% l_KJ = 0.01;

% Priyanshu
x_A = 0.008;
y_A = -0.03;
l_BC = 0.047;
l_CD = 0.025;
l_AH = 0.053;
l_CE = 0.04;
l_EF = 0.035;
l_FG = 0.018;
l_GH = 0.015;
l_GK = 0.007;
l_FI = 0.02617;
l_IJ = 0.01968;


%% System states
exo_t1_rel = exo_t_rel(1);
exo_t6_rel = exo_t_rel(2);
exo_t9_rel = exo_t_rel(3);

U = [exo_t1_rel, exo_t6_rel, exo_t9_rel];

P = [x_A, y_A, l_BC, l_CD, l_AH, l_CE, l_EF, l_FG, l_GH, l_GK, l_FI, l_IJ];

estimates = kinematics_exact_solution(U,P);

exo_t1 = estimates(1);
exo_t2 = estimates(2);
exo_x3 = estimates(3);
t_mcp = estimates(4);
exo_t5 = estimates(5);
exo_t6 = estimates(6);
exo_t7 = estimates(7);
t_pip = estimates(8);
exo_t9 = estimates(9);
exo_t10 = estimates(10);
exo_x11 = estimates(11);
t_dip = estimates(12);

[tau_finger,tau_finger_dot,tau_exo,tau_exo_dot,...
    J,J_dot,K_1,K_2,K1,K2,Jn,Jn_dot] = statics_solution(estimates,P,exo_t_dot);

if(mechanism_plot_flag)
    h = figure(20);
    clf;
    set(h,'Color','black');
    
    plot_index_finger_exo_2d(P,estimates,rj)

% %     rectangle('Position',[-0.1-rm,-rm,2*rm,2*rm],...
% %         'Curvature',[1,1],...
% %         'LineWidth',2,'LineStyle','--');
% %     hold on;
%     %     plot(-0.1,0,-0.1+rm*cos(theta_m(1)),rm*sin(theta_m(1)),'-b');
%     
%     rectangle('Position',[-rj,-rj,2*rj,2*rj],...
%         'Curvature',[1,1],...
%         'LineWidth',2,'LineStyle','--');
%     hold on;
%     % Plotting MCP Chain
%     plot([0 real(l_BC*exp(i*exo_t1)),...
%         real(l_BC*exp(i*exo_t1)+l_CD*exp(i*exo_t2)),...
%         real(l_BC*exp(i*exo_t1)+l_CD*exp(i*exo_t2)+exo_x3*exp(i*(t_mcp-pi)))],...
%         [0 imag(l_BC*exp(i*exo_t1)),...
%         imag(l_BC*exp(i*exo_t1)+l_CD*exp(i*exo_t2)),...
%         imag(l_BC*exp(i*exo_t1)+l_CD*exp(i*exo_t2)+exo_x3*exp(i*(t_mcp-pi)))]);
%     
%     % Plotting PIP Chain
%     plot([l_BC*cos(exo_t1),...
%         l_BC*cos(exo_t1)+l_CE*cos(exo_t5),...
%         l_BC*cos(exo_t1)+l_CE*cos(exo_t5)+l_EF*cos(exo_t6),...
%         l_BC*cos(exo_t1)+l_CE*cos(exo_t5)+l_EF*cos(exo_t6)+l_FG*cos(exo_t7),...
%         l_BC*cos(exo_t1)+l_CE*cos(exo_t5)+l_EF*cos(exo_t6)+l_FG*cos(exo_t7)-l_GH*cos(t_pip),...
%         l_BC*cos(exo_t1)+l_CE*cos(exo_t5)+l_EF*cos(exo_t6)+...
%         l_FG*cos(exo_t7)-l_GH*cos(t_pip)-(l_AH-exo_x3)*cos(t_mcp)],...
%         [l_BC*sin(exo_t1),...
%         l_BC*sin(exo_t1)+l_CE*sin(exo_t5),...
%         l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+l_EF*sin(exo_t6),...
%         l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+l_EF*sin(exo_t6)+l_FG*sin(exo_t7),...
%         l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+l_EF*sin(exo_t6)+l_FG*sin(exo_t7)-l_GH*sin(t_pip),...
%         l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+l_EF*sin(exo_t6)+...
%         l_FG*sin(exo_t7)-l_GH*sin(t_pip)-(l_AH-exo_x3)*sin(t_mcp)]);
%     
%     % Plotting DIP Chain
%     plot([l_BC*cos(exo_t1)+l_CE*cos(exo_t5)+l_EF*cos(exo_t6)+l_FG*cos(exo_t7),...
%         l_BC*cos(exo_t1)+l_CE*cos(exo_t5)+l_EF*cos(exo_t6)+l_FG*cos(exo_t7)+l_GK*cos(t_pip),...
%         l_BC*cos(exo_t1)+l_CE*cos(exo_t5)+l_EF*cos(exo_t6)+l_FG*cos(exo_t7)+l_GK*cos(t_pip)+l_KJ*cos(t_dip)],...
%         [l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+l_EF*sin(exo_t6)+l_FG*sin(exo_t7),...
%         l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+l_EF*sin(exo_t6)+l_FG*sin(exo_t7)+l_GK*sin(t_pip),...
%         l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+l_EF*sin(exo_t6)+l_FG*sin(exo_t7)+l_GK*sin(t_pip)+l_KJ*sin(t_dip)]);
%     
%     axis([-0.1-rm 0.14 -0.1 0.055]);
%     hold on;
%     ht = text(-0.001,-0.001,'B','FontSize',18);
%     set(ht,'FontName','Times New Roman');
%     text((real(l_BC*exp(i*exo_t1))+0.001),imag(l_BC*exp(i*exo_t1))+0.001,...
%         'C','FontName','Times New Roman','FontSize',18);
%     text((real(l_BC*exp(i*exo_t1)+l_CD*exp(i*exo_t2))-0.002),...
%         imag(l_BC*exp(i*exo_t1)+l_CD*exp(i*exo_t2))+0.001,...
%         'D','FontName','Times New Roman','FontSize',18);
%     text((real(l_BC*exp(i*exo_t1)+l_CD*exp(i*exo_t2)+exo_x3*exp(i*(t_mcp-pi)))-0.002),...
%         imag(l_BC*exp(i*exo_t1)+l_CD*exp(i*exo_t2)+exo_x3*exp(i*(t_mcp-pi)))+0.001,...
%         'A','FontName','Times New Roman','FontSize',18);
%     
%     text((l_BC*cos(exo_t1)+l_CE*cos(exo_t5)-0.002),...
%         l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+0.001,...
%         'E','FontName','Times New Roman','FontSize',18);
%     
%     text((l_BC*cos(exo_t1)+l_CE*cos(exo_t5)+l_EF*cos(exo_t6)-0.002),...
%         l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+l_EF*sin(exo_t6)+0.001,...
%         'F','FontName','Times New Roman','FontSize',18);
%     
%     text((l_BC*cos(exo_t1)+l_CE*cos(exo_t5)+l_EF*cos(exo_t6)+l_FG*cos(exo_t7)-0.002),...
%         l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+l_EF*sin(exo_t6)+l_FG*sin(exo_t7)+0.001,...
%         'G','FontName','Times New Roman','FontSize',18);
%     
%     text((l_BC*cos(exo_t1)+l_CE*cos(exo_t5)+l_EF*cos(exo_t6)+l_FG*cos(exo_t7)-l_GH*cos(t_pip)-0.002),...
%         l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+l_EF*sin(exo_t6)+l_FG*sin(exo_t7)-l_GH*sin(t_pip)+0.001,...
%         'H','FontName','Times New Roman','FontSize',18);
%     
% %     text(l_BC*cos(exo_t1)+l_CE*cos(exo_t5)+l_EF*cos(exo_t6)+l_FI*cos(exo_t9)-0.002,...
% %         l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+l_EF*sin(exo_t6)+l_FI*sin(exo_t9)+0.001,...
% %         'I','FontName','Times New Roman','FontSize',18);
%     
%     text((l_BC*cos(exo_t1)+l_CE*cos(exo_t5)+l_EF*cos(exo_t6)+...
%         l_FG*cos(exo_t7)+l_GK*cos(t_pip)-0.001),...
%         l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+l_EF*sin(exo_t6)+...
%         l_FG*sin(exo_t7)+l_GK*sin(t_pip)+0.001,...
%         'K','FontName','Times New Roman','FontSize',18);
%     
%     text((l_BC*cos(exo_t1)+l_CE*cos(exo_t5)+l_EF*cos(exo_t6)+...
%         l_FG*cos(exo_t7)+l_GK*cos(t_pip)+l_KJ*cos(t_dip)-0.002),...
%         l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+l_EF*sin(exo_t6)+...
%         l_FG*sin(exo_t7)+l_GK*sin(t_pip)+l_KJ*sin(t_dip)+0.001,...
%         'J','FontName','Times New Roman','FontSize',18);
%     
% %     plot(-[0 real(l_BC*exp(i*exo_t1)),...
% %         real(l_BC*exp(i*exo_t1)+l_CD*exp(i*exo_t2)),...
% %         real(l_BC*exp(i*exo_t1)+l_CD*exp(i*exo_t2)+exo_x3*exp(i*(t_mcp-pi)))],...
% %         [0 imag(l_BC*exp(i*exo_t1)),...
% %         imag(l_BC*exp(i*exo_t1)+l_CD*exp(i*exo_t2)),...
% %         imag(l_BC*exp(i*exo_t1)+l_CD*exp(i*exo_t2)+exo_x3*exp(i*(t_mcp-pi)))]);
% %     
% %     % Plotting PIP Chain
% %     plot(-[l_BC*cos(exo_t1),...
% %         l_BC*cos(exo_t1)+l_CE*cos(exo_t5),...
% %         l_BC*cos(exo_t1)+l_CE*cos(exo_t5)+l_EF*cos(exo_t6),...
% %         l_BC*cos(exo_t1)+l_CE*cos(exo_t5)+l_EF*cos(exo_t6)+l_FG*cos(exo_t7),...
% %         l_BC*cos(exo_t1)+l_CE*cos(exo_t5)+l_EF*cos(exo_t6)+l_FG*cos(exo_t7)-l_GH*cos(t_pip),...
% %         l_BC*cos(exo_t1)+l_CE*cos(exo_t5)+l_EF*cos(exo_t6)+...
% %         l_FG*cos(exo_t7)-l_GH*cos(t_pip)-(l_AH-exo_x3)*cos(t_mcp)],...
% %         [l_BC*sin(exo_t1),...
% %         l_BC*sin(exo_t1)+l_CE*sin(exo_t5),...
% %         l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+l_EF*sin(exo_t6),...
% %         l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+l_EF*sin(exo_t6)+l_FG*sin(exo_t7),...
% %         l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+l_EF*sin(exo_t6)+l_FG*sin(exo_t7)-l_GH*sin(t_pip),...
% %         l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+l_EF*sin(exo_t6)+...
% %         l_FG*sin(exo_t7)-l_GH*sin(t_pip)-(l_AH-exo_x3)*sin(t_mcp)]);
% %     
% %     % Plotting DIP Chain
% %     plot(-[l_BC*cos(exo_t1)+l_CE*cos(exo_t5)+l_EF*cos(exo_t6)+l_FG*cos(exo_t7),...
% %         l_BC*cos(exo_t1)+l_CE*cos(exo_t5)+l_EF*cos(exo_t6)+l_FG*cos(exo_t7)+l_GK*cos(t_pip),...
% %         l_BC*cos(exo_t1)+l_CE*cos(exo_t5)+l_EF*cos(exo_t6)+l_FG*cos(exo_t7)+l_GK*cos(t_pip)+l_KJ*cos(t_dip)],...
% %         [l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+l_EF*sin(exo_t6)+l_FG*sin(exo_t7),...
% %         l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+l_EF*sin(exo_t6)+l_FG*sin(exo_t7)+l_GK*sin(t_pip),...
% %         l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+l_EF*sin(exo_t6)+l_FG*sin(exo_t7)+l_GK*sin(t_pip)+l_KJ*sin(t_dip)]);
% %     
% %     axis([-0.1-rm 0.14 -0.1 0.055]);
% %     hold on;
% %     ht = text(-0.001,-0.001,'B','FontSize',18);
% %     set(ht,'FontName','Times New Roman');
% %     text(-(real(l_BC*exp(i*exo_t1))+0.001),imag(l_BC*exp(i*exo_t1))+0.001,...
% %         'C','FontName','Times New Roman','FontSize',18);
% %     text(-(real(l_BC*exp(i*exo_t1)+l_CD*exp(i*exo_t2))-0.002),...
% %         imag(l_BC*exp(i*exo_t1)+l_CD*exp(i*exo_t2))+0.001,...
% %         'D','FontName','Times New Roman','FontSize',18);
% %     text(-(real(l_BC*exp(i*exo_t1)+l_CD*exp(i*exo_t2)+exo_x3*exp(i*(t_mcp-pi)))-0.002),...
% %         imag(l_BC*exp(i*exo_t1)+l_CD*exp(i*exo_t2)+exo_x3*exp(i*(t_mcp-pi)))+0.001,...
% %         'A','FontName','Times New Roman','FontSize',18);
% %     
% %     text(-(l_BC*cos(exo_t1)+l_CE*cos(exo_t5)-0.002),...
% %         l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+0.001,...
% %         'E','FontName','Times New Roman','FontSize',18);
% %     
% %     text(-(l_BC*cos(exo_t1)+l_CE*cos(exo_t5)+l_EF*cos(exo_t6)-0.002),...
% %         l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+l_EF*sin(exo_t6)+0.001,...
% %         'F','FontName','Times New Roman','FontSize',18);
% %     
% %     text(-(l_BC*cos(exo_t1)+l_CE*cos(exo_t5)+l_EF*cos(exo_t6)+l_FG*cos(exo_t7)-0.002),...
% %         l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+l_EF*sin(exo_t6)+l_FG*sin(exo_t7)+0.001,...
% %         'G','FontName','Times New Roman','FontSize',18);
% %     
% %     text(-(l_BC*cos(exo_t1)+l_CE*cos(exo_t5)+l_EF*cos(exo_t6)+l_FG*cos(exo_t7)-l_GH*cos(t_pip)-0.002),...
% %         l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+l_EF*sin(exo_t6)+l_FG*sin(exo_t7)-l_GH*sin(t_pip)+0.001,...
% %         'H','FontName','Times New Roman','FontSize',18);
% %     
% % %     text(l_BC*cos(exo_t1)+l_CE*cos(exo_t5)+l_EF*cos(exo_t6)+l_FI*cos(exo_t9)-0.002,...
% % %         l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+l_EF*sin(exo_t6)+l_FI*sin(exo_t9)+0.001,...
% % %         'I','FontName','Times New Roman','FontSize',18);
% %     
% %     text(-(l_BC*cos(exo_t1)+l_CE*cos(exo_t5)+l_EF*cos(exo_t6)+...
% %         l_FG*cos(exo_t7)+l_GK*cos(t_pip)-0.001),...
% %         l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+l_EF*sin(exo_t6)+...
% %         l_FG*sin(exo_t7)+l_GK*sin(t_pip)+0.001,...
% %         'K','FontName','Times New Roman','FontSize',18);
% %     
% %     text(-(l_BC*cos(exo_t1)+l_CE*cos(exo_t5)+l_EF*cos(exo_t6)+...
% %         l_FG*cos(exo_t7)+l_GK*cos(t_pip)+l_KJ*cos(t_dip)-0.002),...
% %         l_BC*sin(exo_t1)+l_CE*sin(exo_t5)+l_EF*sin(exo_t6)+...
% %         l_FG*sin(exo_t7)+l_GK*sin(t_pip)+l_KJ*sin(t_dip)+0.001,...
% %         'J','FontName','Times New Roman','FontSize',18);
%     
%     grid on;
%     drawnow;
end


% leg_left = 0.35;
% leg_bottom = 0.84;
% leg_width = 0.5;
% leg_height = 0.09;
%
% figure(3);
% clf;
% plot((T_sensor(start:finish)-T_sensor(start))/1000,(2*pi-Estimates(start:finish,4))*180/pi,'-r',...
%     (T_sensor(start:finish)-T_sensor(start))/1000,(-Estimates(start:finish,8)+Estimates(start:finish,4))*180/pi,'--g',...
%     (T_sensor(start:finish)-T_sensor(start))/1000,(2*pi-Estimates(start:finish,12)+...
%     Estimates(start:finish,8)-Estimates(start:finish,4))*180/pi,'-.b','LineWidth',2);
% xlim([0 T_sensor(finish)-T_sensor(start)]/1000);
% ylim([-10 90]);
% grid on;
% hold on;
% set(gca,'FontSize',fontsize-4);
% title('Index Finger Joint Angles with Exoskeleton','Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% xlabel('Time (s) $\rightarrow$','Interpreter','latex','FontName',...
%     'Times New Roman','FontSize',fontsize);
% ylabel('Joint Angles $(\theta)$ (deg) $\rightarrow$','Interpreter','latex','FontName',...
%     'Times New Roman','FontSize',fontsize);
% str = {'$\theta_{mcp}$','$\theta_{pip}$','$\theta_{dip}$'};
% h_leg = legend(str,...
%     'Interpreter','latex','FontName','Times New Roman','FontSize',fontsize-2,...
%     'Color','none','Location',[leg_left leg_bottom leg_width leg_height],...
%     'Orientation','Horizontal','Box','on');
% legend('boxoff');
%
% figure(5);
% clf;
% plot((T_sensor(start:finish)-T_sensor(start))/1000,Tau(start:finish,1),'-r',...
%     (T_sensor(start:finish)-T_sensor(start))/1000,Tau(start:finish,2),'--g',...
%     'LineWidth',2);
% xlim([0 T_sensor(finish)-T_sensor(start)]/1000);
% grid on;
% hold on;
% set(gca,'FontSize',fontsize-4);
% title('Index Finger Exoskeleton Torques','Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% xlabel('Time (s) $\rightarrow$','Interpreter','latex','FontName',...
%     'Times New Roman','FontSize',fontsize);
% ylabel('Joint Torque $(\tau)$ (Nm) $\rightarrow$','Interpreter','latex',...
%     'FontName',...
%     'Times New Roman','FontSize',fontsize);
% str = {'$\tau_{1}$','$\tau_{6}$'};
% h_leg = legend(str,...
%     'Interpreter','latex','FontName','Times New Roman','FontSize',fontsize-2,...
%     'Color','none','Location',[leg_left leg_bottom leg_width leg_height],...
%     'Orientation','Horizontal','Box','on');
% legend('boxoff');
%
% figure(6);
% clf;
% plot((T_sensor(start:finish)-T_sensor(start))/1000,Tau_mcp(start:finish),'-r',...
%     (T_sensor(start:finish)-T_sensor(start))/1000,Tau_pip(start:finish),'--g',...
%     'LineWidth',2);
% xlim([0 T_sensor(finish)-T_sensor(start)]/1000);
% grid on;
% hold on;
% set(gca,'FontSize',fontsize-4);
% title('Finger Torques','Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
% xlabel('Time (s) $\rightarrow$','Interpreter','latex','FontName',...
%     'Times New Roman','FontSize',fontsize);
% ylabel('Joint Torque $(\tau)$ (Nm) $\rightarrow$','Interpreter','latex',...
%     'FontName',...
%     'Times New Roman','FontSize',fontsize);
% str = {'$\tau_{mcp}$','$\tau_{pip}$'};
% h_leg = legend(str,...
%     'Interpreter','latex','FontName','Times New Roman','FontSize',fontsize-2,...
%     'Color','none','Location',[leg_left leg_bottom leg_width leg_height],...
%     'Orientation','Horizontal','Box','on');
% legend('boxoff');

% %% Printing the results in pdf files
% print_str={'joint_angles','exoskeleton_torque','finger_torque'};
% try
%     figure_array = {3,5,6};
%     if(print_flag)
%         for j=1:3
%             figure(figure_array{j});
%             export_fig([print_dir '/' print_str{j} '_' subject_name '_E_'...
%                 experiment_type ],'-pdf',...
%                 '-transparent');
%         end
%     end
% end