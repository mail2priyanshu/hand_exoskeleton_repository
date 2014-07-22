clear all
clc

filepath = '/home/reneu-t1600/reneu-backup/Hand_Exoskeleton_Programs/30082013_exoskeleton_kinematics_experiments';
filename_sensor = 'encoder_data.csv';
data_sensor = dlmread([filepath '/' filename_sensor],'\t');
T_sensor = data_sensor(:,1);

str = {'time','$\theta_1$','$\theta_2$','$x_3$','$\theta_4$',...
    '$\theta_5$','$\theta_6$','$\theta_7$','$\theta_8$','$\theta_9$',...
    '$\theta_{10}$','$x_{11}$','$\theta_{12}$'};

fontsize = 14;

% figure(1);
% bar(data_sensor(:,1));
% title([str{1} ' vs Time'],'Interpreter','latex',...
%     'FontName','Times New Roman','FontSize',fontsize);
start =8250;
finish = 19000;

% for i=[4,8,12]
%     figure(i+1);
%     plot(T_sensor(start:finish),(data_sensor(start:finish,i+1)-min(data_sensor(start:finish,i+1)))*180/pi);
%     title([str{i+1} ' vs Time'],'Interpreter','latex',...
%         'FontName','Times New Roman','FontSize',fontsize);
% end

for i=[1,6,9]
    figure(i+1);
    plot(T_sensor(start:finish),(data_sensor(start:finish,i+1)-min(data_sensor(start:finish,i+1)))*180/pi);
    title([str{i+1} ' vs Time'],'Interpreter','latex',...
        'FontName','Times New Roman','FontSize',fontsize);
end

% for i=2:13
%     figure(i-1);
%     plot(T_sensor(start:finish),data_sensor(start:finish,i)*180/pi);
%     title([str{i} ' vs Time'],'Interpreter','latex',...
%         'FontName','Times New Roman','FontSize',fontsize);
% end