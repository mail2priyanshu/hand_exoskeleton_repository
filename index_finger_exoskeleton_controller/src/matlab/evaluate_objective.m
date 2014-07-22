function [val] = evaluate_objective(x,Theta_r)
start_ind = 1;
end_ind = 1000;
% J_final = zeros(end_ind-start_ind+1,4,2);
% Estimates = zeros(end_ind-start_ind+1,12);

x_A = x(1);
y_A = x(2);
l_BC = x(3);
l_CD = x(4);
l_AH = x(5);
l_CE = 0.04;
l_EF = 0.035;
l_FG = x(6);
l_GH = x(7);
l_GK = 0.007;
l_FI = 0.02617;
l_IJ = 0.01968;
    
val = 0;
for i=start_ind:end_ind
    exo_t_rel = [Theta_r(i,1), Theta_r(i,3), Theta_r(i,4)]'*pi/180;
%     [tau_finger,tau_finger_dot,...
%         tau_exo,tau_exo_dot,J,J_dot,...
%         K_1,K_2,K1,K2,~,~,estimates] = index_finger_exoskeleton_statics(exo_t_rel,...
%         Theta_r_dot(i,:)',0);       
    
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
%     exo_t6 = estimates(6);
%     exo_t7 = estimates(7);
    t_pip = estimates(8);
%     exo_t9 = estimates(9);
%     exo_t10 = estimates(10);
%     exo_x11 = estimates(11);
    t_dip = estimates(12);
    
    %     % Position input
    %     exo_t1 = estimates(1);
    %     exo_x3 = estimates(3);
    %     t_mcp = estimates(4);
    %     exo_t5 = estimates(5);
    %     exo_t6 = estimates(6);
    %     exo_t7 = estimates(7);
    %     t_pip = estimates(8);
    %     exo_t6r = exo_t_rel(2);
    %
    %     l_FH = sqrt(l_GH*l_GH+l_FG*l_FG);
    %
    %     t_HFG = atan(l_GH/l_FG);
    %     l_DH = l_AH-exo_x3;
    %     l_CH = sqrt(l_CD*l_CD+l_DH*l_DH);
    %     t_HFG = atan(l_GH/l_FG);
    %     t_DCH = atan(l_DH/l_CD);
    %
    %     J = index_finger_J(exo_t1,exo_t5,exo_x3,exo_t6r,...
    %         l_AH,l_BC,l_CD,l_CE,l_EF,l_FH,t_HFG,t_mcp,t_pip);
    %
    %     J_final(i,:,:) = J;
%     Estimates(i,:) = estimates*180/pi;
    
    theta_5r = exo_t1-pi-exo_t5;
        
%     if(wrapTo180(2*pi-t_mcp)>-20*pi/180 &&...
%        wrapTo180(2*pi-t_mcp)<70*pi/180 &&...
%        wrapTo180(t_mcp-t_pip)>-10*pi/180 &&...
%        wrapTo180(t_mcp-t_pip)<70*pi/180)        
    val = val+(Theta_r(i,2)*pi/180-theta_5r)^2;
%     else
%         val = 100
%         return;
%     end
end

val
% Theta_finger(:,1) = wrapTo180((2*180-Estimates(:,4)));
% Theta_finger(:,2) = wrapTo180((2*180-Estimates(:,8))-Theta_finger(:,1));

