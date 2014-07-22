function [x] = solve_quad_eqn(A,B,C)
if(C~=B && A*A+B*B-C*C>0) % equation is quadratic
    x(1) = 2*atan2(-A+sqrt(A*A+B*B-C*C),C-B);
    x(2) = 2*atan2(-A-sqrt(A*A+B*B-C*C),C-B);
elseif(abs(C-B)<1e-10) % equation is linear
    x(1) = 2*atan2(-(B+C),(2*A));
    x(2) = x(1);
else % find the minimizing solution
    x(1) = 2*atan2(-A,(C-B));
    x(2) = x(1);
    % else
    %     x(1) = NaN;
    %     x(2) = NaN;
    %     disp('Failed to solve quadratic!')
end

