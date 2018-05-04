function K = sm_cart_dpen_control_gains(sys,poles)
%SM_CART_DPEN_CONTROL_GAINS - Uses pole placement technique to design the
%                             control gains given the linearized model and 
%                             the desired closed loop pole locations

% Copyright 2011 The MathWorks, Inc.

% Compute Controllability Matrix
[~,n] = size(sys.A);
Wc = sys.B;
for idx=1:n-1
    newCol = sys.A*Wc(:,end);
    Wc = [Wc newCol];
end

% Use Ackermann's formula to compute control gains (Pole Placement).
K = Wc\polyvalm(real(poly(poles)),sys.A);
K = K(n,:);