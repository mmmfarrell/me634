% estimate_states
%   - estimate the MAV states using gyros, accels, pressure sensors, and
%   GPS.
%
% Outputs are:
%   pnhat    - estimated North position, 
%   pehat    - estimated East position, 
%   hhat     - estimated altitude, 
%   Vahat    - estimated airspeed, 
%   alphahat - estimated angle of attack
%   betahat  - estimated sideslip angle
%   phihat   - estimated roll angle, 
%   thetahat - estimated pitch angel, 
%   chihat   - estimated course, 
%   phat     - estimated roll rate, 
%   qhat     - estimated pitch rate, 
%   rhat     - estimated yaw rate,
%   Vghat    - estimated ground speed, 
%   wnhat    - estimate of North wind, 
%   wehat    - estimate of East wind
%   psihat   - estimate of heading angle
% 
% 
% Modified:  3/15/2010 - RB
%            5/18/2010 - RB
%

function xhat = estimate_states(uu, P)

   % rename inputs
   y_gyro_x      = uu(1);
   y_gyro_y      = uu(2);
   y_gyro_z      = uu(3);
   y_accel_x     = uu(4);
   y_accel_y     = uu(5);
   y_accel_z     = uu(6);
   y_static_pres = uu(7);
   y_diff_pres   = uu(8);
   y_gps_n       = uu(9);
   y_gps_e       = uu(10);
   y_gps_h       = uu(11);
   y_gps_Vg      = uu(12);
   y_gps_course  = uu(13);
   t             = uu(14);
   
   g = P.gravity;
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %%%%%%%%%%% LPF  Section 8.3 %%%%%%%%%%%%%%%%%
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
   persistent lpf_gyro_x
   persistent lpf_gyro_y
   persistent lpf_gyro_z
   persistent lpf_accel_x
   persistent lpf_accel_y
   persistent lpf_accel_z
   persistent lpf_static_pres
   persistent lpf_diff_pres
   persistent lpf_gps_n
   persistent lpf_gps_e
   persistent lpf_gps_Vg
   persistent lpf_gps_course
   
   if t==0
       lpf_gyro_x = 0;
       lpf_gyro_y = 0;
       lpf_gyro_z = 0;
       lpf_accel_x = 0;
       lpf_accel_y = 0;
       lpf_accel_z = 0;
       lpf_static_pres = P.rho*P.gravity*(-P.pd0);
       lpf_diff_pres = 0.5*P.rho*P.Va0^2;
   end
   
   %%% LPF Angular Rates %%%
   lpf_gyro_x = P.lpf_alpha*lpf_gyro_x + (1 - P.lpf_alpha)*y_gyro_x;
   lpf_gyro_y = P.lpf_alpha*lpf_gyro_y + (1 - P.lpf_alpha)*y_gyro_y;
   lpf_gyro_z = P.lpf_alpha*lpf_gyro_z + (1 - P.lpf_alpha)*y_gyro_z;
   phat = lpf_gyro_x;
   qhat = lpf_gyro_y;
   rhat = lpf_gyro_z;
   
   %%% LPF Altitude from Static Pres %%%
   lpf_static_pres = P.lpf_alpha*lpf_static_pres + (1 - P.lpf_alpha)*y_static_pres;
   hhat = lpf_static_pres/P.rho/P.gravity;
   
   %%% LPF Air Speed from diff Pres %%%
   lpf_diff_pres = P.lpf_alpha*lpf_diff_pres + (1 - P.lpf_alpha)*y_diff_pres;
   Vahat = sqrt(2*lpf_diff_pres/P.rho);
   
   %%% LPF Roll, Pitch from Accel %%%
   lpf_accel_x = P.lpf_alpha*lpf_accel_x + (1 - P.lpf_alpha)*y_accel_x;
   lpf_accel_y = P.lpf_alpha*lpf_accel_y + (1 - P.lpf_alpha)*y_accel_y;
   lpf_accel_z = P.lpf_alpha*lpf_accel_z + (1 - P.lpf_alpha)*y_accel_z;
   phihat = atan(lpf_accel_y/lpf_accel_z);
   thetahat = asin(lpf_accel_x/P.gravity);
   
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %%%%%% EKF %%%%%%%%%%%%%%%%%%%%%%%%%%%
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
   %%%%%%%%%%%%%%%%%%%%%%%
   % Attitude Estimation %
   %%%%%%%%%%%%%%%%%%%%%%%
   
   persistent xhat_a 
   persistent y_accel_x_old
   persistent P_a
   
   % Init persistent variables
   if t ==0
       xhat_a = [P.phi0; P.theta0];
       P_a = [1, 0; 0, 1];
   end
   
   % Matricies
   R_accel = P.sigma_accel^2;
   Q_a = [0.0000001, 0; 0, 0.0000001];
   
   %%%% Prediction %%%%
   N = 10; % Prediction steps
   for i=1:N
       % States
       phi = xhat_a(1);
       theta = xhat_a(2);
   
       % Trigs
       sp = sin(phi);
       cp = cos(phi);
       ct = cos(theta);
       tt = tan(theta);
   
       f_a = [phat + qhat*sp*tt + rhat*cp*tt;...
       qhat*cp - rhat*sp];
   
       df_a = [qhat*cp*tt-rhat*sp*tt, (qhat*sp-rhat*cp)/((ct)^2);...
       -qhat*sp-rhat*cp, 0];
   
       G_a = [ 1, sp*tt, cp*tt; ...
                0, cp, -sp];
   
       xhat_a = xhat_a + (P.Ts/N)*f_a;
       A_a = df_a;
       P_a = P_a + (P.Ts/N)*(A_a*P_a + P_a*A_a' + Q_a);
   end
   
   %%%% Update %%%%
   % Equations
   sp = sin(phi);
   cp = cos(phi);
   ct = cos(theta);
   st = sin(theta);
       
   h_a = [qhat*Vahat*st+g*st;...
       rhat*Vahat*ct-phat*Vahat*st-g*ct*sp;...
       -qhat*Vahat*ct-g*ct*cp];
   
   dh_a = [0, qhat*Vahat*ct+g*ct;...
       -g*cp*ct, -rhat*Vahat*st-phat*Vahat*ct+g*sp*st;...
       g*sp*ct, (qhat*Vahat+g*cp)*st];
   
   if (y_accel_x_old ~= y_accel_x)
       % x-axis accel
       h_i = h_a(1);
       C_i = dh_a(1,:);
       L_a = P_a*C_i'/(R_accel + C_i*P_a*C_i');
       P_a = (eye(2) - L_a*C_i)*P_a;
       xhat_a = xhat_a + L_a*(y_accel_x - h_i);
       % y-axis accel
       h_i = h_a(2);
       C_i = dh_a(2,:);
       L_a = P_a*C_i'/(R_accel + C_i*P_a*C_i');
       P_a = (eye(2) - L_a*C_i)*P_a;
       xhat_a = xhat_a + L_a*(y_accel_y - h_i);
       % z-axis accel
       h_i = h_a(3);
       C_i = dh_a(3,:);
       L_a = P_a*C_i'/(R_accel + C_i*P_a*C_i');
       P_a = (eye(2) - L_a*C_i)*P_a;
       xhat_a = xhat_a + L_a*(y_accel_z - h_i);
   end
   
   % update estimates
   phihat = xhat_a(1);
   thetahat = xhat_a(2);
       
    % not estimating these states 
    alphahat = 0;
    betahat  = 0;
    bxhat    = 0;
    byhat    = 0;
    bzhat    = 0;
    
    pnhat    = 0;  
    pehat    = 0; 
    hhat     = 0; 
    Vahat     = 0;
    chihat     = 0;
    phat     = 0;
    qhat     = 0;
    rhat     = 0;
    Vghat     = 0;
    wnhat     = 0;
    wehat     = 0;
    psihat     = 0;
     
    
      xhat = [...
        pnhat;...   
        pehat;...
        hhat;...
        Vahat;...
        alphahat;...
        betahat;...
        phihat;...
        thetahat;...
        chihat;...
        phat;...
        qhat;...
        rhat;...
        Vghat;...
        wnhat;...
        wehat;...
        psihat;...
        bxhat;...
        byhat;...
        bzhat;...
        ];
end
