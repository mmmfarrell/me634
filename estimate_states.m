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
   
   
    % not estimating these states 
    alphahat = 0;
    betahat  = 0;
    bxhat    = 0;
    byhat    = 0;
    bzhat    = 0;
    
    %%%%%% Algorithm 2 EKF Pg. 156
    % Init
    % xhat = X0
    
    % Sample Rate < sample rate of sensors
    
    % At each sample Time Tout:
    
    % for i = 1 to N do {Prediction}
    %   xhat = xhat + (Tout/N)*f(xhat,u)
    %   A = (df/dx)(xhat,u)
    %   P = P + (Tout/N)*(AP + PA^T + Q)
    % end for
    
    % if Measurment received from sensor i then {Update}
    %   Ci = (dhi/dx)(xhat,u[n])
    %   Li = PCi^T(Ri+CiPCi^T)^-1
    %   P = (I - LiCi)P
    %   xhat = xhat + Li(yi[n] - h(xhat,u[n]))
    % end if 
    
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
