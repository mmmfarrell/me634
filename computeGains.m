

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gains
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Roll Hold
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%% Tuning Params %%%%%%
% Max step
phi_step_max = 60*pi/180;
roll_zeta = 0.7;

% TF delta_a to phi
[num,den] = tfdata(T_phi_delta_a, 'v');
a_phi2 = num(3);
a_phi1 = den(2);

% Max aileron command
delta_a_max = 45*pi/180;

P.roll_kp = delta_a_max/phi_step_max;

wn_phi = sqrt(P.roll_kp*a_phi2);

P.roll_kd = (2*roll_zeta*wn_phi-a_phi1)/a_phi2;

%%%%%%%%%%%%%%%%%%%%%%%%
% Course Hold
%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%% Tuning Params %%%%%
% Bandwidth Separation
Wx = 20;
zeta_chi = 0.7;

% TF delta_a to phi
[num,den] = tfdata(T_chi_phi, 'v');
gVa = num(2);

wn_chi = (1/Wx)*wn_phi;

P.course_kp = 2*zeta_chi*wn_chi/gVa;
P.course_ki = (wn_chi^2)/gVa;


%%%%%%%%%%%%%%%%%%%%%%%%
% Pitch command Hold
%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%% Tuning Params %%%%%
theta_step_max = 30*pi/180;
pitch_zeta = 0.7;
theta_max = 30*pi/180;

% TF delta_e to theta
[num,den]=tfdata(T_theta_delta_e,'v');
a_theta1 = den(2);
a_theta2 = den(3);
a_theta3 = num(3);

P.pitch_kp = ((theta_max)/(theta_step_max))*sign(a_theta3);

wn_theta = sqrt(a_theta2 + P.pitch_kp*a_theta3);

P.pitch_kd = -(2*pitch_zeta*wn_theta - a_theta1)/a_theta3;

P.pitch_DC = (P.pitch_kp*a_theta3)/(a_theta2 + P.pitch_kp*a_theta3);


%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% TECS Gains %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%% Theta Command from Energy Diff %%%%%
P.TECS_theta_kp = 0.75; % 1
P.TECS_theta_kd = 0.5; % 0.75
P.TECS_theta_ki = 0.25; % 0.5

%%%% Throttle from Total Energy %%%%%
P.TECS_T_kp = 0.75; % 1
P.TECS_T_kd = 0.5; % 0.75
P.TECS_T_ki = 0.25; % 0.5