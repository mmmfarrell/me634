
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

