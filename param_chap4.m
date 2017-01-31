%%%%%%%
% Param file to be run at start of simulink
%%%%


% Airframe Params from file
run aerosonde.m

P.gravity = 9.8;
   
% Lambda = adjustable gain for quaternions
P.lambda = 100;

% initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0; % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = 0;  % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

%
P.Va0 = 25;
P.Ts = 0.01;

% Wind
P.wind_n    = 5;
P.wind_e    = 5;
P.wind_d    = 5;

% Gusts
P.sigma_u = 0.02;
P.sigma_v = 0.02;
P.sigma_w = 0.02;
P.L_u = 1;
P.L_v = 1;
P.L_w = 1;

