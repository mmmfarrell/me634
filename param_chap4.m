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
P.u0     = 25;  % initial velocity along body x-axis
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
P.wind_n    = -20;
P.wind_e    = 0;
P.wind_d    = 0;

% Dryden Gust Model Parameters
% Low alt, light turb
P.sigma_u = 1.06;
P.sigma_v = 1.06;
P.sigma_w = 0.7;
P.L_u = 200;
P.L_v = 200;
P.L_w = 50;

