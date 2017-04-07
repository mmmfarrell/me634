P.gravity = 9.8;
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params for Aersonade UAV
%physical parameters of airframe
P.mass = 25;
P.Jx   = 0.8244;
P.Jy   = 1.135;
P.Jz   = 1.759;
P.Jxz  = .1204;

% aerodynamic coefficients
P.S_wing        = 0.55;
P.b             = 2.8956;
P.c             = 0.18994;
P.S_prop        = 0.2027;
P.rho           = 1.2682;
P.k_motor       = 80;
P.k_T_P         = 0;
P.k_Omega       = 0;
P.e             = 0.9;

P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_alpha     = 0.30;
P.C_D_p         = 0.0437;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_m_0         = -0.02338;
P.C_m_alpha     = -0.38;
P.C_m_q         = -3.6;
P.C_m_delta_e   = -0.5;
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.98;
P.C_Y_p         = 0.0;
P.C_Y_r         = 0.0;
P.C_Y_delta_a   = 0.0;
P.C_Y_delta_r   = -0.17;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.12;
P.C_ell_p       = -0.26;
P.C_ell_r       = 0.14;
P.C_ell_delta_a = 0.08;
P.C_ell_delta_r = 0.105;
P.C_n_0         = 0.0;
P.C_n_beta      = 0.25;
P.C_n_p         = 0.022;
P.C_n_r         = -0.35;
P.C_n_delta_a   = 0.06;
P.C_n_delta_r   = -0.032;
P.C_prop        = 1.0;
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

P.gamma = P.Jx*P.Jz - (P.Jxz)^2;
P.gamma1 = (P.Jxz*(P.Jx-P.Jy+P.Jz))/P.gamma;
P.gamma2 = (P.Jz*(P.Jz-P.Jy)+P.Jxz^2)/P.gamma;
P.gamma3 = P.Jz/P.gamma;
P.gamma4 = P.Jxz/P.gamma;
P.gamma5 = (P.Jz-P.Jx)/P.Jy;
P.gamma6 = P.Jxz/P.Jy;
P.gamma7 = ((P.Jx-P.Jy)*P.Jx + P.Jxz^2)/P.gamma;
P.gamma8 = P.Jx/P.gamma;

%%%%%%%%

P.C_p_0 = P.gamma3*P.C_ell_0 + P.gamma4*P.C_n_0;
P.C_p_beta = P.gamma3*P.C_ell_beta + P.gamma4*P.C_n_beta;
P.C_p_p = P.gamma3*P.C_ell_p + P.gamma4*P.C_n_p;
P.C_p_r = P.gamma3*P.C_ell_r + P.gamma4*P.C_n_p;
P.C_p_delta_a = P.gamma3*P.C_ell_delta_a + P.gamma4*P.C_n_delta_a;
P.C_p_delta_r = P.gamma3*P.C_ell_delta_r + P.gamma4*P.C_n_delta_r;

P.C_r_0 = P.gamma4*P.C_ell_0 + P.gamma8*P.C_n_0;
P.C_r_beta = P.gamma4*P.C_ell_beta + P.gamma8*P.C_n_beta;
P.C_r_p = P.gamma4*P.C_ell_p + P.gamma8*P.C_n_p;
P.C_r_r = P.gamma4*P.C_ell_r + P.gamma8*P.C_n_r;
P.C_r_delta_a = P.gamma4*P.C_ell_delta_a + P.gamma8*P.C_n_delta_a;
P.C_r_delta_r = P.gamma4*P.C_ell_delta_r + P.gamma8*P.C_n_delta_r;

P.AR = (P.b^2)/P.S_wing;

%%%%%%%

% wind parameters
P.wind_n = 1.5;%3;
P.wind_e = 1;%2;
P.wind_d = 0;
P.L_u = 200;
P.L_v = 200;
P.L_w = 50;
P.sigma_u = 1.06; 
P.sigma_v = 1.06;
P.sigma_w = .7;


% compute trim conditions using 'mavsim_chap5_trim.slx'
% initial airspeed
P.Va0 = 35;
gamma = 0*pi/180;  % desired flight path angle (radians)
R     = Inf;         % desired radius (m) - use (+) for right handed orbit, 
P.lambda = 100;

% autopilot sample rate
P.Ts = 0.01;

% Dirty Derivative gain
P.tau = 0.05;

% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = P.Va0; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

                    %                          (-) for left handed orbit

% run trim commands
[x_trim, u_trim]=compute_trim('mavsim_trim',P.Va0,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;

% set initial conditions to trim conditions
% initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -200;  % initial Down position (negative altitude)
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle
P.psi0   = x_trim(9);  % initial yaw angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate

% compute different transfer functions
[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim,P);

% linearize the equations of motion around trim conditions
[A_lon, B_lon, A_lat, B_lat] = compute_ss_model('mavsim_trim',x_trim,u_trim);

computeGains;

%%%%%%%%%%%%%%%
%%% Sensors %%%
%%%%%%%%%%%%%%%
P.sigma_gyro = 0.13 * pi/180.0; % rad/s Appendix H.1 for ADXRS540
P.bias_gyro_x = 0;
P.bias_gyro_y = 0;
P.bias_gyro_z = 0;

P.sigma_accel = 0.0025*P.gravity; % m/s/s Appendix H.2 for ADXL325

P.sigma_abs_press = 0.01*1000; % Pa Appendix H.3
P.beta_abs_press = 0;%0.125*1000; % Pa Appendix H.3

P.sigma_diff_press = 0.002 * 1000; % Pa Appendix H.3
P.beta_diff_press = 0;%0.020*1000; % Pa Appendix H.3

P.mag_inclination = 65.7 * pi/180; % rad Pg. 132
P.mag_declination = 12.12 * pi/180; % rad Pg. 131

% Init mag field in inertial frame
mag_north = cos(P.mag_inclination)*cos(P.mag_declination);
mag_east = cos(P.mag_inclination)*sin(P.mag_declination);
mag_down = sin(P.mag_inclination);
P.mag_inertia = [mag_north;mag_east;mag_down];

%%% GPS %%% from Table 7.2 pg. 139
P.Ts_gps = 1.0; % s
P.gps_k = 1/1100; % 1/s
P.gps_sigma_n = 0.21; % m
P.gps_sigma_e = 0.21; % m
P.gps_sigma_alt = 0.40; % m
P.gps_sigma_Vg = 0.05; % m/s
P.gps_sigma_x = P.gps_sigma_Vg/P.Va0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%% Filtering %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% LPF Params
P.lpf_a = 50; % Cutoff freq for lpf
P.lpf_alpha = exp(-P.lpf_a*P.Ts);
P.lpf_a1 = 1.5; % Cutoff freq for lpf
P.lpf_alpha1 = exp(-P.lpf_a1*P.Ts);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% Guidance Model Update %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
P.b_chidot = 1.0;
P.b_Va = 1.0;
P.b_chi = 0.75;
P.b_hdot = 1.1;
P.b_h = 2.0;
P.gamma_max = 45*pi/180;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Path Follow Gains %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

P.Chi_inf = 60*pi/180;
P.k_path = 0.010;
P.k_orbit = 2.5;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% chapter 11 - path manager %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
P.phi_max = 45*pi/180;
% number of waypoints in data structure
P.size_waypoint_array = 100;
P.R_min = P.Va0^2/P.gravity/tan(P.phi_max);

% create random city map
city_width      = 2000;  % the city is of size (width)x(width)
building_height = 300;   % maximum height of buildings
%building_height = 1;   % maximum height of buildings (for camera)
num_blocks      = 5;    % number of blocks in city
street_width    = .8;   % percent of block that is street.
% P.pd0           = -h0;  % initial height of MAV
P.map = createWorld(city_width, building_height, num_blocks, street_width);

