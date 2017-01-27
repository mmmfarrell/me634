function [sys,x0,str,ts,simStateCompliance] = mav_dynamics(t,x,u,flag,P)

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,P);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 12;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [...
    P.pn0;...
    P.pe0;...
    P.pd0;...
    P.u0;...
    P.v0;...
    P.w0;...
    P.phi0;...
    P.theta0;...
    P.psi0;...
    P.p0;...
    P.q0;...
    P.r0;...
    ];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,uu, P)

    pn    = x(1);
    pe    = x(2);
    pd    = x(3);
    u     = x(4);
    v     = x(5);
    w     = x(6);
    phi   = x(7);
    theta = x(8);
    psi   = x(9);
    p     = x(10);
    q     = x(11);
    r     = x(12);
    fx    = uu(1);
    fy    = uu(2);
    fz    = uu(3);
    ell   = uu(4);
    m     = uu(5);
    n     = uu(6);
    
    rot = [cos(theta)*cos(psi), -cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);...
        cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);...
        -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];
    
    comp1 = rot * [u; v; w];
    
    comp2 = [r*v-q*w; p*w-r*u; q*u-p*v] + (1/P.mass)*[fx; fy; fz];
    
    comp3 = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);...
        0, cos(phi), -sin(phi);...
        0, (sin(phi))/(cos(theta)), (cos(phi))/(cos(theta))]*[p; q; r];
    
    gamma = P.Jx*P.Jz - (P.Jxz)^2;
    gamma1 = (P.Jxz*(P.Jx-P.Jy+P.Jz))/gamma;
    gamma2 = (P.Jz*(P.Jz-P.Jy)+P.Jxz^2)/gamma;
    gamma3 = P.Jz/gamma;
    gamma4 = P.Jxz/gamma;
    gamma5 = (P.Jz-P.Jx)/P.Jy;
    gamma6 = P.Jxz/P.Jy;
    gamma7 = ((P.Jx-P.Jy)*P.Jx + P.Jxz^2)/gamma;
    gamma8 = P.Jx/gamma;
    
    comp4 = [gamma1*p*q-gamma2*q*r;...
        gamma5*p*r-gamma6*(p^2 - r^2);...
        gamma7*p*q-gamma1*q*r] + [gamma3*ell+gamma4*n;...
                                (1/P.Jy)*m;...
                                gamma4*ell+gamma8*n];
    
    pndot = comp1(1);
    pedot = comp1(2);
    pddot = comp1(3);
    udot = comp2(1);
    vdot = comp2(2);
    wdot = comp2(3);
    phidot = comp3(1);
    thetadot = comp3(2);
    psidot = comp3(3);
    pdot = comp4(1);
    qdot = comp4(2);
    rdot = comp4(3);

sys = [pndot; pedot; pddot; udot; vdot; wdot; phidot; thetadot; psidot; pdot; qdot; rdot];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)

sys = x;

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
