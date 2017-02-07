
function drawVehicle(uu,V,F,patchcolors)

    % process inputs to function
    pn       = uu(1);       % inertial North position     
    pe       = uu(2);       % inertial East position
    pd       = uu(3);           
    u        = uu(4);       
    v        = uu(5);       
    w        = uu(6);       
    e0       = uu(7);              
    e1       = uu(8);          
    e2       = uu(9); 
    e3       = uu(10);
    p        = uu(11);       % roll rate
    q        = uu(12);       % pitch rate     
    r        = uu(13);       % yaw rate    
    t        = uu(14);       % time

    % Convert quaternions to rpy
    rpy     = quat_to_euler([e0; e1; e2; e3]);
    phi     = rpy(1);
    theta   = rpy(2);
    psi     = rpy(3);
    
    % define persistent variables 
    persistent vehicle_handle;
    persistent Vertices
    persistent Faces
    persistent facecolors
    
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        [Vertices,Faces,facecolors] = defineVehicleBody;
        vehicle_handle = drawVehicleBody(Vertices,Faces,facecolors,...
                                               pn,pe,pd,phi,theta,psi,...
                                               [],'normal');
        title('Vehicle')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        grid on
        view(32,47)  % set the view angle for figure
        plot_size = 100;
        axis([-plot_size,plot_size,-plot_size,plot_size,-plot_size,plot_size]);
        hold on
        
    % at every other time step, redraw base and rod
    else 
        drawVehicleBody(Vertices,Faces,facecolors,...
                           pn,pe,pd,phi,theta,psi,...
                           vehicle_handle);
    end
end

  
%=======================================================================
% drawVehicle
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawVehicleBody(V,F,patchcolors,...
                                     pn,pe,pd,phi,theta,psi,...
                                     handle,mode)
%   V = translate(V, pn, pe, pd);  % translate vehicle
  
  V = rotate(V, phi, theta, psi);  % rotate vehicle
  V = translate(V, pn, pe, pd);  % translate vehicle

  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = R*V;
  
  if isempty(handle),
  handle = patch('Vertices', V', 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',V','Faces',F);
    xlim(handle.Parent,[pe-100, pe+100]);
    ylim(handle.Parent,[pn-100, pn+100]);
    zlim(handle.Parent,[-(pd+100), -(pd-100)]);
    drawnow
  end
end

%%%%%%%%%%%%%%%%%%%%%%%
function pts=rotate(pts,phi,theta,psi)

  % define rotation matrix (right handed)
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), sin(phi);...
          0, -sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, -sin(theta);...
          0, 1, 0;...
          sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), sin(psi), 0;...
          -sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_roll*R_pitch*R_yaw;  
    % note that R above either leaves the vector alone or rotates
    % a vector in a left handed rotation.  We want to rotate all
    % points in a right handed rotation, so we must transpose
  R = R';

  % rotate vertices
  pts = R*pts;
  
end
% end rotateVert

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function pts = translate(pts,pn,pe,pd)

  pts = pts + repmat([pn;pe;pd],1,size(pts,2));
  
end

% end translate


%=======================================================================
% defineVehicleBody
%=======================================================================
function [V,F,facecolors] = defineVehicleBody

% Variables from Figure 2.14 pg 26
fuse_l1 = 1;
fuse_l2 = 0.5;
fuse_l3 = 4;
fuse_w = 0.75;
fuse_h = 0.75;
wing_l = 1.5;
wing_w = 3;
tailwing_l = 0.5;
tailwing_w = 2;
tail_h = 0.75;
scale_factor = 10;

% Define the vertices (physical location of vertices
V = scale_factor*[...
    fuse_l1, 0, 0;...   % pt 1
    fuse_l2, fuse_w/2, -fuse_h/2;... % pt 2
    fuse_l2, -fuse_w/2, -fuse_h/2;...   % pt 3
    fuse_l2, -fuse_w/2, fuse_h/2;...  % pt 4
    fuse_l2, fuse_w/2, fuse_h/2;...  % pt 5
    -fuse_l3, 0, 0;...   % pt 6
    0, wing_w/2, 0;... % pt 7
    -wing_l, wing_w/2, 0;...   % pt 8
    -wing_l, -wing_w/2, 0;...  % pt 9
    0, -wing_w/2, 0;...  % pt 10
    (-fuse_l3+tailwing_l), tailwing_w/2, 0;...   % pt 11
    -fuse_l3, tailwing_w/2, 0;...   % pt 12
    -fuse_l3, -tailwing_w/2, 0;... % pt 13
    (-fuse_l3+tailwing_l), -tailwing_w/2, 0;...  % pt 14
    (-fuse_l3+tailwing_l), 0, 0;...  % pt 15
    -fuse_l3, 0, -tail_h;...  % pt 16
    ]';

% define faces as a list of vertices numbered above
  F = [...
        1, 2, 3;...  % nose top
        1, 3, 4;...  % nose left
        1, 4, 5;...  % nose right
        1, 5, 2;...  % nose bottom
        
        2, 3, 6;...  % body top
        4, 3, 6;...  % body left
        2, 5, 6;...  % body right
        5, 4, 6;...  % body bottom
        
        7, 8, 9;... % wing1
        7, 9, 10;... % wing2
        
        11, 12, 13;... % tailwing1
        11, 13, 14;... % tailwing2
    
        6, 15, 16;...  % tail 
        ];

% define colors for each face    
  myred = [1, 0, 0];
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myyellow = [1, 1, 0];
  mycyan = [0, 1, 1];

  facecolors = [...
    
    myred;...      % nose
    myred;...      % nose
    myred;...      % nose
    myred;...      % nose
    
    myblue;...     % body
    myblue;...     % body
    myblue;...     % body
    myblue;...     % body
    
    mygreen;...    % wing
    mygreen;...    % wing
    
    myyellow;...    % tailwing
    myyellow;...    % tailwing
    
    mycyan;...     % tail
    ];
end
  