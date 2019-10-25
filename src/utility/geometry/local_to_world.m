function P_out = local_to_world(P_robot, P_local, set_dx, set_dy, Dx, Dy)
% P_out = local_to_world(P_robot, P_local, set_dx, set_dy, Dx, Dy)
%
% Given a position and heading of the robot in its local frame, positions
% in the local frame as xy points, and an offset of the robot in its local
% frame (set_dx, set_dy), transform the points from the local frame to the
% world frame and scale them up by the distances Dx and Dy; note that if Dy
% is not passed in, we set Dy = Dx.
%
% INPUTS
%   P_robot     robot position (x,y,heading)
%   P_world     obstacle points in local frame (2xN)
%   set_dx      x position of robot in its local frame
%   set_dy      y position of robot in its local frame
%   Dx          scaling in the x dimension
%   Dy          scaling in the y dimension (equal to Dx by default)
%
% OUTPUTS:
%   P_out       points in world frame

    if nargin < 3
        set_dx = 0 ;
        set_dy = 0 ;
        Dx = 1 ;
        Dy = 1 ;
    end

    if ~exist('Dy','var')
        Dy = Dx ;
    end


    % extract position and heading from input
    x = P_robot(1,1) ;
    y = P_robot(2,1) ;
    h = P_robot(3,1) ;

    % get matrix to scale the points
    N = size(P_local,2) ;
    scale_mat = repmat([Dx;Dy],1,N) ;

    % get matrix to shift the points in the local frame and global frame
    shift_mat_local = (1./scale_mat) .* repmat([set_dx;set_dy],1,N) ;
    shift_mat_global = repmat([x;y],1,N) ;

    % get matrix to rotate points
    R = [cos(h), -sin(h);
        sin(h), cos(h)];

    % create the output
    P_local = P_local - shift_mat_local ;
    P_local = R*P_local ;
    P_out = (scale_mat).*P_local + shift_mat_global ;
end