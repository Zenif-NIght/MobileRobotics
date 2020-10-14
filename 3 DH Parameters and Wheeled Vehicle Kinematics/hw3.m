
% prob1()
prob3_3()


function [] = prob1()

    % Initial rotation matrix
    rotx = @(t) [1 0 0; 0 cos(t) -sin(t) ; 0 sin(t) cos(t)] ;
    roty = @(t) [cos(t) 0 sin(t) ; 0 1 0 ; -sin(t) 0  cos(t)] ;
    rotz = @(t) [cos(t) -sin(t) 0 ; sin(t) cos(t) 0 ; 0 0 1] ;
    rot3D =@(a,b,y) rotz(a)*roty(b)* rotx(y);
    Initial_rotation_matrix =   rot3D(pi/4,pi/4,pi/4)




    skew_symmetric_matrix =@(v) [0 -v(3) v(2);
                                v(3) 0 -v(1) ;
                                -v(2) v(1) 0;];

    v =[1 1 1]/sqrt(3);
    vhat = skew_symmetric_matrix(v) * pi/4

    % skew_sym_matrix = 0.5* (R- transpose( R))

    R_m =  expm(vhat);
    R_m


    %% Calculated quaternion
    quat = get_quaternion(Initial_rotation_matrix)
    quat2 = get_quaternion(R_m)


    %%  Axis of rotation
    % quat
    axis_mag = sqrt(sum(quat(2:4).^2));
    axis = quat(2:4)/axis_mag

    theta = 2* atan2(axis_mag,quat(1))


    % quat2
    axis_mag2 = sqrt(sum(quat2(2:4).^2));
    axis2 = quat2(2:4)/axis_mag

    theta2 = 2* atan2(axis_mag2,quat2(1))


    %% Exponential Coordinates
    w  = (skew_symmetric_matrix(axis)*pi/4)
    w2  = (skew_symmetric_matrix(axis2)*pi/4)

    %% Resulting rotation matrix
    R_new =expm(w)
    R_new2 =expm(w2)


    %%
    function [quat] = get_quaternion(R_mat)
        qw = sqrt(1 + trace(R_mat)) /2;
        quat =-1;
        if(qw ==0)
        % to be determined....
            return
        else
            quat = [qw
                (R_mat(3,2) - R_mat(2,3))/( 4 *qw)
                (R_mat(1,3) - R_mat(3,1))/( 4 *qw)
                (R_mat(2,1) - R_mat(1,2))/( 4 *qw)
                ];
        end
    end



end




