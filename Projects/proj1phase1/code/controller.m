function [F, M] = controller(t, s, s_des)

% F is the total thrust
% M is the total moment
% t is the current time
% s is the current state of the quadrotor
% s_des is the desired state of the quadrotor

global params

m = params.mass;
g = params.grav;
I = params.I;

% disp('s=');
% disp(s);
% disp('s_des=');
% disp(s_des);

% s(1:3) current position
% s(4:6) current velocity
% s(7:10) current attitude quaternion
% s(11:13) current body angular velocity

% s_des(1:3) desire position
% s_des(4:6) desire velocity
% s_des(7:9) desire acceleration
% s_des(10) desire yaw
% s_des(11) desire yaw rate

F = 0.0; M = [0.0, 0.0, 0.0]'; % You should calculate the output F and M

%% PID parameters
kp_position = [8; 10; 18]; % 8 8 18
kd_position = [4; 6; 9]; % 4 4 9
% ki_position = [0; 0; 0];
kp_rotation = [2000; 2000; 200]; % 2000 2000 2000
kd_rotation = [70; 70; 70]; % 70 70 70
% ki_rotation = [0; 0; 0];

%% Angular velocity
w = s(11:13);
psi_dot_c = s_des(11);

% disp('w=');
% disp(w);
%% rotational matrix, euler angles
rotational_matrix = quaternion_to_R(s(7:10));
[phi,theta,psi] = RotToRPY_ZXY(rotational_matrix);

% disp('rotational_matrix=');
% disp(rotational_matrix);

euler = [phi; theta; psi];

% disp('euler=');
% disp(euler);

%% Position control (Lecture 3, P46)
%%% PID
p_dd_1_c = s_des(7) + kd_position(1)*(s_des(4)-s(4)) + kp_position(1)*(s_des(1)-s(1));
p_dd_2_c = s_des(8)  + kd_position(2)*(s_des(5)-s(5)) + kp_position(2)*(s_des(2)-s(2));
p_dd_3_c = s_des(9)  + kd_position(3)*(s_des(6)-s(6)) + kp_position(3)*(s_des(3)-s(3));
%%% Model
F       = m*(g + p_dd_3_c);
phi_c   = (1/g)*(p_dd_1_c*sin(psi) - p_dd_2_c*cos(psi));
theta_c = (1/g)*(p_dd_1_c*cos(psi) + p_dd_2_c*sin(psi));
psi_c = s_des(10);
psi_dot_c = s_des(11);

%% Attitude control (Lecture 3, P46)
%%% Discontinuous point of the Euler angle
psi_diff = psi_c-psi;
if psi_diff >= pi
    psi_diff=psi_diff-2*pi;
elseif psi_diff <= -pi
    psi_diff=psi_diff+2*pi;
end

%%% PID
euler_dd_c = [
    kp_rotation(1)*(phi_c-phi) + kd_rotation(1)*(0-w(1));
    kp_rotation(2)*(theta_c-theta) + kd_rotation(2)*(0-w(2));
    kp_rotation(3)*(psi_diff) + kd_rotation(3)*(psi_dot_c-w(3));
    ];

%%% Model
M = I*euler_dd_c + cross(w,I*w);

% disp('F=')
% disp(F);

% disp('M=')
% disp(M);

% pause(1000);

%% Calculate RMS error
persistent RMS_x_position RMS_x_position_des RMS_x_velocity RMS_x_velocity_des
persistent RMS_y_position RMS_y_position_des RMS_y_velocity RMS_y_velocity_des
persistent RMS_z_position RMS_z_position_des RMS_z_velocity RMS_z_velocity_des
persistent RMS_phi RMS_phi_des RMS_theta RMS_theta_des RMS_psi_diff

RMS_x_position = [RMS_x_position, s(1)];
RMS_x_position_des = [RMS_x_position_des, s_des(1)];
RMS_x_velocity = [RMS_x_velocity, s(4)];
RMS_x_velocity_des = [RMS_x_velocity_des, s_des(4)];
RMS_y_position = [RMS_y_position, s(2)];
RMS_y_position_des = [RMS_y_position_des, s_des(2)];
RMS_y_velocity = [RMS_y_velocity, s(5)];
RMS_y_velocity_des = [RMS_y_velocity_des, s_des(5)];
RMS_z_position = [RMS_z_position, s(3)];
RMS_z_position_des = [RMS_z_position_des, s_des(3)];
RMS_z_velocity = [RMS_z_velocity, s(6)];
RMS_z_velocity_des = [RMS_z_velocity_des, s_des(6)];
RMS_phi = [RMS_phi, phi];
RMS_phi_des = [RMS_phi_des, phi_c];
RMS_theta = [RMS_theta, theta];
RMS_theta_des = [RMS_theta_des, theta_c];
RMS_psi_diff = [RMS_psi_diff, psi_diff];

RMS_error_x_position = sqrt(sum((RMS_x_position - RMS_x_position_des).^2)/length(RMS_x_position));
RMS_error_x_velocity = sqrt(sum((RMS_x_velocity - RMS_x_velocity_des).^2)/length(RMS_x_velocity));
RMS_error_y_position = sqrt(sum((RMS_y_position - RMS_y_position_des).^2)/length(RMS_y_position));
RMS_error_y_velocity = sqrt(sum((RMS_y_velocity - RMS_y_velocity_des).^2)/length(RMS_y_velocity));
RMS_error_z_position = sqrt(sum((RMS_z_position - RMS_z_position_des).^2)/length(RMS_z_position));
RMS_error_z_velocity = sqrt(sum((RMS_z_velocity - RMS_z_velocity_des).^2)/length(RMS_z_velocity));
RMS_error_phi = sqrt(sum((RMS_phi - RMS_phi_des).^2)/length(RMS_phi));
RMS_error_theta = sqrt(sum((RMS_theta - RMS_theta_des).^2)/length(RMS_theta));
RMS_error_psi = sqrt(sum((RMS_psi_diff).^2)/length(RMS_psi_diff));

disp('RMS_error_x_position=');
disp(RMS_error_x_position);
disp('RMS_error_x_velocity=');
disp(RMS_error_x_velocity);
disp('RMS_error_y_position=');
disp(RMS_error_y_position);
disp('RMS_error_y_velocity=');
disp(RMS_error_y_velocity);
disp('RMS_error_z_position=');
disp(RMS_error_z_position);
disp('RMS_error_z_velocity=');
disp(RMS_error_z_velocity);
disp('RMS_error_phi=');
disp(RMS_error_phi);
disp('RMS_error_theta=');
disp(RMS_error_theta);
disp('RMS_error_psi=');
disp(RMS_error_psi);
disp('______________________________________________________')

end
