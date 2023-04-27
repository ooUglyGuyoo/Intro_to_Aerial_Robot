function s_des = heart_trajectory(t, true_s)
    s_des = zeros(11,1);

%% TODO: Implement heart trajectory here
% A ¡±heart¡± with corners at (0, 0, 0), (0, -1, -1), (0, -0.5, 1.5), (0, 0, 1)£¬(0£¬1.5£¬1.5)£¬(0, 1, 1) when projected into the yz plane, and x coordinate starting at 0 and ending at 4. The quadrotor should start at (0, 0, 0) and finish two circulations within 25s. You should implement this trajectory.

    t_interval = 25/12;
    interval_no = fix(t/t_interval)+1; % start from 1, end at 12
    inner_t = 0;

    x_des = 4/25*t;
    x_vdes = 4/25;
    x_ades = 0;

    if interval_no == 1 | interval_no == 7 % y(0,-1) z(0,1)
        inner_t = t - (interval_no-1)*t_interval;
        y_des = 0 + (-1)/t_interval*inner_t;
        y_vdes = (-1)/t_interval;
        y_ades = 0;
        z_des = 0 + (1)/t_interval*inner_t;
        z_vdes = (1)/t_interval;
        z_ades = 0;

    elseif interval_no == 2 | interval_no == 8 % y(-1,-0.5) z(1,1.5)
        inner_t = t - (interval_no-1)*t_interval;
        y_des = -1 + (0.5)/t_interval*inner_t;
        y_vdes = (0.5)/t_interval;
        y_ades = 0;
        z_des = 1 + 0.5/t_interval*inner_t;
        z_vdes = 0.5/t_interval;
        z_ades = 0;

    elseif interval_no == 3 | interval_no == 9 % y(-0.5,0) z(1.5,1)
        inner_t = t - (interval_no-1)*t_interval;
        y_des = -0.5 + (0.5)/t_interval*inner_t;
        y_vdes = (0.5)/t_interval;
        y_ades = 0;
        z_des = 1.5 - (0.5)/t_interval*inner_t;
        z_vdes = - (0.5)/t_interval;
        z_ades = 0;

    elseif interval_no == 4 | interval_no == 10 % y(0,0.5) z(1,1.5)
        inner_t = t - (interval_no-1)*t_interval;
        y_des = 0 + (0.5)/t_interval*inner_t;
        y_vdes = (0.5)/t_interval;
        y_ades = 0;
        z_des = 1 + (0.5)/t_interval*inner_t;
        z_vdes = (0.5)/t_interval;
        z_ades = 0;

    elseif interval_no == 5 | interval_no == 11 % y(0.5,1) z(1.5,1)
        inner_t = t - (interval_no-1)*t_interval;
        y_des = 0.5 + (0.5)/t_interval*inner_t;
        y_vdes = + (0.5)/t_interval;
        y_ades = 0;
        z_des = 1.5 - (0.5)/t_interval*inner_t;
        z_vdes = - (0.5)/t_interval;
        z_ades = 0;

    elseif interval_no == 6 | interval_no == 12 % y(1,0) z(1,0)
        inner_t = t - (interval_no-1)*t_interval;
        y_des = 1 - (1)/t_interval*inner_t;
        y_vdes = - (1)/t_interval;
        y_ades = 0;
        z_des = 1 - (1)/t_interval*inner_t;
        z_vdes = - (1)/t_interval;
        z_ades = 0;

    else
        disp('error, hover');
        x_des = true_s(1);
        y_des = true_s(2);
        z_des = true_s(3);
        x_vdes = 0;
        x_ades = 0;
        y_vdes = 0;
        y_ades = 0;
        z_vdes = 0;
        z_ades = 0;
    end

    %% Given yaw, DO NOT CHANGE
    yaw_des = mod(0.2 * pi * t,2 * pi);
    dyaw_des = 0.2 * pi;

    s_des(1)=x_des;
    s_des(2)=y_des;
    s_des(3)=z_des;
    s_des(4)=x_vdes;
    s_des(5)=y_vdes;
    s_des(6)=z_vdes;
    s_des(7)=x_ades;
    s_des(8)=y_ades;
    s_des(9)=z_ades;
    s_des(10)=yaw_des;
    s_des(11)=dyaw_des;

end
