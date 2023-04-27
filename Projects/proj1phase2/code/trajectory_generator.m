function s_des = trajectory_generator(t, path, h)

    persistent time_stamp;
    persistent time_stamp_m;
    persistent time_interval;
    persistent P;
    persistent des_positions;

    if nargin > 1 % pre-process can be done here (given waypoints)

        N = 8; % Minimum snap
        total_time = 25.0; % total time for the trajectory
        waypoints_no = length(path); % number of waypoints
        segments_no = waypoints_no - 1; % number of segments

    %% [BEGIN]  Time allocation eqully distributed (choose to use 1/2)
        % time_interval = zeros(1,segments_no)+total_time/segments_no;
        % time_stamp_m = [0, cumsum(time_interval)];
    %% [END]    Time allocation in equal (choose to use 1/2)

    %% [BEGIN]  Time allocation by distance (choose to use 2/2)
        seg_length = sqrt(sum((path(2:end, :) - path(1:end-1,:)).^2,2)); % matrix containing each segment length (vertical)
        cum_length = cumsum(seg_length); % cumulative length at each waypoints (vertical)
        time_stamp = (cum_length/cum_length(end))*total_time; % time stamp at each waypoints in terms of total time
        time_stamp_m = [0; time_stamp]'; % middle element for calculating time interval (horizontal)
        time_interval = time_stamp_m(2:end) - time_stamp_m(1:end-1); % matrix containing each segment time interval (horizontal)
    %% [end]    Time allocation by distance (choose to use 2/2)

        %% Cost function (Lecture 4, P16) Minimize P^T_k * Q_k * P_k
        Q_k = zeros(N*segments_no, N*segments_no); % Q_k£ºthe block diagonal matrix. Minimum snap, 8 coefficients.
        for k=1:segments_no % k here is the segement number !!!!!!!!!!!!
            for i=4:N
                for j=4:N
                        Q_k((k-1)*8+i,(k-1)*8+j)=i*(i-1)*(i-2)*(i-3)*j*(j-1)*(j-2)*(j-3)/(j+i-7)*(time_interval(k)^(i+j-7));
                end
            end
        end
    
        %% Derivative constraint (Lecture 4, P17) A_j * P_j = d_j
        dim = 2*segments_no+4*(segments_no-1)+ 3*2; % continutity[4]+starting[3]+ending[3]
        P = zeros(N*segments_no,3); % 8 coefficients for each segment, 3 dimensions
        A = zeros(dim,N*segments_no); % infer from the equation
        d = zeros(dim,3); % 3 columns for x,y,z

        d(1:2*segments_no,:) =  reshape([path(1:end-1, :) path(2:end, :)]',3, [])';
        for j=1:segments_no % position
            A(2*j-1:2*j,N*j-7:N*j) = [ ...
                    1, 0, 0, 0, 0, 0, 0, 0;...
                    1, time_interval(j), time_interval(j)^2, time_interval(j)^3, ...
                    time_interval(j)^4, time_interval(j)^5, time_interval(j)^6, time_interval(j)^7
                ];
        end

        %% Continuity constraint (Lecture 4, P18)
        for j=1:(segments_no-1)
            A((2*segments_no+4*j-3):(2*segments_no+4*j),(8*j-7):(8*j+8)) = [...
                    1, time_interval(j), time_interval(j)^2, time_interval(j)^3, ...
                    time_interval(j)^4, time_interval(j)^5, time_interval(j)^6, time_interval(j)^7,-1,0,0,0,0,0,0,0;... % position
                    0, 1, 2*time_interval(j), 3*time_interval(j)^2, 4*time_interval(j)^3, 5*time_interval(j)^4, ...
                    6*time_interval(j)^5, 7*time_interval(j)^6,0,-1,0,0,0,0,0,0;... % velocity
                    0, 0, 2, 6*time_interval(j), 12*time_interval(j)^2, 20*time_interval(j)^3, ...
                    30*time_interval(j)^4, 42*time_interval(j)^5,0,0,-2,0,0,0,0,0;... % acceleration
                    0, 0, 0, 6, 24*time_interval(j), 60*time_interval(j)^2, ...
                    120*time_interval(j)^3, 210*time_interval(j)^4,0,0,0,-6,0,0,0,0 % jerk
                ];
        end

        %% Starting point vel, acc, jerk
        A(6*segments_no-3:6*segments_no-1, 1:N) = [ ...
                0,1,0,0,0,0,0,0;...
                0,0,2,0,0,0,0,0;...
                0,0,0,6,0,0,0,0
            ];
        A(6*segments_no:6*segments_no+2, N*segments_no-7:N*segments_no) = [ ...
                0, 1, 2*time_interval(segments_no), 3*time_interval(segments_no)^2, 4*time_interval(segments_no)^3, 5*time_interval(segments_no)^4, ...
                6*time_interval(segments_no)^5, 7*time_interval(segments_no)^6; ...
                0, 0, 2, 6*time_interval(segments_no), 12*time_interval(segments_no)^2, 20*time_interval(segments_no)^3, ...
                30*time_interval(segments_no)^4, 42*time_interval(segments_no)^5; ...
                0, 0, 0, 6, 24*time_interval(segments_no), 60*time_interval(segments_no)^2, ...
                120*time_interval(segments_no)^3, 210*time_interval(segments_no)^4
            ];
    
        %% Quadratic programming (QP) (Lecture 4, P19)
        % x = quadprog(H,f,A,b,Aeq,beq) solves the preceding problem subject to the additional restrictions Aeq * x = beq. Aeq is a matrix of doubles, and beq is a vector of doubles. If no inequalities exist, set A = [] and b = [].
        f=zeros(8*segments_no,1);
        P(:,1) = quadprog(Q_k,f,[],[],A,d(:,1)); % solve for x
        P(:,2) = quadprog(Q_k,f,[],[],A,d(:,2)); % solve for y
        P(:,3) = quadprog(Q_k,f,[],[],A,d(:,3)); % solve for z

    else % output desired trajectory here (given time)
    
        % s_des(1:3) desire position
        % s_des(4:6) desire velocity
        % s_des(7:9) desire acceleration
        % s_des(10) desire yaw
        % s_des(11) desire yaw rate
    
        %% find t
        s_des = zeros(13,1);
        if t >time_stamp_m(end) %hover at the end position
            s_des(1:3)= des_positions;
            s_des(7:10) =[1;0;0;0];
            return;
        end
        ind = 1;
        time = 0;
        for i = 1:length(time_interval)
            if t>=time_stamp_m(i) && t<=time_stamp_m(i+1)
                ind = i;
                time = time_stamp_m(i);
                break
            end
        end
        T = t - time;
        p = P(8*ind-7:8*ind,:);
        s_des(1:3) = [1,T,T^2,T^3,T^4,T^5,T^6,T^7]*p;
        des_positions = s_des(1:3);
        s_des(4:6) = [0,1,2*T,3*T^2,4*T^3,5*T^4,6*T^5,7*T^6]*p;
        s_des(7:10) =[1;0;0;0];
    
    end
    
    end
    
    
    