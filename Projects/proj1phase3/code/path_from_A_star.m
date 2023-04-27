function Optimal_path = path_from_A_star(map)
    Optimal_path = [];
    size_map = size(map,1);

    MAX_X=10;
    MAX_Y=10;
    MAX_Z=10;
    
    %Define the 3D grid map array.
    %Obstacle=-1, Target = 0, Start=1
    MAP=2*(ones(MAX_X,MAX_Y,MAX_Z));
    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1));
    yval=floor(map(size_map, 2));
    zval=floor(map(size_map, 3));
    
    xTarget=xval;
    yTarget=yval;
    zTarget=zval;
    MAP(xval,yval,zval)=0; % set the target point
    
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1
        xval=floor(map(i, 1));
        yval=floor(map(i, 2));
        zval=floor(map(i, 3));
        MAP(xval,yval,zval)=-1;
    end

    %Initialize MAP with location of the start point
    xval=floor(map(1, 1));
    yval=floor(map(1, 2));
    zval=floor(map(1, 3));
    xStart=xval;
    yStart=yval;
    zStart=zval;
    MAP(xval,yval,zval)=1; % set the start point

    % Main structure in the A* search =====================================================

    % Container storing nodes to be expanded, along with the f score (f=g+h)
    % Each node's (x,y,z) coordinate and its f score is stored in a row
    % For example, queue = [x1, y1, z1, f1; x2, y2, z2, f2; ...; xn, yn, zn, fn]
    % g is the cost
    % h is the heuristic
    queue = [];

    % Arrays for storing the g score of each node, g score of undiscovered nodes is inf
    g = inf(MAX_X,MAX_Y,MAX_Z); % store the cost

    % Arrays recording whether a node is expanded (popped from the queue) or not
    % expanded: 1, not expanded: 0
    expanded = zeros(MAX_X,MAX_Y,MAX_Z);

    % Arrays recording the parent of each node
    parents = zeros(MAX_X,MAX_Y,MAX_Z, 3); % Every node have one parent node

    %Start your code here ==================================================================
    % TODO

    queue = [xStart,yStart,zStart,abs(xStart-xTarget) + abs(yStart-yTarget) + abs(zStart-zTarget)]; % put the starting node inside the queue
    search_iteration = 0; % total number of node searched
    current_position = [xStart,yStart,zStart]; % x,y,z,f
    current_position_with_f = queue;
    g(current_position(1),current_position(2),current_position(3)) = 0;

    while ~isempty(queue)
        % set current position MAP and as expanded
        MAP(current_position(1),current_position(2),current_position(3)) = -1;
        expanded(current_position(1),current_position(2),current_position(3)) = 1;
        if current_position == [xTarget,yTarget,zTarget];
            break
        end
        % calculate the f = g + h of each neighbors
        current_neighbors_no_f = [  current_position(1)-1, current_position(2), current_position(3);...
                                    current_position(1)+1, current_position(2), current_position(3);...
                                    current_position(1), current_position(2)-1, current_position(3);...
                                    current_position(1), current_position(2)+1, current_position(3);...
                                    current_position(1), current_position(2), current_position(3)-1;...
                                    current_position(1), current_position(2), current_position(3)+1;...
                                    ]; % x,y,z,f
        for i = 1:size(current_neighbors_no_f,1)
            x = current_neighbors_no_f(i,1);
            y = current_neighbors_no_f(i,2);
            z = current_neighbors_no_f(i,3);
            if x > 0 && y > 0 && z > 0 && x <= MAX_X && y <= MAX_Y & z <= MAX_Z && MAP(x,y,z) ~= -1 
                current_n_g = g(current_position(1),current_position(2),current_position(3)) + 1; % cost of current neighbor
                % current_n_h = abs(x-xTarget)+abs(y-yTarget)+abs(z-zTarget); % heuristic of current neighbor: mahanlton distance
                current_n_h = norm([abs(x-xTarget),abs(y-yTarget),abs(z-zTarget)]); % heuristic of current neighbor: direct distance
                current_n_f = current_n_g + current_n_h;
                if g(x,y,z) ~= inf % store the g value
                    g(x,y,z) = current_n_g;
                end
                for i = 1:3 % store the parent node
                    parents(x,y,z,i) = current_position(i);
                end
                queue = [queue;x,y,z,current_n_f]; % put all neighbors inside the queue
            end
        end

        % delete current node from the queue
        current_position_queue_index = find(ismember(queue,current_position_with_f,'row'));
        queue(current_position_queue_index,:)= []

        % find the min f in queue and set
        [min_f,min_f_idx] = min(queue(:, 4));
        current_position_with_f = queue(min_f_idx,:)
        g_next = g(current_position(1),current_position(2),current_position(3));
        current_position = [current_position_with_f(1),current_position_with_f(2),current_position_with_f(3)];
        g(current_position(1),current_position(2),current_position(3)) = g_next;
        search_iteration = search_iteration+1;
    end
    
    while 1
        Optimal_path = [current_position(1)-0.5,current_position(2)-0.5,current_position(3)-0.5;Optimal_path];
        if current_position == [xStart,yStart,zStart]
            break;
        end
        x = current_position(1); y = current_position(2); z = current_position(3);
        current_position = [parents(x,y,z,1),parents(x,y,z,2),parents(x,y,z,3)];
    end

end
