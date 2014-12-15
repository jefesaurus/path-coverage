function rrt_star_test

    BugTrapTestMulti();
    %BugTrapTest();

%fnplt(xtraj);


end 

function BugTrapTest()
    bugtrap = [ .35 .8; .49 .65; .49 .60; .35 .75; .25 .75; .25 .25; ...
    .75 .25; .75 .75; .65 .75; .51 .60; .51 .65; .65 .8; .8 .8; .8 .2; ...
    .2 .2; .2 .8; .35 .8]';

    figure(1); clf; hold on;
    patch(bugtrap(1,:),bugtrap(2,:),'r');
    axis equal; 
    axis([0 1 0 1]);
    x_goal = [.1;.1];
    x_start = [.3;.3];
    plot(x_start(1),x_start(2),'bx',x_goal(1),x_goal(2),'gx','MarkerSize',20,'LineWidth',3);

    prob = MotionPlanningProblem(2);
    prob = addConstraint(prob,FunctionHandleConstraint(0,0,2,@(x)inpolygon(x(1),x(2),bugtrap(1,:),bugtrap(2,:)),-2));
    
    conf = struct;
    conf.delta_goal_point = 1;          % Radius of goal point
    conf.delta_near = 1.5;              % Radius for neighboring nodes
    conf.max_step = 0.5;                % Maximum position change when we add a new node to the tree
    conf.max_dist_between_constraint_checks = .01;
    conf.bin_size = .05;
    conf.distance_metric_fcn = @MotionPlanningProblem.euclideanDistance;
    conf.display_fnc = @MotionPlanningProblem.drawFirstTwoCoordinates;
    
    MAX_ITER = 2000;

    %rrt_data = FNSimple2D(5, MAX_ITER, x_start, x_goal, prob, conf);
    guards = [FNSimple2D(5, MAX_ITER, x_start, x_goal, prob, conf)];

    for ind = 1:MAX_ITER
        Extend(guards(1), .05, x_goal);
    end
    guards(1).plotstuff(0);
end

function BugTrapTestMulti()
    bugtrap = [ .35 .8; .49 .65; .49 .60; .35 .75; .25 .75; .25 .25; ...
    .75 .25; .75 .75; .65 .75; .51 .60; .51 .65; .65 .8; .8 .8; .8 .2; ...
    .2 .2; .2 .8; .35 .8]';
    figure(1); clf; hold on;
    patch(bugtrap(1,:),bugtrap(2,:),'r');
    axis equal; 
    axis([0 1 0 1]);
    x_goal = [.1;.1];
    x_start = [.3;.3];
    plot(x_start(1),x_start(2),'bx',x_goal(1),x_goal(2),'gx','MarkerSize',20,'LineWidth',3);

    prob = MotionPlanningProblem(2);
    prob = addConstraint(prob,FunctionHandleConstraint(0,0,2,@(x)inpolygon(x(1),x(2),bugtrap(1,:),bugtrap(2,:)),-2));
    
    conf = struct;
    conf.delta_goal_point = 1;          % Radius of goal point
    conf.delta_near = 1.5;              % Radius for neighboring nodes
    conf.max_step = 0.5;                % Maximum position change when we add a new node to the tree
    conf.max_dist_between_constraint_checks = .01;
    conf.bin_size = .05;
    conf.distance_metric_fcn = @MotionPlanningProblem.euclideanDistance;
    conf.display_fnc = @MotionPlanningProblem.drawFirstTwoCoordinates;
    
    MAX_ITER = 1000;

    guards = [FNSimple2D(5, MAX_ITER, x_start, x_goal, prob, conf)];
    num_guards = 1;
    for ind = 1:MAX_ITER
        ind
        np = Extend(guards(randi(numel(guards))), .05, x_goal, prob);
        if IsNewGuard(guards, np)
            new_guard = FNSimple2D(1, MAX_ITER, np, x_goal, prob, conf);
            plot(np(1),np(2),'rx','MarkerSize',20,'LineWidth',3);

            guards = [guards, new_guard];
            num_guards = num_guards + 1;
            disp('new guard');
        end
    end
    for i = 1:num_guards
        guards(i).plotstuff(0);
    end
end


function DoRRTStar(iters, rrt_data, goal_bias, x_goal)
    for ind = 1:iters
        Extend(rrt_data, goal_bias, x_goal);
    end
    rrt_data.plotstuff(0);
end

function is_new_guard = IsNewGuard(forest, new_node_position)
    is_new_guard = true;
    for i = 1:numel(forest)
        is_new_guard = ~forest(i).GuardCanSee(new_node_position);
        if ~is_new_guard
            return;
        end
    end
end

function new_node = Extend(rrt_data, goal_bias, x_goal, problem)
    while true
        if rand < goal_bias
            new_node = x_goal;
        else
            new_node = uniformSamples();
        end
        nearest_node_ind = rrt_data.nearest(new_node);
        new_node = rrt_data.steer(nearest_node_ind, new_node);
        if(checkConstraints(problem, new_node) & ~rrt_data.obstacle_collision(new_node, nearest_node_ind))
            neighbors = rrt_data.neighbors(new_node);
            min_node_ind = rrt_data.chooseParent(neighbors, nearest_node_ind, new_node);
            new_node_ind = rrt_data.insert_node(min_node_ind, new_node);
            rrt_data.rewire(new_node_ind, neighbors, min_node_ind);
            return
        end
    end
end


function xs = uniformSamples
xs = rand(2,1);
end