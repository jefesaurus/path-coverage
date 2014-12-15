function rrt_bugtrap

bugtrap = [ .35 .8; .49 .65; .49 .60; .35 .75; .25 .75; .25 .25; ...
  .75 .25; .75 .75; .65 .75; .51 .60; .51 .65; .65 .8; .8 .8; .8 .2; ...
  .2 .2; .2 .8; .35 .8]';

%figure(1); clf; hold on;
%patch(bugtrap(1,:),bugtrap(2,:),'r');
%axis equal; 
%axis([0 1 0 1]);
x_goal = [.1;.1];
x_start = [.3;.3];
%plot(x_start(1),x_start(2),'bx',x_goal(1),x_goal(2),'gx','MarkerSize',20,'LineWidth',3);

%prob = MotionPlanningProblem(2);
%prob = addConstraint(prob,FunctionHandleConstraint(0,0,2,@(x)inpolygon(x(1),x(2),bugtrap(1,:),bugtrap(2,:)),-2));

%options.figure_num = gcf;
%options.max_edge_length = .1;
%options.max_length_between_constraint_checks = .01;
%xtraj = do_rrt(prob, x_start,x_goal,@uniformSamples,options);

%fnplt(xtraj);
alg1(x_start, x_goal)
end



function alg1(start, goal)
    % The forest is:
    % (root index, parent indices, vertex data, best path indices)
    forest = {1, [], [start], []};
    
    % choose random tree/guard out of the forest.
    selected_guard = randi(size(forest, 1));
    forest(selected_guard, :)
    
    
    
    %guards = [];
    %forest = [];
    %X = sprintf('start: (%d, %d), goal: (%d, %d)', start(1), start(2), goal(1), goal(2));
    %disp(X);
end


function x_new = ExtendTree(root_index, parents, vertices, sample_function)
    xnew_node = sample_function();
    %nearest_node_ind = problem.nearest(new_node);
    %new_node = problem.steer(nearest_node_ind, new_node);   % if new node is very distant from the nearest node we go from the nearest node in the direction of a new node
    %if(~problem.obstacle_collision(new_node, nearest_node_ind))
    %    neighbors = problem.neighbors(new_node, nearest_node_ind);
    %    min_node_ind = problem.chooseParent(neighbors, nearest_node_ind, new_node);
    %    new_node_ind = problem.insert_node(min_node_ind, new_node);
    %    problem.rewire(new_node_ind, neighbors, min_node_ind);
    %end
    
    %if(mod(ind, 1000) == 0)
    %    disp([num2str(ind) ' iterations ' num2str(problem.nodes_added-1) ' nodes in ' num2str(toc) ' rewired ' num2str(problem.num_rewired)]);
    %end
end

function xs = uniformSamples
xs = rand(2,1);
end