function [xtraj,info] = do_rrt_star(obj,x_start,x_goal,random_sample_fcn,options)
      % Simple RRT algorithm
      % @param x_start
      % @param x_goal
      % @param tolerance scalar distance that you must achieve to the goal
      % @param random_sample_fcn function handle of the form
      %    xs = random_sample_fcn
      % which generates a nX-by-1 random sample
      % @option distance_metric_fcn function handle of the form
      %    d = distance_metric_fcn(X,xs)
      % where X is a nX-by-N list of points, and xs is an nX-by-1 sample.
      % d should be a 1-by-N list of distances @default euclideanDistance
      % @option display_fcn @default drawFirstTwoCoordinates
      % @option display_after_every draws the tree after this many points
      % are added
      % @option max_edge_length a distance by which to trim
      % @options max_length_between_constraint_checks
      % @options goal_bias @default P(try_goal)=.05

      % parameters
      sizecheck(x_start,[obj.num_vars,1]);
      sizecheck(x_goal,[obj.num_vars,1]);
      typecheck(random_sample_fcn,'function_handle');

      % options
      if nargin<5, options=struct(); end
      defaultOptions.distance_metric_fcn = @MotionPlanningProblem.euclideanDistance;
      defaultOptions.display_fcn = @MotionPlanningProblem.drawFirstTwoCoordinates;
      defaultOptions.display_after_every = 50;
      defaultOptions.max_edge_length = inf;  % because i don't have any sense of scale here
      defaultOptions.max_length_between_constraint_checks = inf;
      defaultOptions.goal_bias = .05;
      options = applyDefaults(options,defaultOptions);
      typecheck(options.distance_metric_fcn,'function_handle');

      N = 10000;  % for pre-allocating memory
      V = repmat(x_start,1,N);  % vertices
      parent = nan(1,N-1); % edges

      n=2;
      n_at_last_display=0;
      info=2; xtraj=[];  % default return values
      while n<=N
        try_goal = rand<options.goal_bias;
        if try_goal
          xs = x_goal;
        else
          xs = random_sample_fcn();
          if ~checkConstraints(obj,xs), continue; end
        end

        d = options.distance_metric_fcn(V(:,1:n-1),xs);
        [dmin,imin] = min(d);

        % steer towards the new point
        if dmin>options.max_edge_length
          % then linearly interpolate along that distance
          xs = V(:,imin)+(options.max_edge_length/dmin)*(xs - V(:,imin));
          dmin = options.max_edge_length;
          try_goal = false;
        end

        if dmin>options.max_length_between_constraint_checks
          % then linearly interpolate along that distance
          num_intermediate_samples = 2+ceil(dmin/options.max_length_between_constraint_checks);
          xss = linspacevec(V(:,imin),xs,num_intermediate_samples);
          valid = true;
          for i=2:num_intermediate_samples-1
            if ~checkConstraints(obj,xss(:,i)),
              valid=false;
              break;
            end
          end
          if ~valid, continue; end
        end

        % Insert the new point into the tree.
        V(:,n) = xs;
        parent(n-1) = imin;
        Near(V, n, xs, options.distance_metric_fcn)
        
        % neighbors = problem.neighbors(new_node, nearest_node_ind);
        % min_node_ind = problem.chooseParent(neighbors, nearest_node_ind, new_node);
        % new_node_ind = problem.insert_node(min_node_ind, new_node);
        % problem.rewire(new_node_ind, neighbors, min_node_ind);
        

        if (try_goal)  % if I get here, then I successfully connected to the goal
          path = n;
          while path(1)>1
            path = [parent(path(1)-1),path];
          end
          xtraj = PPTrajectory(foh(1:length(path),V(:,path)));
          info = 1;
        end

        if mod(n,options.display_after_every)==0 || try_goal
          options.display_fcn(V(:,1:n),parent(1:n-1),n_at_last_display);
          drawnow;
          n_at_last_display = n;
        end

        if try_goal, return; end
        n=n+1;
      end
end

function near_nodes = Near(vertices, num_vertices, node, distance_metric_fcn)
    ball_radius = 1.0*(log(num_vertices)/num_vertices)^(1/2);
    
    near_nodes = [];
    d = distance_metric_fcn(vertices(:,1:num_vertices),node);
    for i = 1:num_vertices
        if d(i) < ball_radius
            near_nodes = [near_nodes, i];
        end
    end
end

