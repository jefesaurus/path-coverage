classdef FNSimple2D < handle
    properties (SetAccess = private)
        tree                % Array stores position information of states
        parent              % Array stores relations of nodes
        children            % Number of children of each node
        free_nodes          % Indices of free nodes
        free_nodes_ind      % Last element in free_nodes
        cost                % Cost between 2 connected states
        cumcost             % Cost from the root of the tree to the given node
        goal_point          % Goal position
        delta_goal_point    % Radius of goal position region
        delta_near          % Radius of near neighbor nodes
        nodes_added         % Keeps count of added nodes
        max_step            % The length of the maximum step while adding the node
        max_dist_between_constraint_checks
        problem
        best_path_node      % The index of last node of the best path
        distance_fnc
        goal_reached
        max_nodes
        display_fnc
        
        %%% temporary variables
        compare_table
        index
        list
        num_rewired
    end
    methods
        % class constructor
        function this = FNSimple2D(rand_seed, max_nodes, start_point, goal_point, problem, conf)
            max_nodes = int32(max_nodes);
            this.max_nodes = max_nodes;
            rng(rand_seed);
            this.tree = zeros(2, max_nodes);
            this.parent = zeros(1, max_nodes);
            this.children = zeros(1, max_nodes);
            this.free_nodes = zeros(1, max_nodes);
            this.free_nodes_ind = 1;
            this.cost = zeros(1, max_nodes);
            this.cumcost = zeros(1,max_nodes);
            this.tree(:, 1) = start_point;
            this.goal_point = goal_point;
            this.delta_goal_point = conf.delta_goal_point;
            this.delta_near = conf.delta_near;
            this.nodes_added = uint32(1);
            this.max_step = conf.max_step;
            this.max_dist_between_constraint_checks = conf.max_dist_between_constraint_checks;
            this.problem = problem;
            this.best_path_node = -1;
            this.distance_fnc = conf.distance_metric_fcn;
            this.goal_reached = false;
            
            this.display_fnc = conf.display_fnc;
            
            %%% temp var-s initialization
            this.compare_table = zeros(1, max_nodes);
            this.index = zeros(1, max_nodes);
            this.list = 1:max_nodes;
            this.num_rewired = 0;
        end

        
        function node_index = nearest(this, new_node_position)
            this.compare_table(1:this.nodes_added) = this.distance_fnc(this.tree(:,1:this.nodes_added), new_node_position);
            [min_dist, node_index] = min(this.compare_table(1:this.nodes_added));
        end
        
        function position = steer(this, nearest_node, new_node_position)
            dmin = this.distance_fnc(this.tree(:, nearest_node), new_node_position);
            if dmin > this.max_step
                % then linearly interpolate along that distance
                position = this.tree(:,nearest_node)+(this.max_step/dmin)*(new_node_position - this.tree(:,nearest_node));
            else
                position = new_node_position;
            end
        end
        
        
        function collision = obstacle_collision(this, new_node_position, node_index)
            dist = norm(new_node_position - this.tree(:, node_index));
            num_intermediate_samples = 2 + ceil(dist/this.max_dist_between_constraint_checks);
            xss = linspacevec(this.tree(:,node_index), new_node_position, num_intermediate_samples);
            collision = false;
            for i=2:num_intermediate_samples-1
                if ~checkConstraints(this.problem, xss(:,i)),
                    collision=true;
                    return;
                end
            end
        end
        
        function new_node_ind = insert_node(this, parent_node_ind, new_node_position)
            % method insert new node in the tree
            this.nodes_added = this.nodes_added + 1;
            this.tree(:, this.nodes_added) = new_node_position;         % adding new node position to the tree
            this.parent(this.nodes_added) = parent_node_ind;            % adding information about parent-children information
            this.children(parent_node_ind) = this.children(parent_node_ind) + 1;
            this.cost(this.nodes_added) = norm(this.tree(:, parent_node_ind) - new_node_position);  % not that important
            this.cumcost(this.nodes_added) = this.cumcost(parent_node_ind) + this.cost(this.nodes_added);   % cummulative cost
            new_node_ind = this.nodes_added;
        end
        
        %%% RRT* specific functions
        
        function neighbor_nodes = neighbors(this, new_node_position)
            radius = 1.0 * (log(double(this.nodes_added))/double(this.nodes_added))^(1/2);
            this.compare_table(1:this.nodes_added) = this.distance_fnc(this.tree(:,1:this.nodes_added), new_node_position);
            this.compare_table(1:this.nodes_added) = this.compare_table(1:this.nodes_added) - radius;
            neighbor_nodes = [];
            for i = (1:this.nodes_added)
                if this.compare_table(i) < 0
                    neighbor_nodes = [neighbor_nodes, i];
                end
            end
        end
        
        function min_node_ind = chooseParent(this, neighbors, nearest_node, new_node_position)
            % finds the node with minimal cummulative cost node from the root of
            % the tree. i.e. find the cheapest path end node.
            min_node_ind = nearest_node;
            min_cumcost = this.cumcost(nearest_node) + norm(this.tree(:, nearest_node)- new_node_position);
            for ind=1:numel(neighbors)
                temp_cumcost = this.cumcost(neighbors(ind)) + norm(this.tree(:, neighbors(ind)) - new_node_position);
                if temp_cumcost < min_cumcost && ~this.obstacle_collision(new_node_position, neighbors(ind))
                    min_cumcost = temp_cumcost;
                    min_node_ind = neighbors(ind);
                end
            end
        end
        
        function rewire(this, new_node_ind, neighbors, min_node_ind)
            % method looks thru all neighbors(except min_node_ind) and
            % seeks and reconnects neighbors to the new node if it is
            % cheaper
            queue = zeros(1, int32(this.max_nodes/5));
            for ind = 1:numel(neighbors)
                % omit
                if (min_node_ind == neighbors(ind))
                    continue;
                end
                temp_cost = this.cumcost(new_node_ind) + norm(this.tree(:, neighbors(ind)) - this.tree(:, new_node_ind));
                if (temp_cost < this.cumcost(neighbors(ind)) && ...
                        ~this.obstacle_collision(this.tree(:, new_node_ind), neighbors(ind)))
                    
                    %this.cumcost(neighbors(ind)) = temp_cost;
                    this.children(this.parent(neighbors(ind))) = this.children(this.parent(neighbors(ind))) - 1;
                    this.parent(neighbors(ind)) = new_node_ind;
                    this.children(new_node_ind) = this.children(new_node_ind) + 1;
                    this.num_rewired = this.num_rewired + 1;
                    
                    bottom = 0;
                    top = 0;
                    bottom = bottom + 1;
                    queue(bottom) = neighbors(ind);
                    delta_cost = temp_cost - this.cumcost(neighbors(ind));
                    
                    while top < bottom
                        top = top+1;
                        cur = queue(top);
                        this.cumcost(cur) = this.cumcost(cur)+delta_cost;
                        kids = this.list(this.parent == cur);
                        for k_ind = 1:numel(kids)
                            bottom = bottom + 1;
                            queue(bottom) = kids(k_ind);
                        end
                    end           
                end
            end
        end
        
        function is_visible = GuardCanSee(this, new_node_position)
            %~checkConstraints(this.problem, new_node_position);
            is_visible = ~this.obstacle_collision(new_node_position, 1);
        end
       
        function plotstuff(this, starting_n)
            this.display_fnc(this.tree(:,1:this.nodes_added),this.parent(2:this.nodes_added),starting_n);
            drawnow;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%
        function plotpath(this)
            %%% Find the optimal path to the goal
            % finding all the point which are in the desired region
            this.compare_table(1:this.nodes_added) = this.distance_fnc(this.tree(:,1:this.nodes_added), this.goal_point);
            [min_dist, min_index] = min(this.compare_table(1:this.nodes_added));
            this.compare_table(1:this.nodes_added) = this.compare_table(1:this.nodes_added) + this.cumcost(1:this.nodes_added);
            if min_dist > this.delta_goal_point
                % can't get there yet
            end

            % backtracing the path
            current_index = min_index;
            path_iter = 1;
            backtrace_path = zeros(1,1);
            while(current_index ~= 1)
                backtrace_path(path_iter) = current_index;
                path_iter = path_iter + 1;
                current_index = this.parent(current_index);
            end
            backtrace_path(path_iter) = current_index;
            line([this.tree(1,backtrace_path(1:end -1));this.tree(1,backtrace_path(2:end))],[this.tree(2,backtrace_path(1:end -1));this.tree(2,backtrace_path(2:end))],'Color',0.3*[1 1 1]);
            drawnow;
        end
        
        function newObj = copyobj(thisObj)
            % Construct a new object based on a deep copy of the current
            % object of this class by copying properties over.
            props = properties(thisObj);
            for i = 1:length(props)
                % Use Dynamic Expressions to copy the required property.
                % For more info on usage of Dynamic Expressions, refer to
                % the section "Creating Field Names Dynamically" in:
                % web([docroot '/techdoc/matlab_prog/br04bw6-38.html#br1v5a9-1'])
                newObj.(props{i}) = thisObj.(props{i});
            end
        end
    end
    methods(Static)
        function plot_circle(x, y, r)
            t = 0:0.001:2*pi;
            cir_x = r*cos(t) + x;
            cir_y = r*sin(t) + y;
            plot(cir_x, cir_y, 'r-', 'LineWidth', 1.5);
        end
    end
end
