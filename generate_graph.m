%% First define the 3D grid and obstacles
grid = zeros(30,30,30);
grid(5:26,5:26,5:26) = 1; % This is the obstacle

%% Generate graph based on the grid with obstacles
list_node_ids = find(grid == 0);
num_nodes = size(list_node_ids,1);
[list_nodes_x, list_nodes_y, list_nodes_z] = ind2sub(size(grid), list_node_ids);
list_nodes = [list_nodes_x, list_nodes_y, list_nodes_z];
adj_matrix = zeros(num_nodes, num_nodes);

% List of possible directions and deltas
dir = [1 -1];
delta_list = {[1,0,0],[0,1,0],[0,0,1],[1,1,0],[1,-1,0],...
    [1,0,1],[1,0,-1],[0,1,1],[0,1,-1],[1,1,1],[1,1,-1],[1,-1,1],[-1,1,1]};
for n = 1:num_nodes
    curr_node = list_nodes(n, :);
    cnt = 1;
    for i = 1:max(size(delta_list))
        for j = 1:max(size(dir))
            temp_node = curr_node + dir(j)*delta_list{1,i};
            if(min(temp_node) > 0 && max(temp_node) <= max(size(grid)))
                temp_node_id = sub2ind(size(grid),temp_node(1),temp_node(2),temp_node(3));
                temp_result = find(list_node_ids == temp_node_id);
                if temp_result
                    adj_matrix(n, temp_result) = 1;
                end
            end
        end
    end
end

G = graph(adj_matrix) % Init graph

%% Create and plot shortest path
start_node = [1,1,1];
target_node = [30,30,30];
start_node_id = sub2ind(size(grid),start_node(1), start_node(2), start_node(3));
target_node_id = sub2ind(size(grid),target_node(1), target_node(2), target_node(3));

P = shortestpath(G,find(list_node_ids == start_node_id),find(list_node_ids == target_node_id));
[path_x, path_y, path_z] = ind2sub(size(grid),list_node_ids(P));

plot3(path_x, path_y, path_z);
xlim([1 size(grid,1)]); ylim([1 size(grid,2)]); zlim([1 size(grid,3)]);
