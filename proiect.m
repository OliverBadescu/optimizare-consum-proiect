% Definirea grafului traseului
num_nodes = 6;

% Matricea de adiacenta (costurile între noduri, aici putem pune distante initiale)
graph = [0 10 0 30 100 0;
         10 0 50 0 0 0;
         0 50 0 20 10 60;
         30 0 20 0 60 0;
         100 0 10 60 0 10;
         0 0 60 0 10 0];

% Algoritmul Dijkstra pentru cel mai scurt drum
function [dist, path] = dijkstra(graph, start_node, end_node)
    num_nodes = size(graph, 1);
    visited = false(1, num_nodes);
    dist = inf(1, num_nodes);
    prev = NaN(1, num_nodes);
    dist(start_node) = 0;
    
    for i = 1:num_nodes
        [~, u] = min(dist + visited * inf);
        visited(u) = true;
        
        for v = 1:num_nodes
            if graph(u, v) > 0 && ~visited(v)
                alt = dist(u) + graph(u, v);
                if alt < dist(v)
                    dist(v) = alt;
                    prev(v) = u;
                end
            end
        end
    end
    
    path = [];
    u = end_node;
    while ~isnan(prev(u))
        path = [u, path];
        u = prev(u);
    end
    path = [start_node, path];
end

% Testare algoritm
start_node = 1;
end_node = 6;
[dist, path] = dijkstra(graph, start_node, end_node);

% Afișare rezultate
fprintf('Distanta minima: %d\n', dist(end_node));
fprintf('Traseu optim: ');
fprintf('%d ', path);
fprintf('\n');
