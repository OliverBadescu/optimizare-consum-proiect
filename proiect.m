% Vehicle Routing Optimization Project
% --------------------------------------------------------------
% Task 1: Definirea grafului traseului
num_nodes = 6;
% Matricea de adiacenta (costurile între noduri, aici putem pune distante initiale)
graph = [0 10 0 30 100 0;
         10 0 50 0 0 0;
         0 50 0 20 10 60;
         30 0 20 0 60 0;
         100 0 10 60 0 10;
         0 0 60 0 10 0];

% Task 2: Algoritmul Dijkstra pentru cel mai scurt drum
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

% Task 3: Structuri de date pentru reprezentarea traseului vehiculului
% ------------------------------------------------------------------

% Funcție pentru crearea unei structuri de date pentru vehicul
function vehicle = createVehicle(id, capacity, position)
    vehicle.id = id;
    vehicle.capacity = capacity;
    vehicle.position = position;
    vehicle.route = [];
    vehicle.load = 0;
    vehicle.totalDistance = 0;
end

% Funcție pentru crearea unei structuri de date pentru punct/nod
function node = createNode(id, x, y, demand, timeWindow)
    node.id = id;
    node.x = x;
    node.y = y;
    node.demand = demand;
    node.timeWindow = timeWindow; % [început, sfârșit]
    node.visited = false;
    node.type = 'client'; % poate fi 'depozit', 'client', 'stație_încărcare', etc.
end

% Funcție pentru crearea unei structuri de date pentru traseul complet
function routeData = createRouteData(nodes, vehicles, distanceMatrix)
    routeData.nodes = nodes;
    routeData.vehicles = vehicles;
    routeData.distanceMatrix = distanceMatrix;
    routeData.totalDistance = 0;
    routeData.totalTime = 0;
    routeData.unvisitedNodes = length(nodes) - 1; % excluzând depozitul
end

% Task 4: Algoritmi exacti și metaeuristici pentru optimizarea traseului
% ----------------------------------------------------------------------

% Algoritmul Nearest Neighbor (Greedy) pentru construcția inițială a traseului
function route = nearestNeighbor(distanceMatrix, start)
    n = size(distanceMatrix, 1);
    unvisited = true(1, n);
    unvisited(start) = false;
    route = start;
    current = start;
    
    while any(unvisited)
        minDist = inf;
        nextNode = -1;
        
        for i = 1:n
            if unvisited(i) && distanceMatrix(current, i) < minDist
                minDist = distanceMatrix(current, i);
                nextNode = i;
            end
        end
        
        if nextNode == -1
            break;
        end
        
        route = [route, nextNode];
        unvisited(nextNode) = false;
        current = nextNode;
    end
    
    % Adăugăm întoarcerea la depozit (start)
    route = [route, start];
end

% Algoritmul 2-Opt pentru îmbunătățirea traseului
function [improved_route, improved_dist] = twoOpt(route, distanceMatrix)
    n = length(route);
    improved = true;
    improved_route = route;
    improved_dist = calculateRouteDistance(improved_route, distanceMatrix);
    
    while improved
        improved = false;
        for i = 2:n-2
            for j = i+1:n-1
                new_route = improved_route;
                % Inversează ordinea nodurilor între i și j
                new_route(i:j) = improved_route(j:-1:i);
                
                new_dist = calculateRouteDistance(new_route, distanceMatrix);
                
                if new_dist < improved_dist
                    improved_dist = new_dist;
                    improved_route = new_route;
                    improved = true;
                    break;
                end
            end
            if improved
                break;
            end
        end
    end
end

% Calcularea distanței totale a unui traseu
function total_distance = calculateRouteDistance(route, distanceMatrix)
    total_distance = 0;
    for i = 1:length(route)-1
        total_distance = total_distance + distanceMatrix(route(i), route(i+1));
    end
end

% Algoritmul Simulated Annealing pentru optimizarea traseului
function [best_route, best_dist] = simulatedAnnealing(initial_route, distanceMatrix, initial_temp, cooling_rate, num_iterations)
    current_route = initial_route;
    current_dist = calculateRouteDistance(current_route, distanceMatrix);
    best_route = current_route;
    best_dist = current_dist;
    temp = initial_temp;
    
    for iter = 1:num_iterations
        % Generăm o perturbare aleatoare a traseului
        new_route = current_route;
        n = length(new_route);
        
        % Alegem două poziții aleatorii diferite (excluzând depozitul)
        i = randi([2, n-1]);
        j = randi([2, n-1]);
        while i == j
            j = randi([2, n-1]);
        end
        
        % Schimbăm nodurile de la pozițiile i și j
        temp_node = new_route(i);
        new_route(i) = new_route(j);
        new_route(j) = temp_node;
        
        new_dist = calculateRouteDistance(new_route, distanceMatrix);
        
        % Calculăm diferența dintre distanțe
        delta = new_dist - current_dist;
        
        % Decidem dacă acceptăm noua soluție
        if delta < 0 || rand < exp(-delta/temp)
            current_route = new_route;
            current_dist = new_dist;
            
            if current_dist < best_dist
                best_route = current_route;
                best_dist = current_dist;
            end
        end
        
        % Răcim temperatura
        temp = temp * cooling_rate;
    end
end

% Algoritmul Genetic pentru optimizarea traseului
function [best_route, best_fitness] = geneticAlgorithm(distanceMatrix, pop_size, num_generations, mutation_rate, start_depot)
    n = size(distanceMatrix, 1);
    
    % Inițializăm populația
    population = initializePopulation(pop_size, n, start_depot);
    
    % Evaluăm fitness-ul inițial
    fitness = evaluatePopulation(population, distanceMatrix);
    [best_fitness, best_idx] = min(fitness);
    best_route = population(best_idx, :);
    
    for gen = 1:num_generations
        % Selecție
        parents = tournamentSelection(population, fitness, pop_size);
        
        % Încrucișare (Crossover)
        children = crossover(parents, pop_size, start_depot);
        
        % Mutație
        children = mutate(children, mutation_rate, start_depot);
        
        % Evaluăm copiii
        children_fitness = evaluatePopulation(children, distanceMatrix);
        
        % Înlocuim populația
        [population, fitness] = replacePopulation(population, fitness, children, children_fitness);
        
        % Actualizăm cea mai bună soluție
        [min_fit, min_idx] = min(fitness);
        if min_fit < best_fitness
            best_fitness = min_fit;
            best_route = population(min_idx, :);
        end
    end
end

% Funcții auxiliare pentru algoritmul genetic
function population = initializePopulation(pop_size, n, start_depot)
    population = zeros(pop_size, n+1);
    for i = 1:pop_size
        route = [start_depot, randperm(n-1) + (start_depot ~= 1), start_depot];
        route(route == start_depot & (1:n+1) ~= 1 & (1:n+1) ~= n+1) = [];
        route = [start_depot, route, start_depot];
        if length(route) > n+1
            route = route(1:n+1);
        end
        population(i, :) = route;
    end
end

function fitness = evaluatePopulation(population, distanceMatrix)
    [pop_size, ~] = size(population);
    fitness = zeros(pop_size, 1);
    
    for i = 1:pop_size
        fitness(i) = calculateRouteDistance(population(i, :), distanceMatrix);
    end
end

function parents = tournamentSelection(population, fitness, pop_size)
    [~, route_length] = size(population);
    parents = zeros(pop_size, route_length);
    tournament_size = 3;
    
    for i = 1:pop_size
        candidates = randi(pop_size, 1, tournament_size);
        [~, idx] = min(fitness(candidates));
        winner = candidates(idx);
        parents(i, :) = population(winner, :);
    end
end

function children = crossover(parents, pop_size, start_depot)
    [~, route_length] = size(parents);
    children = zeros(pop_size, route_length);
    
    for i = 1:2:pop_size
        if i+1 <= pop_size
            p1 = parents(i, :);
            p2 = parents(i+1, :);
            
            % Ordered Crossover (OX)
            cut1 = randi([2, route_length-2]);
            cut2 = randi([cut1+1, route_length-1]);
            
            % Copiii primesc segmentele de la părinți
            c1 = zeros(1, route_length);
            c2 = zeros(1, route_length);
            
            c1(1) = start_depot;
            c1(end) = start_depot;
            c2(1) = start_depot;
            c2(end) = start_depot;
            
            c1(cut1:cut2) = p1(cut1:cut2);
            c2(cut1:cut2) = p2(cut1:cut2);
            
            % Completăm restul nodurilor
            p1_idx = cut2 + 1;
            c1_idx = cut2 + 1;
            
            while c1_idx <= route_length-1 || (c1_idx > route_length-1 && c1_idx < cut1)
                if p1_idx > route_length-1
                    p1_idx = 2;
                end
                
                if ~ismember(p2(p1_idx), c1) && p2(p1_idx) ~= start_depot
                    if c1_idx > route_length-1
                        c1_idx = 2;
                    end
                    
                    c1(c1_idx) = p2(p1_idx);
                    c1_idx = c1_idx + 1;
                end
                
                p1_idx = p1_idx + 1;
            end
            
            p2_idx = cut2 + 1;
            c2_idx = cut2 + 1;
            
            while c2_idx <= route_length-1 || (c2_idx > route_length-1 && c2_idx < cut1)
                if p2_idx > route_length-1
                    p2_idx = 2;
                end
                
                if ~ismember(p1(p2_idx), c2) && p1(p2_idx) ~= start_depot
                    if c2_idx > route_length-1
                        c2_idx = 2;
                    end
                    
                    c2(c2_idx) = p1(p2_idx);
                    c2_idx = c2_idx + 1;
                end
                
                p2_idx = p2_idx + 1;
            end
            
            children(i, :) = c1;
            children(i+1, :) = c2;
        else
            children(i, :) = parents(i, :);
        end
    end
end

function children = mutate(children, mutation_rate, start_depot)
    [pop_size, route_length] = size(children);
    
    for i = 1:pop_size
        if rand < mutation_rate
            % Swap Mutation
            idx1 = randi([2, route_length-1]);
            idx2 = randi([2, route_length-1]);
            
            while idx1 == idx2
                idx2 = randi([2, route_length-1]);
            end
            
            temp = children(i, idx1);
            children(i, idx1) = children(i, idx2);
            children(i, idx2) = temp;
        end
    end
end

function [new_population, new_fitness] = replacePopulation(population, fitness, children, children_fitness)
    pop_size = size(population, 1);
    combined_population = [population; children];
    combined_fitness = [fitness; children_fitness];
    
    [~, sorted_idx] = sort(combined_fitness);
    
    new_population = combined_population(sorted_idx(1:pop_size), :);
    new_fitness = combined_fitness(sorted_idx(1:pop_size));
end

% Task 5: Implementarea și testarea în MATLAB
% -------------------------------------------

% Funcția principală de testare
function testResults = testVehicleRouting()
    % Inițializăm parametrii de test
    test_nodes = 10;
    grid_size = 100;
    
    % Generăm noduri aleatorii (x, y)
    nodes_x = randi([0, grid_size], 1, test_nodes);
    nodes_y = randi([0, grid_size], 1, test_nodes);
    
    % Poziția depozitului (primul nod)
    depot_x = grid_size/2;
    depot_y = grid_size/2;
    nodes_x(1) = depot_x;
    nodes_y(1) = depot_y;
    
    % Calculăm matricea de distanțe (folosind distanța euclidiană)
    distanceMatrix = zeros(test_nodes);
    for i = 1:test_nodes
        for j = 1:test_nodes
            if i ~= j
                distanceMatrix(i, j) = sqrt((nodes_x(i) - nodes_x(j))^2 + (nodes_y(i) - nodes_y(j))^2);
            end
        end
    end
    
    % Testăm algoritmii
    start_time = tic;
    nn_route = nearestNeighbor(distanceMatrix, 1);
    nn_time = toc(start_time);
    nn_distance = calculateRouteDistance(nn_route, distanceMatrix);
    
    start_time = tic;
    [twoopt_route, twoopt_distance] = twoOpt(nn_route, distanceMatrix);
    twoopt_time = toc(start_time);
    
    start_time = tic;
    [sa_route, sa_distance] = simulatedAnnealing(nn_route, distanceMatrix, 100, 0.95, 1000);
    sa_time = toc(start_time);
    
    start_time = tic;
    [ga_route, ga_distance] = geneticAlgorithm(distanceMatrix, 50, 100, 0.1, 1);
    ga_time = toc(start_time);
    
    % Structurăm rezultatele
    testResults.nodes.x = nodes_x;
    testResults.nodes.y = nodes_y;
    testResults.distanceMatrix = distanceMatrix;
    
    testResults.nearestNeighbor.route = nn_route;
    testResults.nearestNeighbor.distance = nn_distance;
    testResults.nearestNeighbor.time = nn_time;
    
    testResults.twoOpt.route = twoopt_route;
    testResults.twoOpt.distance = twoopt_distance;
    testResults.twoOpt.time = twoopt_time;
    testResults.twoOpt.improvement = (nn_distance - twoopt_distance) / nn_distance * 100;
    
    testResults.simulatedAnnealing.route = sa_route;
    testResults.simulatedAnnealing.distance = sa_distance;
    testResults.simulatedAnnealing.time = sa_time;
    testResults.simulatedAnnealing.improvement = (nn_distance - sa_distance) / nn_distance * 100;
    
    testResults.geneticAlgorithm.route = ga_route;
    testResults.geneticAlgorithm.distance = ga_distance;
    testResults.geneticAlgorithm.time = ga_time;
    testResults.geneticAlgorithm.improvement = (nn_distance - ga_distance) / nn_distance * 100;
    
    % Vizualizăm rezultatele
    visualizeRoutes(testResults);
end

% Funcție pentru vizualizarea traseelor
function visualizeRoutes(results)
    figure('Name', 'Vehicle Routing Comparison', 'Position', [100, 100, 1200, 900]);
    
    % Subploturile pentru diferite algoritmi
    subplot(2, 2, 1);
    plotRoute(results.nodes.x, results.nodes.y, results.nearestNeighbor.route, 'Nearest Neighbor');
    title(sprintf('Nearest Neighbor\nDistance: %.2f, Time: %.3f s', results.nearestNeighbor.distance, results.nearestNeighbor.time));
    
    subplot(2, 2, 2);
    plotRoute(results.nodes.x, results.nodes.y, results.twoOpt.route, '2-Opt');
    title(sprintf('2-Opt\nDistance: %.2f, Time: %.3f s, Improvement: %.2f%%', ...
        results.twoOpt.distance, results.twoOpt.time, results.twoOpt.improvement));
    
    subplot(2, 2, 3);
    plotRoute(results.nodes.x, results.nodes.y, results.simulatedAnnealing.route, 'Simulated Annealing');
    title(sprintf('Simulated Annealing\nDistance: %.2f, Time: %.3f s, Improvement: %.2f%%', ...
        results.simulatedAnnealing.distance, results.simulatedAnnealing.time, results.simulatedAnnealing.improvement));
    
    subplot(2, 2, 4);
    plotRoute(results.nodes.x, results.nodes.y, results.geneticAlgorithm.route, 'Genetic Algorithm');
    title(sprintf('Genetic Algorithm\nDistance: %.2f, Time: %.3f s, Improvement: %.2f%%', ...
        results.geneticAlgorithm.distance, results.geneticAlgorithm.time, results.geneticAlgorithm.improvement));
    
    % Adăugarea unui titlu global
    sgtitle('Vehicle Routing Optimization - Algorithm Comparison');
end

% Funcție pentru plotarea unui traseu
function plotRoute(x, y, route, algorithmName)
    hold on;
    % Desenăm toate nodurile
    scatter(x, y, 100, 'bo', 'filled');
    % Evidențiem depozitul (primul nod)
    scatter(x(1), y(1), 150, 'ro', 'filled');
    
    % Desenăm traseul
    for i = 1:length(route)-1
        from = route(i);
        to = route(i+1);
        plot([x(from), x(to)], [y(from), y(to)], 'k-', 'LineWidth', 1.5);
        % Adăugăm săgeți pentru a indica direcția
        arrow_x = x(from) + 0.7 * (x(to) - x(from));
        arrow_y = y(from) + 0.7 * (y(to) - y(from));
        plot(arrow_x, arrow_y, 'k>', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    end
    
    % Adăugăm etichete pentru noduri
    for i = 1:length(x)
        if i == 1
            text(x(i), y(i) - 5, 'Depot', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
        else
            text(x(i), y(i) - 5, num2str(i), 'HorizontalAlignment', 'center');
        end
    end
    
    hold off;
    grid on;
    axis equal;
    xlabel('X Coordinate');
    ylabel('Y Coordinate');
    title(algorithmName);
end

% Task 6: Compararea performanțelor algoritmilor
% ----------------------------------------------

% Funcție pentru compararea detaliată a performanțelor algoritmilor
function performanceResults = compareAlgorithmPerformance(numTests, minNodes, maxNodes)
    % Inițializarea structurii pentru rezultate
    performanceResults = struct();
    performanceResults.testCases = cell(1, numTests);
    
    % Parametri pentru teste
    gridSize = 100;
    
    % Rulăm mai multe teste cu diferite configurații
    for test = 1:numTests
        % Generăm un număr aleatoriu de noduri între minNodes și maxNodes
        numNodes = randi([minNodes, maxNodes]);
        
        % Generăm noduri aleatorii
        nodes_x = randi([0, gridSize], 1, numNodes);
        nodes_y = randi([0, gridSize], 1, numNodes);
        
        % Poziția depozitului (primul nod)
        depot_x = gridSize/2;
        depot_y = gridSize/2;
        nodes_x(1) = depot_x;
        nodes_y(1) = depot_y;
        
        % Calculăm matricea de distanțe
        distanceMatrix = zeros(numNodes);
        for i = 1:numNodes
            for j = 1:numNodes
                if i ~= j
                    distanceMatrix(i, j) = sqrt((nodes_x(i) - nodes_x(j))^2 + (nodes_y(i) - nodes_y(j))^2);
                end
            end
        end
        
        % Rezultatele pentru acest test
        testCase = struct();
        testCase.numNodes = numNodes;
        testCase.algorithms = struct();
        
        % Testăm algoritmii și măsurăm performanța
        
        % Nearest Neighbor
        start_time = tic;
        nn_route = nearestNeighbor(distanceMatrix, 1);
        nn_time = toc(start_time);
        nn_distance = calculateRouteDistance(nn_route, distanceMatrix);
        
        testCase.algorithms.nearestNeighbor = struct();
        testCase.algorithms.nearestNeighbor.time = nn_time;
        testCase.algorithms.nearestNeighbor.distance = nn_distance;
        testCase.algorithms.nearestNeighbor.route = nn_route;
        
        % 2-Opt
        start_time = tic;
        [twoopt_route, twoopt_distance] = twoOpt(nn_route, distanceMatrix);
        twoopt_time = toc(start_time);
        
        testCase.algorithms.twoOpt = struct();
        testCase.algorithms.twoOpt.time = twoopt_time;
        testCase.algorithms.twoOpt.distance = twoopt_distance;
        testCase.algorithms.twoOpt.route = twoopt_route;
        testCase.algorithms.twoOpt.improvement = (nn_distance - twoopt_distance) / nn_distance * 100;
        
        % Simulated Annealing
        start_time = tic;
        [sa_route, sa_distance] = simulatedAnnealing(nn_route, distanceMatrix, 100, 0.95, 1000);
        sa_time = toc(start_time);
        
        testCase.algorithms.simulatedAnnealing = struct();
        testCase.algorithms.simulatedAnnealing.time = sa_time;
        testCase.algorithms.simulatedAnnealing.distance = sa_distance;
        testCase.algorithms.simulatedAnnealing.route = sa_route;
        testCase.algorithms.simulatedAnnealing.improvement = (nn_distance - sa_distance) / nn_distance * 100;
        
        % Genetic Algorithm
        start_time = tic;
        [ga_route, ga_distance] = geneticAlgorithm(distanceMatrix, 50, 100, 0.1, 1);
        ga_time = toc(start_time);
        
        testCase.algorithms.geneticAlgorithm = struct();
        testCase.algorithms.geneticAlgorithm.time = ga_time;
        testCase.algorithms.geneticAlgorithm.distance = ga_distance;
        testCase.algorithms.geneticAlgorithm.route = ga_route;
        testCase.algorithms.geneticAlgorithm.improvement = (nn_distance - ga_distance) / nn_distance * 100;
        
        % Salvăm rezultatele pentru acest test
        performanceResults.testCases{test} = testCase;
    end
    
    % Calculăm statistici agregate pentru toate testele
    performanceResults.aggregateStats = aggregatePerformanceStats(performanceResults.testCases);
    
    % Vizualizăm rezultatele
    visualizePerformanceComparison(performanceResults);
end

% Funcție pentru calcularea statisticilor agregate
function stats = aggregatePerformanceStats(testCases)
    numTests = length(testCases);
    stats = struct();
    
    % Inițializăm structure pentru fiecare algoritm
    algorithmNames = {'nearestNeighbor', 'twoOpt', 'simulatedAnnealing', 'geneticAlgorithm'};
    
    for i = 1:length(algorithmNames)
        algName = algorithmNames{i};
        stats.(algName) = struct();
        stats.(algName).avgTime = 0;
        stats.(algName).avgDistance = 0;
        stats.(algName).bestDistance = inf;
        stats.(algName).worstDistance = 0;
        
        if ~strcmp(algName, 'nearestNeighbor')
            stats.(algName).avgImprovement = 0;
            stats.(algName).bestImprovement = 0;
            stats.(algName).worstImprovement = inf;
        end
    end
    
    % Agregăm rezultatele
    for t = 1:numTests
        testCase = testCases{t};
        
        for i = 1:length(algorithmNames)
            algName = algorithmNames{i};
            alg = testCase.algorithms.(algName);
            
            stats.(algName).avgTime = stats.(algName).avgTime + alg.time;
            stats.(algName).avgDistance = stats.(algName).avgDistance + alg.distance;
            
            if alg.distance < stats.(algName).bestDistance
                stats.(algName).bestDistance = alg.distance;
            end
            
            if alg.distance > stats.(algName).worstDistance
                stats.(algName).worstDistance = alg.distance;
            end
            
            if ~strcmp(algName, 'nearestNeighbor')
                stats.(algName).avgImprovement = stats.(algName).avgImprovement + alg.improvement;
                
                if alg.improvement > stats.(algName).bestImprovement
                    stats.(algName).bestImprovement = alg.improvement;
                end
                
                if alg.improvement < stats.(algName).worstImprovement
                    stats.(algName).worstImprovement = alg.improvement;
                end
            end
        end
    end
    
    % Calculăm mediile
    for i = 1:length(algorithmNames)
        algName = algorithmNames{i};
        stats.(algName).avgTime = stats.(algName).avgTime / numTests;
        stats.(algName).avgDistance = stats.(algName).avgDistance / numTests;
        
        if ~strcmp(algName, 'nearestNeighbor')
            stats.(algName).avgImprovement = stats.(algName).avgImprovement / numTests;
        end
    end
end

% Funcție pentru vizualizarea comparației performanțelor
function visualizePerformanceComparison(performanceResults)
    stats = performanceResults.aggregateStats;
    
    % Figura pentru compararea timpilor de execuție
    figure('Name', 'Algorithm Performance Comparison', 'Position', [100, 100, 1200, 800]);
    
    % Timpul de execuție
    subplot(2, 2, 1);
    algNames = {'NN', '2-Opt', 'SA', 'GA'};
    times = [
        stats.nearestNeighbor.avgTime, 
        stats.twoOpt.avgTime, 
        stats.simulatedAnnealing.avgTime, 
        stats.geneticAlgorithm.avgTime
    ];
    bar(times);
    set(gca, 'XTickLabel', algNames);
    title('Average Execution Time');
    ylabel('Time (seconds)');
    grid on;
    
    % Distanța medie
    subplot(2, 2, 2);
    distances = [
        stats.nearestNeighbor.avgDistance, 
        stats.twoOpt.avgDistance, 
        stats.simulatedAnnealing.avgDistance, 
        stats.geneticAlgorithm.avgDistance
    ];
    bar(distances);
    set(gca, 'XTickLabel', algNames);
    title('Average Route Distance');
    ylabel('Distance');
    grid on;
    
    % Îmbunătățirea față de Nearest Neighbor
    subplot(2, 2, 3);
    improvements = [
        0, 
        stats.twoOpt.avgImprovement, 
        stats.simulatedAnnealing.avgImprovement, 
        stats.geneticAlgorithm.avgImprovement
    ];
    bar(improvements);
    set(gca, 'XTickLabel', algNames);
    title('Average Improvement over Nearest Neighbor');
    ylabel('Improvement (%)');
    grid on;
    
    % Comparație între timpul de execuție și îmbunătățire
    subplot(2, 2, 4);
    scatter([
        stats.twoOpt.avgTime, 
        stats.simulatedAnnealing.avgTime, 
        stats.geneticAlgorithm.avgTime
    ], [
        stats.twoOpt.avgImprovement, 
        stats.simulatedAnnealing.avgImprovement, 
        stats.geneticAlgorithm.avgImprovement
    ], 100, 'filled');
    text(stats.twoOpt.avgTime, stats.twoOpt.avgImprovement, '2-Opt', 'HorizontalAlignment', 'left');
    text(stats.simulatedAnnealing.avgTime, stats.simulatedAnnealing.avgImprovement, 'SA', 'HorizontalAlignment', 'left');
    text(stats.geneticAlgorithm.avgTime, stats.geneticAlgorithm.avgImprovement, 'GA', 'HorizontalAlignment', 'left');
    title('Time vs. Improvement Trade-off');
    xlabel('Execution Time (seconds)');
    ylabel('Improvement (%)');
    grid on;
    
    sgtitle('Algorithm Performance Comparison');
    
    % Generăm și un grafic pentru scala comportamentului în funcție de numărul de noduri
    if length(performanceResults.testCases) > 1
        figure('Name', 'Scaling Behavior', 'Position', [100, 100, 1200, 600]);
        
        % Extragem numărul de noduri și timpul pentru fiecare test
        numTests = length(performanceResults.testCases);
        nodes = zeros(1, numTests);
        nnTimes = zeros(1, numTests);
        twoOptTimes = zeros(1, numTests);
        saTimes = zeros(1, numTests);
        gaTimes = zeros(1, numTests);
        
        for t = 1:numTests
            testCase = performanceResults.testCases{t};
            nodes(t) = testCase.numNodes;
            nnTimes(t) = testCase.algorithms.nearestNeighbor.time;
            twoOptTimes(t) = testCase.algorithms.twoOpt.time;
            saTimes(t) = testCase.algorithms.simulatedAnnealing.time;
            gaTimes(t) = testCase.algorithms.geneticAlgorithm.time;
        end
        
        % Sortăm datele după numărul de noduri
        [nodes, idx] = sort(nodes);
        nnTimes = nnTimes(idx);
        twoOptTimes = twoOptTimes(idx);
        saTimes = saTimes(idx);
        gaTimes = gaTimes(idx);
        
        % Plotăm timpul de execuție în funcție de numărul de noduri
        subplot(1, 2, 1);
        plot(nodes, nnTimes, 'bo-', 'LineWidth', 2);
        hold on;
        plot(nodes, twoOptTimes, 'ro-', 'LineWidth', 2);
        plot(nodes, saTimes, 'go-', 'LineWidth', 2);
        plot(nodes, gaTimes, 'mo-', 'LineWidth', 2);
        hold off;
        legend('Nearest Neighbor', '2-Opt', 'Simulated Annealing', 'Genetic Algorithm');
        title('Execution Time vs. Number of Nodes');
        xlabel('Number of Nodes');
        ylabel('Execution Time (seconds)');
        grid on;
        
        % Extragere date pentru îmbunătățire
        twoOptImprovements = zeros(1, numTests);
        saImprovements = zeros(1, numTests);
        gaImprovements = zeros(1, numTests);
        
        for t = 1:numTests
            testCase = performanceResults.testCases{idx(t)};
            twoOptImprovements(t) = testCase.algorithms.twoOpt.improvement;
            saImprovements(t) = testCase.algorithms.simulatedAnnealing.improvement;
            gaImprovements(t) = testCase.algorithms.geneticAlgorithm.improvement;
        end
        
        % Plotăm îmbunătățirea în funcție de numărul de noduri
        subplot(1, 2, 2);
        plot(nodes, twoOptImprovements, 'ro-', 'LineWidth', 2);
        hold on;
        plot(nodes, saImprovements, 'go-', 'LineWidth', 2);
        plot(nodes, gaImprovements, 'mo-', 'LineWidth', 2);
        hold off;
        legend('2-Opt', 'Simulated Annealing', 'Genetic Algorithm');
        title('Improvement vs. Number of Nodes');
        xlabel('Number of Nodes');
        ylabel('Improvement over NN (%)');
        grid on;
        
        sgtitle('Scaling Behavior of Algorithms');
    end
end

% Task 7: Aplicații reale în transport și vehicule autonome
% ----------------------------------------------------------
function realWorldApplication()
    % Definim o zonă urbană de 100x100
    gridSize = 100;
    
    % Generăm un număr aleatoriu de locații de livrare
    numDeliveries = randi([10, 20]); % Între 10 și 20 locații
    delivery_x = randi([0, gridSize], 1, numDeliveries);
    delivery_y = randi([0, gridSize], 1, numDeliveries);

    % Poziția depozitului (centrul zonei)
    depot_x = gridSize / 2;
    depot_y = gridSize / 2;
    
    % Adăugăm depozitul la lista de locații
    delivery_x = [depot_x, delivery_x];
    delivery_y = [depot_y, delivery_y];
    
    % Numărul total de puncte (inclusiv depozitul)
    numNodes = length(delivery_x);

    % Calculăm matricea de distanțe
    distanceMatrix = zeros(numNodes);
    for i = 1:numNodes
        for j = 1:numNodes
            if i ~= j
                distanceMatrix(i, j) = sqrt((delivery_x(i) - delivery_x(j))^2 + (delivery_y(i) - delivery_y(j))^2);
            end
        end
    end

    % Aplicăm algoritmii de optimizare
    nn_route = nearestNeighbor(distanceMatrix, 1);
    [twoopt_route, ~] = twoOpt(nn_route, distanceMatrix);
    [sa_route, ~] = simulatedAnnealing(nn_route, distanceMatrix, 100, 0.95, 1000);
    [ga_route, ~] = geneticAlgorithm(distanceMatrix, 50, 100, 0.1, 1);

    % Vizualizăm rezultatele
    plotOptimizedRoutes(delivery_x, delivery_y, depot_x, depot_y, nn_route, twoopt_route, sa_route, ga_route);
end