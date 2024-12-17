% To run this script, please install the following add ons: Matlab
% optimization toolbox, navigation toolbox. Also install any additional ones 
% it might ask you to install

disp("start");

%% Step 1: setting up and visualizing initial environment
% Define the environment
map = binaryOccupancyMap(20,20,2);
obstacles = [6 1; 6 2; 6 3; 6 4; 6 5; 6 6; 6 7; 6 7.5;
             7 1; 7 2; 7 3; 7 4; 7 5; 7 6; 7 7; 7 7.5;
             8 1; 8 2; 8 3; 8 4; 8 5; 8 6; 8 7; 8 7.5;
             9 1; 9 2; 9 3; 9 4; 9 5; 9 6; 9 7; 9 7.5;
             10 1; 10 2; 10 3; 10 4; 10 5; 10 6; 10 7; 10 7.5;
             11 1; 11 2; 11 3; 11 4; 11 5; 11 6; 11 7; 11 7.5;
             12 1; 12 2; 12 3; 12 4; 12 4.5;
             13 1; 13 2; 13 3; 13 4; 13 4.5;
             13.25 7.5;
             16 1; 16 2; 16 3; 16 4; 16 4.5;
             16 8; 16 9; 16 10; 16 11; 16 12; 16 13;
             16 17; 16 17.5;
             6 17.5; 6 17; 6 16; 6 15; 6 14; 6 13; 6 12.5;
             7 17.5; 7 17; 7 16; 7 15; 7 14; 7 13; 7 12.5;
             8 17.5; 8 17; 8 16; 8 15; 8 14; 8 13; 8 12.5;
             9 17.5; 9 17; 9 16; 9 15; 9 14; 9 13; 9 12.5;
             10 17.5; 10 17; 10 16; 10 15; 10 14; 10 13; 10 12.5;
             11 17.5; 11 17; 11 16; 11 15; 11 14; 11 13; 11 12.5;
             12 17.5; 12 17; 12 16.5;
             13 17.5; 13 17; 13 16.5;
             14 17.8; 15 17.8];
setOccupancy(map,obstacles,ones(length(obstacles),1))
inflate(map,0.25)


% Define inital point (translated into our coordinate system)
angles = deg2rad([-90, -54, -18, 18, 54, 90 ]);
x_init = [12, 6 ,deg2rad(45)];
vehiclePose = x_init;
maxrange = 20;


% Show inital point and distance measures
figure("Name", "Inital point");
title("Initial position");
show(map);

% Calculate and plot intersection points with obstacles
intsectionPts = rayIntersection(map,vehiclePose,angles,maxrange);

hold on
plot(intsectionPts(:,1),intsectionPts(:,2),'*r') % Intersection points
plot(vehiclePose(1),vehiclePose(2),'ob') % Vehicle pose
for i = 1:6
    plot([vehiclePose(1),intsectionPts(i,1)],...
        [vehiclePose(2),intsectionPts(i,2)],'-b') % Plot laser rays
end


% Calculate laser distance measurements with custom function
optimal_measurements = laser_measurements(map,vehiclePose,angles,maxrange);
disp("");
disp("initial measurements");
disp(optimal_measurements);



%% Step 2: set up and run optimization problem for localization
% Define Optimisation problem

% objective function: euclidian distance between initial and current distance measures
objective_function = @(x) calculate_distance(optimal_measurements, ...
    laser_measurements(map,x,angles,maxrange));

% constraints: remain within coordinate system and limit angle between 0 and 360 degrees
lb = [0, 0, deg2rad(0)];
ub = [20, 20, deg2rad(360)]; 

% run genetic algorithm and display results 
ga_options = optimoptions('ga', 'FunctionTolerance',1e-3, 'Display', 'iter', 'MaxGenerations', 1000, 'PopulationSize', 1000);

[x_opt, fval] = ga(objective_function, 3, [], [], [], [], lb, ub, [], ga_options);
disp("x_opt");
disp(x_opt);
disp("x_init");
disp(x_init);
disp("fval");
disp(fval); % distance measure between the two distance measures




%% Step 3: visualize resulting location
% compare laser measurements to original
measurements_result = laser_measurements(map,x_opt,angles,maxrange);
disp("result measurements");
disp(measurements_result);

disp("initial measurements");
disp(optimal_measurements);

% show our resulting estimate and distances graphically in figure 2
map_result = map;
figure("Name", "Resulting estimate of inital point");
title("Resulting estimate of initial point");
show(map_result)

vehiclePose_result = x_opt;
intsectionPts_result = rayIntersection(map_result,vehiclePose_result,angles,maxrange);

hold on
plot(intsectionPts_result(:,1),intsectionPts_result(:,2),'*r') % Intersection points
plot(vehiclePose_result(1),vehiclePose_result(2),'ob') % Vehicle pose
for i = 1:6
    plot([vehiclePose_result(1),intsectionPts_result(i,1)],...
        [vehiclePose_result(2),intsectionPts_result(i,2)],'-b') % Plot intersecting rays
end



%% Define custom functions we use for our problem

% This function calculates the euclidian distance between two vectors of
% arbitrary size
% We use it for two purposes: 
    % 1) to calculate the distance from our robot two the nearest obstaces
    % 2) to calculate the loss function in our optimization problem as the
    %    distance between the current laser measurements and the original
    %   measurements

function [distance] = calculate_distance(point1, point2)
    distance = sqrt( sum((point1 - point2) .^2) );
end

% This function replaces NaN values with the maximum distance. The matlab
% function we use to find the intersection points (rayIntersection) returns NaN, if no
% obstacle is found by a laser. Therefore the distance measure has to
% replaced with our maximum distance of 20
function [point1] = replace_nan(point1)
    index = isnan(point1);
    point1(index) = 20;
end

% This function calculates the laser measurements for any point on the map.
% To this end, it uses intersectionPoints, to find the points where our
% lasers hit obstacles. Next it calculates the distances from our current
% position to those intersection points.
function [distances] = laser_measurements(map, vehiclePose, angles, maxrange)
    intsectionPts = rayIntersection(map,vehiclePose,angles,maxrange);
    distances = zeros(1,6);
    vehiclePose = vehiclePose(1:end-1);
    for k = 1:6
        if  isnan(intsectionPts(k))
            distances(k) = NaN;
        else 
            d = calculate_distance(intsectionPts(k,:), vehiclePose);
            distances(k) = d;
        end
    end
    distances = replace_nan(distances);
end