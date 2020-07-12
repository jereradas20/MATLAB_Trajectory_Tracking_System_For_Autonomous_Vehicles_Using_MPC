function [scenario, egoVehicle, scenario_ref_x, scenario_ref_y] = create_scenario()
%% SETTING THE SCENARIO

% scenario object
scenario = drivingScenario;

% road parameters
roadCenters = 2*[0.51 -57.92 0;
    12.98 -58.1 0;
    26.68 -58.25 0;
    40.2 -55.7 0;
    52.2 -47.6 0;
    10 0 0;
    47.4 20.3 0;
    45 49 0;
    31.1 51.6 0;
    -36.7 51.3 0;
    -49 50 0;
    -100 0 0;
    -80 -40 0;
    -64.3 -49.9 0;
    -44.2 -55.3 0;
    -29.7 -57.1 0;
    -14.07 -57.92 0;
    0.51 -57.92 0];
laneSpecification = lanespec(2, 'Width', 2.85);
road(scenario, roadCenters, 'Lanes', laneSpecification);



% vehicle object
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Length', 2.68, ...
    'Position', [5.9 -1.975*59.4 0]);


% road boundaries
rb = roadBoundaries(scenario);
RB=cell2mat(rb);

% centerline coordinates
center_line_x=(RB(:,1)+wrev(RB(:,4)))/2;
center_line_y=(RB(:,2)+wrev(RB(:,5)))/2;

% reference coordinates
scenario_ref_x=(center_line_x+RB(:,1))/2;
scenario_ref_y=(center_line_y+RB(:,2))/2;


end

