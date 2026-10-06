%% ============================================================
%  2D AUTONOMOUS TARGET TRACKING & INTERCEPTION SIMULATION
%  MATLAB single-file project
% =============================================================

clear;
clc;
close all;

%% ---------------- SIMULATION SETTINGS ----------------

dt = 0.02;              % Time step
T  = 40;                % Simulation duration
t  = 0:dt:T;
N  = length(t);

%% ---------------- TARGET SETTINGS ----------------

% Initial target position
targetX = zeros(1,N);
targetY = zeros(1,N);

targetX(1) = 100;
targetY(1) = 60;

% Target velocity
targetSpeed = 3.0;
targetHeading = deg2rad(20);

% Small target maneuver
targetTurnRate = deg2rad(4);

%% ---------------- VEHICLE SETTINGS ----------------

% Initial position
vehicleX = zeros(1,N);
vehicleY = zeros(1,N);

vehicleX(1) = 0;
vehicleY(1) = 0;

% Initial heading
vehicleHeading = zeros(1,N);
vehicleHeading(1) = deg2rad(10);

% Vehicle speed
vehicleSpeed = 8;

% Maximum turn rate
maxTurnRate = deg2rad(45);

% Controller gain
headingGain = 3.0;

% Arrival/interception radius
interceptionRadius = 2.0;

%% ---------------- LOGGING ----------------

distance = zeros(1,N);
headingError = zeros(1,N);

intercepted = false;
interceptTime = NaN;
interceptIndex = N;

%% ============================================================
%                    MAIN SIMULATION
% ============================================================

for k = 1:N-1

    %% -------- TARGET MOTION --------

    % Slowly change target heading
    targetHeading = targetHeading + ...
                    targetTurnRate * dt;

    % Target position
    targetX(k+1) = targetX(k) + ...
                   targetSpeed*cos(targetHeading)*dt;

    targetY(k+1) = targetY(k) + ...
                   targetSpeed*sin(targetHeading)*dt;


    %% -------- RELATIVE POSITION --------

    dx = targetX(k) - vehicleX(k);
    dy = targetY(k) - vehicleY(k);

    distance(k) = sqrt(dx^2 + dy^2);


    %% -------- CHECK INTERCEPTION --------

    if distance(k) <= interceptionRadius

        intercepted = true;
        interceptTime = t(k);
        interceptIndex = k;

        vehicleX(k+1:end) = vehicleX(k);
        vehicleY(k+1:end) = vehicleY(k);

        targetX(k+1:end) = targetX(k);
        targetY(k+1:end) = targetY(k);

        distance(k+1:end) = distance(k);

        break;
    end


    %% -------- DESIRED DIRECTION --------

    desiredHeading = atan2(dy,dx);


    %% -------- HEADING ERROR --------

    error = desiredHeading - vehicleHeading(k);

    % Normalize angle to -pi ... +pi
    error = atan2(sin(error),cos(error));

    headingError(k) = error;


    %% -------- TRACKING CONTROLLER --------

    turnCommand = headingGain * error;


    %% -------- TURN RATE LIMIT --------

    turnCommand = max( ...
        min(turnCommand,maxTurnRate), ...
        -maxTurnRate);


    %% -------- UPDATE VEHICLE HEADING --------

    vehicleHeading(k+1) = ...
        vehicleHeading(k) + turnCommand*dt;


    %% -------- VEHICLE MOTION --------

    vehicleX(k+1) = vehicleX(k) + ...
                    vehicleSpeed*cos(vehicleHeading(k))*dt;

    vehicleY(k+1) = vehicleY(k) + ...
                    vehicleSpeed*sin(vehicleHeading(k))*dt;

end


%% ============================================================
%                  FINAL CALCULATIONS
% ============================================================

distance(interceptIndex) = sqrt( ...
    (targetX(interceptIndex)-vehicleX(interceptIndex))^2 + ...
    (targetY(interceptIndex)-vehicleY(interceptIndex))^2);


%% ============================================================
%                       RESULTS
% ============================================================

fprintf('\n');
fprintf('================================================\n');
fprintf('     2D AUTONOMOUS TARGET TRACKING SYSTEM\n');
fprintf('================================================\n');

fprintf('Initial vehicle position : (%.2f , %.2f)\n', ...
    vehicleX(1),vehicleY(1));

fprintf('Initial target position  : (%.2f , %.2f)\n', ...
    targetX(1),targetY(1));

fprintf('Vehicle speed             : %.2f units/s\n', ...
    vehicleSpeed);

fprintf('Target speed              : %.2f units/s\n', ...
    targetSpeed);

fprintf('Interception radius       : %.2f units\n', ...
    interceptionRadius);

fprintf('Minimum distance          : %.3f units\n', ...
    min(distance(1:interceptIndex)));

if intercepted

    fprintf('\nSTATUS: TARGET REACHED\n');
    fprintf('Time: %.2f seconds\n',interceptTime);
    fprintf('Distance: %.3f units\n', ...
        distance(interceptIndex));

else

    fprintf('\nSTATUS: TARGET NOT REACHED\n');
    fprintf('Final distance: %.3f units\n', ...
        distance(end));

end

fprintf('================================================\n');


%% ============================================================
%                 FIGURE 1: TRAJECTORIES
% ============================================================

figure('Name','2D Autonomous Tracking');

plot(targetX(1:interceptIndex), ...
     targetY(1:interceptIndex), ...
     'r--','LineWidth',2);

hold on;

plot(vehicleX(1:interceptIndex), ...
     vehicleY(1:interceptIndex), ...
     'b','LineWidth',2);

% Starting points
plot(targetX(1),targetY(1), ...
     'ro','MarkerSize',9,'LineWidth',2);

plot(vehicleX(1),vehicleY(1), ...
     'bo','MarkerSize',9,'LineWidth',2);

% Final points
plot(targetX(interceptIndex), ...
     targetY(interceptIndex), ...
     'rx','MarkerSize',14,'LineWidth',3);

plot(vehicleX(interceptIndex), ...
     vehicleY(interceptIndex), ...
     'bx','MarkerSize',14,'LineWidth',3);

grid on;
axis equal;

xlabel('X Position');
ylabel('Y Position');

title('2D Autonomous Target Tracking');

legend( ...
    'Moving Target', ...
    'Autonomous Vehicle', ...
    'Target Start', ...
    'Vehicle Start', ...
    'Target Position', ...
    'Vehicle Position', ...
    'Location','best');


%% ============================================================
%                 FIGURE 2: DISTANCE ERROR
% ============================================================

figure('Name','Distance Error');

plot(t(1:interceptIndex), ...
     distance(1:interceptIndex), ...
     'LineWidth',2);

hold on;

yline(interceptionRadius,'--');

grid on;

xlabel('Time (seconds)');
ylabel('Distance');

title('Distance Between Autonomous Vehicle and Target');

legend('Distance','Interception Threshold');


%% ============================================================
%                 FIGURE 3: HEADING ERROR
% ============================================================

figure('Name','Heading Error');

plot(t(1:interceptIndex), ...
     rad2deg(headingError(1:interceptIndex)), ...
     'LineWidth',2);

grid on;

xlabel('Time (seconds)');
ylabel('Heading Error (degrees)');

title('Heading Tracking Error');


%% ============================================================
%                     ANIMATION
% ============================================================

figure('Name','Live 2D Simulation');

% Determine plot limits
allX = [targetX(1:interceptIndex), ...
        vehicleX(1:interceptIndex)];

allY = [targetY(1:interceptIndex), ...
        vehicleY(1:interceptIndex)];

margin = 10;

xmin = min(allX)-margin;
xmax = max(allX)+margin;

ymin = min(allY)-margin;
ymax = max(allY)+margin;


for k = 1:5:interceptIndex

    clf;

    % Target trajectory
    plot(targetX(1:k), ...
         targetY(1:k), ...
         'r--','LineWidth',1.5);

    hold on;

    % Vehicle trajectory
    plot(vehicleX(1:k), ...
         vehicleY(1:k), ...
         'b','LineWidth',1.5);

    % Target
    plot(targetX(k),targetY(k), ...
         'ro','MarkerSize',10, ...
         'LineWidth',2);

    % Vehicle
    plot(vehicleX(k),vehicleY(k), ...
         'bo','MarkerSize',10, ...
         'LineWidth',2);

    % Connecting line
    plot([vehicleX(k),targetX(k)], ...
         [vehicleY(k),targetY(k)], ...
         'k:');

    grid on;

    axis equal;

    xlim([xmin xmax]);
    ylim([ymin ymax]);

    xlabel('X Position');
    ylabel('Y Position');

    title(sprintf( ...
        '2D Autonomous Tracking | Time = %.2f s | Distance = %.2f', ...
        t(k),distance(k)));

    legend( ...
        'Target Path', ...
        'Vehicle Path', ...
        'Target', ...
        'Vehicle', ...
        'Relative Position', ...
        'Location','best');

    drawnow;

end


%% ============================================================
%                     END
% ============================================================

if intercepted

    fprintf('\nSimulation finished: TARGET REACHED.\n');

else

    fprintf('\nSimulation finished: target not reached.\n');

end