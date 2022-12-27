%% Neural Network Based Autonomus Robot

% Author: Sabir Husnain
% Mechatroncis & Control Engineering
% University of Engineering & Technology Lahore

close all; clear; clc;

%% Simulation setup
% Time
simulationTime_total = 20;      % in seconds
stepSize_time = 0.05;           % in seconds 

% Initial states and controls
voltage_left  = 4;              % in volts
voltage_right = 4;              % in volts

state_initial = zeros(1,24);    % VARIABLES OF IMPORTANCE:
                                % state_initial(13): forward velocity,    v,m/s
                                % state_initial(18): rotational velocity, r,rad/s
                                % state_initial(19): current x-position,  x,m
                                % state_initial(20): current y-position,  y,m
                                % state_initial(24): heading angle,       psi,rad
                                
%% Environment
canvasSize_horizontal = 10;
canvasSize_vertical   = 10;
stepSize_canvas       = 0.01;

%% Neural Network
[net] = NeuralNetwork();

%% Creating Environment
obstacleMatrix = zeros(canvasSize_horizontal / stepSize_canvas, canvasSize_vertical / stepSize_canvas);

% Generating walls
[wall_1, obstacleMatrix] = WallGeneration( -1,  1, 1.2, 1.2, 'h', obstacleMatrix); 
[wall_2, obstacleMatrix] = WallGeneration( -3, -3,  -2,   2, 'v', obstacleMatrix);
[wall_3, obstacleMatrix] = WallGeneration( 3, 3,  -2,   2, 'v', obstacleMatrix);
[wall_4, obstacleMatrix] = WallGeneration( -3, -2,  -2,   0, 'h', obstacleMatrix);

%% Main simulation
% Initialize simulation 
timeSteps_total = simulationTime_total/stepSize_time;
state = state_initial;
time = 0;

% Run simulation
for timeStep = 1:timeSteps_total
    % Assign voltage applied to wheels
    voltages = [voltage_left; voltage_left; voltage_right; voltage_right];
    
    % Controller
    sensorOut=Sensor(state(timeStep, 19), state(timeStep, 20), state(timeStep,24), obstacleMatrix);
    disp(sensorOut)
    Left_sensor = sensorOut(1);
    Right_sensor = sensorOut(2);
    [voltage_left,voltage_right] = NeuralController(Left_sensor, Right_sensor,net);
    voltS=[voltage_left,voltage_right];
    disp(voltS);
    
    % Run model
    [state_derivative(timeStep,:), state(timeStep,:)] = DynamicalModel(voltages, state(timeStep,:), stepSize_time);   
    
    % Euler intergration (Dynamics)
    state(timeStep + 1,:) = state(timeStep,:) + (state_derivative(timeStep,:) * stepSize_time); 
    time(timeStep + 1)    = timeStep * stepSize_time;
    
    % Plotting
    figure(1); clf; hold on; grid on; axis([-5,5,-5,5]);
    DrawRobot(0.2, state(timeStep,20), state(timeStep,19), state(timeStep,24), 'b');
    plot(wall_1(:,1), wall_1(:,2),'k-');
    plot(wall_2(:,1), wall_2(:,2),'k-'); 
    plot(wall_3(:,1), wall_3(:,2),'k-');
    plot(wall_4(:,1), wall_4(:,2),'k-'); 
    xlabel('y, m'); ylabel('x, m');
end

%% Plot Final Results
figure(2);
subplot(2,2,1); hold on; grid on; grid minor;
plot(state(:,20), state(:,19));
title('Robot Trajectory');
xlabel('Position X (m)');
ylabel('Position Y (m)');
axis([-5 5 -5 5]);

subplot(2,2,2); hold on; grid on; grid minor;
plot(time, state(:,13));
title('Linear Velocity of Robot');
xlabel('Time (s)');
ylabel('Velocity (m.s^-^1)');

subplot(2,2,3); hold on; grid on; grid minor;
plot(time, state(:,24));
title('Heading Angle of Robot');
xlabel('Time (s)');
ylabel('Angle (rad)');

subplot(2,2,4); hold on; grid on; grid minor;
plot(time, state(:,18));
title('Rotational Velocity of Robot');
xlabel('Time (s)');
ylabel('Velocity (rad.s^-^1)');