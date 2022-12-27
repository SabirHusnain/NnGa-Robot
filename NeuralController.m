function [voltage_left, voltage_right] = NeuralController(Left_sensor ,Right_sensor,net)
%
% SENSOR - Compute the Motor Voltage Depending Upon the Neural Network
% (Mind's) Decision
%
% Syntax: [voltage_left, voltage_right] = NeuralController(Left_sensor,Right_sensor,net)
%
% Inputs: 
%   Left_sensor         Output of left sensor, in m
%   Right_sensor        Output of right sensor, in m
%   net                 Neural Network Model 
%
% Outputs: 
%   voltage_left        Left Motor Voltage
%   voltage_right       Right Motor Voltage
%
%
% Author: Sabir Husnain
%
sensors = [Left_sensor;Right_sensor];
N_N = net(sensors);
voltage = [4,4];
%disp(N_N);
%display(brian(1));
if N_N == [1;0]
    voltage = [6,-1.5];
    
elseif N_N == [0;1]
    voltage = [-1.5,6];

elseif N_N == [1;1]
    voltage = [4,4];
    
elseif N_N == [0;0]
    voltage = [1,1];
    
end
    
voltage_left=voltage(1);
voltage_right=voltage(2);
%disp(voltage)