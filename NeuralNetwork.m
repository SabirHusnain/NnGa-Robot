function [net] = NeuralNetwork()
%
% NEURALNETWORK - Create a Neural Network for the System
%
% Syntax: [net] = NeuralNetwork()
%
% Outputs: 
%   net: Feed Forward Neural Network (pre trained)
%
%
% Author: Sabir Husnain
%

p1 = [1;1];
t1 = [1;1];

p2 = [1; 0.25];
t2 = [0; 1];
 
p3 = [0.25;1];
t3 = [1; 0];

p4 = [0.9;1];
t4 = [1; 1];

p5 = [0.75; 1];
t5 = [1; 0];

p6 = [1;0.75];
t6 = [0; 1];

p7 = [0.25;0.25];
t7 = [1; 0];

p8 = [0.3;0.3];
t8 = [1; 0];

p9 = [0.5;0.5];
t9 = [1; 0];

p10 = [0.9;0.9];
t10 = [1;1];

p11 = [1;0.5];
t11 = [0;1];

p12 = [0.5;1];
t12 = [1;0];

p13 = [0.4;0.4];
t13 = [1;0];

p14 = [0.2;0.2];
t14 = [1;0];

p = [p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13,p14];
t = [t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13,t14];

%initialise perceptron
net = perceptron;
%set epochs 
net.trainParam.epochs = 1000;
%train 
net = train(net,p,t);
%get weights and bias
w = net.iw{1,1};
b = net.b{1};
%a is final answer, check if right
a = net(p);

if a == t
    disp('Matches!');
else
    disp(a);
    disp(t);
end
disp(net)