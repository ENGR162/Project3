clc
clear all
n = 4;

%{
rows = n;
columns = 2*n + 2

sensitivity = zeros(rows + 1, columns);
%}

data = zeros(10,2); %tests by cost and water

%
[cost, water] = Proj2_SensitivityMain_Team11.m(175, 175, 175, 2)
data(1, 1) = cost;
data(1, 2) = water;

%Biggest Large
[cost, water] = Proj2_SensitivityMain_Team11.m(213, 213, 75, 2);
data(2, 1) = cost;
data(2, 2) = water;

[cost, water] = Proj2_SensitivityMain_Team11.m(175, 175, 100, 2)
data(3, 1) = cost;
data(3, 2) = water;

[cost, water] = Proj2_SensitivityMain_Team11.m(175, 175, 125, 2)
data(4, 1) = cost;
data(4, 2) = water;