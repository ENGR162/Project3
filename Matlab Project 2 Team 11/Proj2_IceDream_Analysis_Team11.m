clc
clear all

length = input('Input Length of Iceberg (m): ');
width = input('Input Width of Iceberg (m): ');
height = input('Input Height of Iceberg (m): ');
numBoats = input('Input Number of Boats pulling the Iceberg: ');

[cost, waterEff, water] = Proj2_SensitivityMain_Team11(length, width, height, numBoats);