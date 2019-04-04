clc
clear all

length = input('Input Length of Iceberg (m): ');
width = input('Input Width of Iceberg (m): ');
height = input('Input Height of Iceberg (m): ');
numBoats = input('Input Number of Boats pulling of Iceberg: ');

%length = [150, 400];
%width = [150, 300];
%height = [120, 180];

%numBoats = 6;
%data = zeros(100,5); %tests by cost and water

[cost, waterEff, water] = Proj2_SensitivityMain_Team11(length, width, height, numBoats);

%{
i = 1;
for x = 20:20:300
    for y = 20:20:300
        [cost, waterEff, water] = Proj2_SensitivityMain_Team11(x + 100, width(2), y + 100, numBoats);
        if(cost <= 0 || waterEff <= 0)
            disp('Failed to Pull 1')
        else
            disp('succ')
        end
        values = [cost, waterEff, water, x + 100, y + 100];
        data(i, :) = values;
        i = i + 1
    end
end

costEff = data(:,2) ./ data(:,1);

%}

%{
[cost, waterEff] = Proj2_SensitivityMain_Team11(length(1), width(1), height(1), numBoats);
if(cost <= 0 || waterEff <= 0)
    disp('Failed to Pull 1')
else
    disp('succ')
end
data(1, 1) = cost;
data(1, 2) = waterEff;

[cost, waterEff] = Proj2_SensitivityMain_Team11(length(2), width(1), height(1), numBoats);
if(cost <= 0 || waterEff <= 0)
    disp('Failed to Pull 2')
end
data(2, 1) = cost;
data(2, 2) = waterEff;

[cost, waterEff] = Proj2_SensitivityMain_Team11(length(1), width(2), height(1), numBoats);
if(cost <= 0 || waterEff <= 0)
    disp('Failed to Pull 3')
end
data(3, 1) = cost;
data(3, 2) = waterEff;

[cost, waterEff] = Proj2_SensitivityMain_Team11(length(1), width(1), height(2), numBoats);
if(cost <= 0 || waterEff <= 0)
    disp('Failed to Pull 4')
end
data(4, 1) = cost;
data(4, 2) = waterEff;

[cost, waterEff] = Proj2_SensitivityMain_Team11(length(2), width(2), height(1), numBoats);
if(cost <= 0 || waterEff <= 0)
    disp('Failed to Pull 5')
end
data(5, 1) = cost;
data(5, 2) = waterEff;

[cost, waterEff] = Proj2_SensitivityMain_Team11(length(2), width(1), height(2), numBoats);
if(cost <= 0 || waterEff <= 0)
    disp('Failed to Pull 6')
end
data(6, 1) = cost;
data(6, 2) = waterEff;

[cost, waterEff] = Proj2_SensitivityMain_Team11(length(1), width(2), height(2), numBoats);
if(cost <= 0 || waterEff <= 0)
    disp('Failed to Pull 7')
end
data(7, 1) = cost;
data(7, 2) = waterEff;

[cost, waterEff] = Proj2_SensitivityMain_Team11(length(2), width(2), height(2), numBoats);
if(cost <= 0 || waterEff <= 0)
    disp('Failed to Pull 8')
end
data(8, 1) = cost;
data(8, 2) = waterEff;

costEff = data(:,2) ./ data(:,1);

Codd = zeros(1,3);
Ceven = zeros(1,3);
Measure = zeros(1,3);
Sensitivity = zeros(1,3);

Codd(1) = (1/4) * ((costEff(8) - costEff(5)) + (costEff(2) - costEff(1)));
Codd(2) = (1/4) * ((costEff(8) - costEff(6)) + (costEff(2) - costEff(2)));
Codd(3) = (1/4) * ((costEff(8) - costEff(7)) + (costEff(2) - costEff(3)));

Ceven(1) = (1/4) * ((costEff(8) - costEff(5)) - (costEff(2) - costEff(1)));
Ceven(2) = (1/4) * ((costEff(8) - costEff(6)) - (costEff(2) - costEff(2)));
Ceven(3) = (1/4) * ((costEff(8) - costEff(7)) - (costEff(2) - costEff(3)));

Measure(1) = abs(Codd(1)) + abs(Ceven(1));
Measure(2) = abs(Codd(2)) + abs(Ceven(2));
Measure(3) = abs(Codd(3)) + abs(Ceven(3));

Sensitivity(1) = Measure(1) / sum(Measure);
Sensitivity(2) = Measure(2) / sum(Measure);
Sensitivity(3) = Measure(3) / sum(Measure);

disp('done')
%}