clc
clear all
numTimeSteps = 10000;
timeStepDuration = 60 * 30;
time = 1:numTimeSteps;
systemVelo = [10, 20];
appVeloWater = zeros(numTimeSteps, 2);
appVeloAir = zeros(numTimeSteps, 2);

waterSpeedX = PerlinNoise(numTimeSteps, 2.25, 0.2, 10);
waterSpeedY = PerlinNoise(numTimeSteps, 2.25, 0.2, 10);
airSpeedX = PerlinNoise(numTimeSteps, 16.5, 0.4, 10);
airSpeedY = PerlinNoise(numTimeSteps, 16.5, 0.4, 10);

appVeloWater(:,1) = waterSpeedX - systemVelo(1);
appVeloWater(:,2) = waterSpeedY - systemVelo(2);
appVeloAir(:,1) = airSpeedX - systemVelo(1);
appVeloAir(:,2) = airSpeedY - systemVelo(2);
 

figure(1)
hold on
plot(time,appVeloWater(:,1))
plot(time,appVeloWater(:,2))
%title('Water Velocity vs. Time')
%xlabel('Time in ' + nun2str(timeStepDuration) + 'Seconds')
%ylabel('Water Velocity (m/s)')
hold off

figure(2)
hold on
plot(time,appVeloAir(:,1))
plot(time,appVeloAir(:,2))
hold off