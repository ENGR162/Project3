clc
clear all
%Variables
numTimeSteps = 10000; %Num of timesteps
timeStepDuration = 60 * 30; %In seconds
time = 1:numTimeSteps; %Array of time
iceberg = [mass, length, width, height]; %Vector of iceberg stats

%Initialization of response data
appVeloWater = zeros(numTimeSteps, 2);
appVeloWaterMag = zeros(numTimeSteps, 1);

appVeloAir = zeros(numTimeSteps, 2);
appVeloAirMag = zeros(numTimeSteps, 1);

formDrag = zeros(numTimeSteps, 2);
formDragMag = zeros(numTimeSteps, 1);

frictionDrag = zeros(numTimeSteps, 2);
frictionDragMag = zeros(numTimeSteps, 1);

%Perlin Noise for water and air velocities
waterSpeedX = PerlinNoise(numTimeSteps, 2.25, 0.2, 10);
waterSpeedY = PerlinNoise(numTimeSteps, 2.25, 0.2, 10);
airSpeedX = PerlinNoise(numTimeSteps, 16.5, 0.4, 10);
airSpeedY = PerlinNoise(numTimeSteps, 16.5, 0.4, 10);

%Simulation Loop
reachedDestination = 0 %Boolean to check if simulation is over
curTF = 1 %Current Time Frame
while reachedDestination == 0

    %The system's current velocity
    systemVelo = [10, 20]; %update to allow changes to sys velo

    %Apparent Velocity Calculation
    appVeloWater(curTF,1:2) = [waterSpeedX(curTF) - systemVelo(1), waterSpeedY(curTF) - systemVelo(2)];
    appVeloWaterMag = VectMag(appVeloWater(curTF, 1:2))

    appVeloAir(curTF,1:2) = [airSpeedX(curTF) - systemVelo(1), airSpeedY(curTF) - systemVelo(2)];
    appVeloAirMag = VectMag(appVeloAir(curTF, 1:2))

    %Mass Loss
    iceberg = MassLoss(iceberg, appVeloWaterMag(curTF), vectMag([airspeedX(curTF), airspeedY(curTF)]), tempWater);

    %Force 
    frictionDrag(curTF,1:2) = FrictionForce(appVeloWaterMag, appVeloWater,)
    frictionDragMag()

    curTF = curTF + 1

end


 

%Forces
%


%
%figure(1)
%hold on
%plot(time,appVeloWater(:,1))
%plot(time,appVeloWater(:,2))
%title('Water Velocity vs. Time')
%xlabel('Time in ' + nun2str(timeStepDuration) + 'Seconds')
%ylabel('Water Velocity (m/s)')
%hold off

%figure(2)
%hold on
%plot(time,appVeloAir(:,1))
%plot(time,appVeloAir(:,2))
%hold off