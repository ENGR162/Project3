clc
clear all
%Variables
numTimeSteps = 10000; %Num of timesteps
timeStepDuration = 60 * 30; %In seconds
time = 1:numTimeSteps; %Array of time
travelDistance = 
tempUpper = 16.75;
tempLower = 4;
dTdX = (tempUpper - tempLower) / travelDistance;
tugForce = 3000000


%Initialization of response data
appVeloWater = zeros(numTimeSteps, 2);
appVeloWaterMag = zeros(numTimeSteps, 1);

appVeloAir = zeros(numTimeSteps, 2);
appVeloAirMag = zeros(numTimeSteps, 1);

frictionDragWater = zeros(numTimeSteps, 2);
frictionDragWaterMag = zeros(numTimeSteps, 1);

formDragWater = zeros(numTimeSteps, 2);
formDragWaterMag = zeros(numTimeSteps, 1);

frictionDragAir = zeros(numTimeSteps, 2);
frictionDragAirMag = zeros(numTimeSteps, 1);

formDragAir = zeros(numTimeSteps, 2);
formDragAirMag = zeros(numTimeSteps, 1);

externalForces = zeros(numTimeSteps, 1);

distance = zeros(numTimeSteps, 1);

velocity = zeros(numTimeSteps, 1);

acceleration = zeros(numTimeSteps, 1);

temperature = zeros(numTimeSteps, 1);
temperature(1) = tempLower;

mass = zeros(numTimeSteps, 1);
length = zeros(numTimeSteps, 1);
width = zeros(numTimeSteps, 1);
height = zeros(numTimeSteps, 1);

heightBelowWater = zeros(numTimeSteps, 1);
heightAboveWater = zeros(numTimeSteps, 1);

forceTotalMag = zeros(numTimeSteps, 1);

tugAngle = zeros(numTimeSteps, 1);

%Perlin Noise for water and air velocities
waterSpeedX = PerlinNoise(numTimeSteps, 2.25, 0.2, 10);
waterSpeedY = PerlinNoise(numTimeSteps, 2.25, 0.2, 10);
airSpeedX = PerlinNoise(numTimeSteps, 16.5, 0.4, 10);
airSpeedY = PerlinNoise(numTimeSteps, 16.5, 0.4, 10);

%Travel Simulation Loop
reachedDestination = 0 %Boolean to check if simulation is over
curTF = 1 %Current Time Frame
while reachedDestination == 0

    %The system's current velocity
    systemVelo = [10, 20]; %update to allow changes to sys velo

    %Apparent Velocity Calculation
    appVeloWater(curTF,1:2) = [waterSpeedX(curTF) - systemVelo(1), waterSpeedY(curTF) - systemVelo(2)];
    appVeloWaterMag(curTF) = VectMag(appVeloWater(curTF, 1:2))

    appVeloAir(curTF,1:2) = [airSpeedX(curTF) - systemVelo(1), airSpeedY(curTF) - systemVelo(2)];
    appVeloAirMag(curTF) = VectMag(appVeloAir(curTF, 1:2))

    %Mass Loss
    [mass(curTF), lenght(curTF), width(curTF), height(curTF)] = MassLoss([mass(curTF), lenght(curTF), width(curTF), height(curTF)], appVeloWaterMag(curTF), vectMag([airspeedX(curTF), airspeedY(curTF)]), temperature(curTF));

    %Water Height
    heightBelowWater(curTF) = mass(curTF) / (997.0 * width * length)
    heightAboveWater(curTF) = height - heightBelowWater(curTF)

    %External Forces__________________

    %Frictional Drag Water
    skinArea = (length * width) + (2 * (heightBelowWater(curTF) * length))
    frictionDragWater(curTF,1:2) = FrictionForce(appVeloWaterMag, appVeloWater, skinArea)
    frictionDragWaterMag(curTF) = VectMag(frictionDragWater(curTF, 1:2))

    %From Drag Water
    crossArea = wdith * heightBelowWater(curTF)
    formDragWater(curTF,1:2) = FormForce(appVeloWaterMag, appVeloWater, crossArea)
    formDragWaterMag(curTF) = VectMag(formDragWater(curTF, 1:2))

    %Frictional Drag Air
    skinArea = (length * width) + (2 * (heightAboveWater(curTF) * length))
    frictionDragAir(curTF,1:2) = FrictionForce(appVeloAirMag, appVeloAir, skinArea)
    frictionDragAirMag(curTF) = VectMag(frictionDragAir(curTF, 1:2))

    %From Drag Air
    crossArea = wdith * heightAboveWater(curTF)
    formDragAir(curTF,1:2) = FormForce(appVeloAirMag, appVeloAir, crossArea)
    formDragAirMag(curTF) = VectMag(formDragAir(curTF, 1:2))

    externalForces(curTF) = frictionDragWater(curTF) + formDragWater(curTF) + frictionDragWater(curTF) + formDragWater(curTF) %+ other forces
    tugAngle = asin(-externalForces(curTF, 1) / tugForce) * 180 / pi

    %Tug Boat Force Calulations________________

    forceTotalMag(curTF) = (tugForce * cos(tugAngle)) + externalForces(curTF, 2)

    %Distance
    acceleration(curTF) = forceTotalMag(curTF) / mass(curTF);
    dV = acceleration(curTF) * timeStepDuration;
    velocity(curTF + 1) = velocity(curTF) + dV;
    dX = velocity(curTF + 1) * timeStepDuration;
    distance(curTF + 1) = distance(curTF) + dX;

    if distance(curTF + 1) >= travelDistance
        reachedDestination = 1;
    end

    %Temperature
    temperature(curTF + 1) = (distance(curTF + 1) * dTdX) + 4;

    curTF = curTF + 1;

    
end

%Arrived Drinking Loop



 

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