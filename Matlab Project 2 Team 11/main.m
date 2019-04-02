%To DO: Corials+PGrad, FUnction names need to be standard
clc
clear all
%Variables
numTimeSteps = 10000000; %Num of timesteps
timeStepDuration = 10; %In seconds
time = 1:numTimeSteps; %Array of time
travelDistance = 4078291;
tempUpper = 16.75;
tempLower = 4;
dTdX = (tempUpper - tempLower) / travelDistance;
tugForce = 3000000;


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

externalForces = zeros(numTimeSteps, 2);

distance = zeros(numTimeSteps, 1);

velocity = zeros(numTimeSteps, 1);
velocity(1) = 10;
acceleration = zeros(numTimeSteps, 1);

temperature = zeros(numTimeSteps, 1);
temperature(1) = tempLower;

mass = zeros(numTimeSteps, 1);
length = zeros(numTimeSteps, 1);
width = zeros(numTimeSteps, 1);
height = zeros(numTimeSteps, 1);

%INITIAL VALUES
length(1) = 175;
width(1) = 175;
height(1) = 175;
mass(1) = length(1) * width(1) * height(1) * 917;

heightBelowWater = zeros(numTimeSteps, 1);
heightAboveWater = zeros(numTimeSteps, 1);

forceTotalMag = zeros(numTimeSteps, 1);

tugAngle = zeros(numTimeSteps, 1);

%Perlin Noise for water and air velocities
waterSpeedX = zeros(numTimeSteps, 1); %PerlinNoise(numTimeSteps, 2 / sqrt(2), 0.05, 10);
waterSpeedY = zeros(numTimeSteps, 1); % PerlinNoise(numTimeSteps, 2 / sqrt(2), 0.05, 10);
airSpeedX = zeros(numTimeSteps, 1); %PerlinNoise(numTimeSteps, 12 / sqrt(2), 0.1, 10);
airSpeedY = zeros(numTimeSteps, 1); %PerlinNoise(numTimeSteps, 12 / sqrt(2), 0.1, 10);

%Travel Simulation Loop
reachedDestination = 0; %Boolean to check if simulation is over
curTF = 1; %Current Time Frame
bergNotMelted = true;
while reachedDestination == 0

    %Apparent Velocity Calculation
    appVeloWater(curTF,1:2) = [waterSpeedX(curTF), waterSpeedY(curTF) - velocity(curTF)];
    appVeloWaterMag(curTF) = VectMag(appVeloWater(curTF, 1:2));

    appVeloAir(curTF,1:2) = [airSpeedX(curTF), airSpeedY(curTF) - velocity(curTF)];
    appVeloAirMag(curTF) = VectMag(appVeloAir(curTF, 1:2));

    %Mass Loss
    [mass(curTF + 1), length(curTF + 1), width(curTF + 1), height(curTF + 1)] = MassLoss([mass(curTF), length(curTF), width(curTF), height(curTF)], appVeloWaterMag(curTF), VectMag([airSpeedX(curTF), airSpeedY(curTF)]), temperature(curTF), timeStepDuration);
    if mass(curTF + 1) <= 0
        disp("Ice Cube melted :(")
        burgNotMelted = 0;
        break;
    end
    %Water Height
    heightBelowWater(curTF) = mass(curTF) / (997.0 * width(curTF) * length(curTF));
    heightAboveWater(curTF) = height(curTF) - heightBelowWater(curTF);

    %External Forces__________________

    %Frictional Drag Water
    skinArea = (length(curTF) * width(curTF)) + (2 * (heightBelowWater(curTF) * length(curTF)));
    frictionDragWater(curTF,1:2) = FrictionForce(appVeloWaterMag(curTF), appVeloWater(curTF,1:2), skinArea, 997);
    frictionDragWaterMag(curTF) = VectMag(frictionDragWater(curTF, 1:2));

    %From Drag Water
    crossArea = width(curTF) * heightBelowWater(curTF);
    formDragWater(curTF,1:2) = FormForce(appVeloWaterMag(curTF), appVeloWater(curTF,1:2), crossArea, 997);
    formDragWaterMag(curTF) = VectMag(formDragWater(curTF, 1:2));

    %Frictional Drag Air
    skinArea = (length(curTF) * width(curTF)) + (2 * (heightAboveWater(curTF) * length(curTF)));
    frictionDragAir(curTF,1:2) = FrictionForce(appVeloAirMag(curTF), appVeloAir(curTF,1:2), skinArea, 1.28);
    frictionDragAirMag(curTF) = VectMag(frictionDragAir(curTF, 1:2));

    %From Drag Air
    crossArea = width(curTF) * heightAboveWater(curTF);
    formDragAir(curTF,1:2) = FormForce(appVeloAirMag(curTF), appVeloAir(curTF,1:2), crossArea, 1.28);
    formDragAirMag(curTF) = VectMag(formDragAir(curTF, 1:2));

    
    externalForces(curTF,1:2) = frictionDragWater(curTF,1:2) + formDragWater(curTF,1:2) + frictionDragAir(curTF,1:2) + formDragAir(curTF,1:2); %+ other forces
    if abs(externalForces(curTF, 1)) > tugForce
        externalForces(curTF, 1) = tugForce;
    end
    tugAngle = asin(-externalForces(curTF, 1) / tugForce);

    %Tug Boat Force Calulations________________

    forceTotalMag(curTF) = (tugForce * cos(tugAngle)) + externalForces(curTF, 2);

    %Distance
    acceleration(curTF) = forceTotalMag(curTF) / (mass(curTF + 1) + 3855535);
    dV = acceleration(curTF) * timeStepDuration;
    velocity(curTF + 1) = velocity(curTF) + dV;
    dX = velocity(curTF) * timeStepDuration;
    distance(curTF + 1) = distance(curTF) + dX;

    if distance(curTF + 1) >= travelDistance
        reachedDestination = 1;
    end

    disp(travelDistance - distance(curTF))
    %disp(length(curTF))
    
    %Temperature
    temperature(curTF + 1) = (distance(curTF) * dTdX) + 4;

    curTF = curTF + 1;  
end


sittingMelt = zeros(numTimeSteps, 1);
drinkingMelt = 210000000 * timeStepDuration / 86400; %kg per time step

travelTime = curTF;

while bergNotMelted
    
    velocity(curTF) = 0;

    appVeloWater(curTF,1:2) = [waterSpeedX(curTF), waterSpeedY(curTF)];
    appVeloWaterMag(curTF) = VectMag(appVeloWater(curTF, 1:2));

    %appVeloAir(curTF,1:2) = [airSpeedX(curTF), airSpeedY(curTF)];
    %appVeloAirMag(curTF) = VectMag(appVeloAir(curTF, 1:2))

    [mass(curTF + 1), length(curTF + 1), width(curTF + 1), height(curTF + 1)] = MassLoss([mass(curTF), length(curTF), width(curTF), height(curTF)], appVeloWaterMag(curTF), VectMag([airSpeedX(curTF), airSpeedY(curTF)]), temperature(curTF), timeStepDuration);    sittingMelt = mass(curTF) - mass(curTF + 1);
    %Drinking
    mass(curTF + 1) = mass(curTF + 1) - drinkingMelt;

    if(mass(curTF + 1))
        bergNotMelted = false;
        travelTime = curTF - travelTime;
    end
    disp(mass(curTF))
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