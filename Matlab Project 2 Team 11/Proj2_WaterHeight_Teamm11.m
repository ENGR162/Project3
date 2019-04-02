function [heightBelow, heightAbove] = Proj2_WaterHeight_Team11(mass, length, width, height, shape)
    waterDen = 1029.0 %density of sea ice
    iceDen = 934.0

    if(shape == 1)
        heightBelow = mass / (waterDen * width * length);

    elseif (shape == 2)
        volume = (height * width * length / 3)        

    elseif (shape == 3)

    elseif (shape == 4)

    elseif (shape == 5)
        
    end

    heightAbove = height - heightBelowWater;