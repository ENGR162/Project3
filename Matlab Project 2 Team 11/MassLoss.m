function [newM, newL, newW, newH] = MassLoss(iceAtt, appVel, velAir, tempWater, timeStep)
    %Calculates the new mass due to lost mass from melt and erosion
    tempIce = -.4; %deg C, Fixed acc. to El-Tahan et al. 1987
    densityIce = 917; %kg/m^3
    %Wind-driven wave erosion
    mE = 8.7e-6 * (abs(velAir) ^ .5) + 5.8e-7 * abs(velAir);
    
    %Thermal sidewall erosion from buoyant convection
    mV = 8.8e-8 * tempWater + 1.5e-8 * (tempWater ^ 2);
    
    %Turbulent basal melt 
    mB = 6.7e-6 * ((appVel) ^ (4/5)) * (tempWater - tempIce) * (iceAtt(2) ^ (-1/5));
    
    %Changes
    
    
    dL = mE + mV;
    dW = mE + mV;
    dH = mB;
    
    dL = dL * timeStep;
    dW = dW * timeStep;
    dH = dH * timeStep;
    
    newL = iceAtt(2) - dL;
    newW = iceAtt(3) - dW;
    newH = iceAtt(4) - dH;
    
    
    %Mass stuff
    dV = (iceAtt(2) * iceAtt(3) * iceAtt(4)) - (newL * newW * newH);
    dM = dV * densityIce;
    disp([dL, dW, dH])
    newM = iceAtt(1) - dM;
    end
    
    