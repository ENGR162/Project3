function [newM, newL, newW, newH] = MassLoss([mass, leng, wid, hei], appVel, velAir, tempWater)
    %Calculates the new mass due to lost mass from melt and erosion
    tempIce = -.4; %deg C, Fixed acc. to El-Tahan et al. 1987
    densityIce = 917 %kg/m^3
    %Wind-driven wave erosion
    mE = 8.7e-6 * (abs(velAir) ^ .5) + 5.8e-7 * abs(velAir);
    
    %Thermal sidewall erosion from buoyant convection
    mV = 8.8e-8 * tempWater + 1.5e-8 * (tempWater ^ 2);
    
    %Turbulent basal melt 
    mB = 6.7e-6 * ((appVel) ^ (4/5)) * (tempWater - tempIce) * (leng ^ (-1/5));
    
    %Changes
    
    dL = mE + mV;
    dW = mE + mV;
    dH = mB;
    
    newL = leng + dL;
    newW = wid + dW;
    newH = hei + dH;
    
    %Mass stuff
    dV = dL * dW * dH;
    dM = dV * densityIce;
    newM = dM + mass;
    end
    
    