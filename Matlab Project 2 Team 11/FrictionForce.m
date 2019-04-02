function [fricForce] = FrictionForce(appVeloMag, appVelo, skinArea, density)
    waterDensity = 997; %kg/m^3
    Cd = 0.2226781246;

    fricForce = (0.5 * density * skinArea * Cd * appVeloMag) .* appVelo;
end