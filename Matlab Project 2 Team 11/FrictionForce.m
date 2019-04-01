function [fricForce] = FrictionForce(appVeloMag, appVelo, skinArea)
    waterDensity = 997 %kg/m^3
    Cd = 0.2226781246

    fricForce = (0.5 * waterDensity * skinArea * Cd * appVeloMag) .* appVelo
end