%Perlin Noise
function [noiseMap] = PerlinNoise(perlinMapSize, maxAmp, increasePerTS, bias)
%Creates a perlin noise array which is perlinMapSize long
%and no larger than maxAmp in each individual value
noiseMap = zeros(perlinMapSize, 1);
noiseMap(1) = (rand * 2 * maxAmp) - maxAmp;

biasAdd = 0;
for i = 2 : perlinMapSize
    newIncrement = (rand * 2 - 1) * increasePerTS + biasAdd;
    if(abs(noiseMap(i - 1) + newIncrement) < maxAmp)
        noiseMap(i) = noiseMap(i - 1) + newIncrement;
        biasAdd = -newIncrement / bias;
    else
        noiseMap(i) = noiseMap(i - 1) - newIncrement;
        biasAdd = newIncrement / bias;
    end
end

end