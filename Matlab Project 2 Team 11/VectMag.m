function [mag] = VectMag(vec)
    %finds the magnitude of the vector
    mag = sqrt(sum(vec .^ 2));