function fourierCoeffs = returnFourier(t, N)
    
    orders = 1:1:N;
    
    fourierCoeffs = [1, cos(orders * 2 * pi * t), sin(orders * 2 * pi * t)];
end