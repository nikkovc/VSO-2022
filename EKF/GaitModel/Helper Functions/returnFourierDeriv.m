function fourierCoeffs = returnFourierDeriv(t, N)
    
    w = 1:1:N;
    %derivs of cos(w * t)
    d1 = (2*pi) * w .* -sin(w * 2 * pi * t);
    
    %derivs of sin(w * t)
    d2 = (2*pi) * w .* cos(w * 2 * pi * t);
    fourierCoeffs = [0, d1, d2];
end