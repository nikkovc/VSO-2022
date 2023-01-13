function fourierCoeffs = returnFourier2ndDeriv(t, N)
    
    w = 1:1:N;
%     d1 = w .* -sin(w * t);
%     d2 = w .* cos(w * t);

    dd1 = (2*pi)^2 * (w.^2) .* -cos(w * 2*pi * t);
    dd2 = (2*pi)^2 * (w.^2) .* -sin(w * 2*pi * t);
    
    fourierCoeffs = [0, dd1, dd2];
end