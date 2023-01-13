function bezierDerivCoeffs = returnBezierDerivQuadratic(t)

 bezierDerivCoeffs = [-2*(1-t),...
      (-2*(t) + 2*(1 - t)), ...
     2*t];

end
