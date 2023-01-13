function fourierDerivCoeffs3P = returnFourierBasis_DerivEval_dsL(phase,stepLength,incline, N)


inclineFuncs = returnBezierLinear(incline);
 stepLengthFuncs = returnBezierDerivLinear(stepLength);
 phaseFuncs = returnFourier(phase, N);
 
 numInclineFuncs = length(inclineFuncs);
 numStepLengthFuncs = length(stepLengthFuncs);
 numPhaseFuncs = length(phaseFuncs);
    
    fourierDerivCoeffs3P = kron(inclineFuncs, kron( stepLengthFuncs,  phaseFuncs));
%  N=1;
%  for ii = 1:numInclineFuncs
%      inclineFunc = inclineFuncs(ii);
%      
%      for jj = 1:numStepLengthFuncs
%          stepLengthFunc = stepLengthFuncs(jj);
%          
%          for kk = 1:numPhaseFuncs
%              phaseFunc = phaseFuncs(kk);
%              fourierDerivCoeffs3P(N) = inclineFunc * stepLengthFunc * phaseFunc;
%              N = N + 1;
%          end
%          
%      end
%      
%  end

end
