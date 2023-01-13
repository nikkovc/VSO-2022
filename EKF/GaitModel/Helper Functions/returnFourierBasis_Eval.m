function fourierCoeffs3P = returnFourierBasis_Eval(phase,stepLength,incline, N)

 inclineFuncs = returnBezierLinear(incline);
 stepLengthFuncs = returnBezierLinear(stepLength);
 phaseFuncs = returnFourier(phase, N);
 
 numInclineFuncs = length(inclineFuncs);
 numStepLengthFuncs = length(stepLengthFuncs);
 numPhaseFuncs = length(phaseFuncs);
    
% fourierCoeffs3P = zeros(1,numInclineFuncs*numStepLengthFuncs*numPhaseFuncs);
%  N=1;

fourierCoeffs3P = kron(inclineFuncs, kron( stepLengthFuncs,  phaseFuncs));
 
%  for ii = 1:numInclineFuncs
%      inclineFunc = inclineFuncs(ii);
%      
%      for jj = 1:numStepLengthFuncs
%          stepLengthFunc = stepLengthFuncs(jj);
%          
%          for kk = 1:numPhaseFuncs
%              phaseFunc = phaseFuncs(kk);
%              fourierCoeffs3P(N) = inclineFunc * stepLengthFunc * phaseFunc;
%              N = N + 1;
%          end
%          
%      end
%      
%  end
%  fourierCoeffs3P

% pause
end