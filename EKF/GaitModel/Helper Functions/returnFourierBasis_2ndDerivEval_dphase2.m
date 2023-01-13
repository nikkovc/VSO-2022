function fourier2ndDerivCoeffs3P = returnFourierBasis_2ndDerivEval_dphase2(phase,stepLength,incline, N)
    

inclineFuncs = returnBezierLinear(incline);
 stepLengthFuncs = returnBezierLinear(stepLength);
 phaseFuncs = returnFourier2ndDeriv(phase,N);
 
 numInclineFuncs = length(inclineFuncs);
 numStepLengthFuncs = length(stepLengthFuncs);
 numPhaseFuncs = length(phaseFuncs);
    
%  fourier2ndDerivCoeffs3P = zeros(1,numInclineFuncs*numStepLengthFuncs*numPhaseFuncs);
fourier2ndDerivCoeffs3P = kron(inclineFuncs, kron( stepLengthFuncs,  phaseFuncs));

%  N=1;
%  for ii = 1:numInclineFuncs
%      inclineFunc = inclineFuncs(ii);
%      
%      for jj = 1:numStepLengthFuncs
%          stepLengthFunc = stepLengthFuncs(jj);
%          
%          for kk = 1:numPhaseFuncs
%              phaseFunc = phaseFuncs(kk);
%              fourier2ndDerivCoeffs3P(N) = inclineFunc * stepLengthFunc * phaseFunc;
%              N = N + 1;
%          end
%          
%      end
%      
%  end
%  
 
%  bezier2ndDerivCoeffs3P = [...
%      (incline).*stepLength.*(6 * (1 - phase)),...
%      (incline).*stepLength.* ( -2*(6*(1 - phase)) +  (6*phase) ), ...
%      (incline).*stepLength.*( 6*(1 - phase) + (-2 * (6 * phase)) ),...
%      (incline).*stepLength.*(6 * phase),...
%      (incline).*(1 - stepLength).*(6 * (1 - phase)), ...
%      (incline).*(1 - stepLength).*( -2*(6*(1 - phase)) +  (6*phase) ), ...
%      (incline).*(1 - stepLength).* ( 6*(1 - phase) + (-2 * (6 * phase)) ),...
%      (incline).*(1 - stepLength).*(6 * phase),...
%      (1 - incline).*stepLength.*(6 * (1 - phase)),...
%      (1 - incline).*stepLength.* ( -2*(6*(1 - phase)) +  (6*phase) ), ...
%      (1 - incline).*stepLength.*( 6*(1 - phase) + (-2 * (6 * phase)) ),...
%      (1 - incline).*stepLength.*(6 * phase),...
%      (1 - incline).*(1 - stepLength).*(6 * (1 - phase)), ...
%      (1 - incline).*(1 - stepLength).*( -2*(6*(1 - phase)) +  (6*phase) ), ...
%      (1 - incline).*(1 - stepLength).* ( 6*(1 - phase) + (-2 * (6 * phase)) ),...
%      (1 - incline).*(1 - stepLength).*(6 * phase) ...
%      ];

end
