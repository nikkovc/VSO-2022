function output = returnPiecewiseBezier3DDeriv_dphase(phase,stepLength,incline,h_piecewise, phaseDelins,numFuncs)

    output = zeros(size(phase));

    if phase < 0
        phase = 0;
        
    elseif phase > 1
        phase = 1;
        
    end
    
    
    for i = 1:length(phase)
        if phase(i) <= phaseDelins(1)
            phaseIdxs = 1:1*numFuncs;
        elseif phase(i) <= phaseDelins(2)
            phaseIdxs = 1*numFuncs+1:2*numFuncs;
        elseif phase(i) <= phaseDelins(3)
            phaseIdxs = 2*numFuncs+1:3*numFuncs;
        else%if phase_estimate <= phaseDelins(4)
            phaseIdxs = 3*numFuncs+1:4*numFuncs;

        end
        
        output(i) = h_piecewise(phaseIdxs)' * returnBezier3P_sL_incline_DerivEval_dphase(phase(i),stepLength,incline)';
    
    end

end