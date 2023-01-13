close all;clc;clearvars -except Gaitcycle Continuous %removes all variables except for gaitcyle and continuous from the matlab workspace. Useful because loading these variables can take several minutes.
%load the InclineExperiment data from the folder location you specify
% load('Z:\your_file_location_here\InclineExperiment.mat') 
%This file is large and may take several minutes to load. We recommend not
%clearing the variable, and only loading it as many times as necessary.

%% Example: removing strides with outliers from kinematic data, and taking the mean

% Gaitcyle and Continuous store the InclineExperiment data in a MATLAB
% structure array, or 'struct'

% The fields of a struct can be iterated over using a cell array

% A cell array can be defined manually with this notation: 
sub={'AB01','AB02','AB03','AB04','AB05','AB06','AB07','AB08','AB09','AB10'};

% sub={'AB01'};

% The command 'fieldnames' also returns the name of all fields within a
% struct as a cell array

% trials = {'s1i10','s1i7x5','s1i5','s1i2x5','s1i0','s1d2x5','s1d5','s1d7x5','s1d10'}
trials = fieldnames(Gaitcycle.AB01);
% The command 'setdiff' is useful for removing cells of a given name
trials = setdiff(trials,'subjectdetails');


leg={'left'};
joint={'foot'};
percent_gait = linspace(0,1,150);

c = {'r','g','b'}
% c1 = {'b','b','b','b','b','r','r','r','r'}
% 
% c2 = [0.5,0.4,0.3,0.2,0.1,0.2,0.3,0.4,0.5]

% Initializing a matrix that will contain the mean joint data of all subjects,
%all tasks
% concatenated_data=NaN(150,numel(sub)*numel(trials));

% figure(1)
% subplot(3,1,1)
% hold on
% subplot(3,1,2)
% hold on
% subplot(3,1,3)
% hold on

figure(2)
subplot(3,1,1)
hold on
subplot(3,1,2)
hold on
subplot(3,1,3)
hold on





% figure(3)
% subplot(3,1,1)
% hold on
% subplot(3,1,2)
% hold on
% subplot(3,1,3)
% hold on

figure(4)
subplot(3,1,1)
hold on
subplot(3,1,2)
hold on
subplot(3,1,3)
hold on



stepMin = 0.9;
stepMax = 1.5;

inclineMin = -10;
inclineMax = 10;

phaseDelins = [0.1,0.5,0.65,1]


A_dataport_mat_master = [];
b_dataport_torque_master = [];

for i = 1:length(sub) %loop through all subjects in 'sub

    subj_mass = Gaitcycle.(sub{i}).subjectdetails{4,2}
    
    for j = 1:length(trials) %loop through all trials in 'trial'
        % store the kinematic data in a temporary variable
%         sub{i}
        trial = trials{j};
        [treadmillSpeed, treadmillIncline] = returnSpeedIncline(trial)
        
        torque_data = Gaitcycle.(sub{i}).(trials{j}).kinetics.jointmoment.(leg{1}).('ankle').x * (subj_mass/1000);
        % adjust
%         [rows,cols] = size(torque_data)
        
        time_data = Gaitcycle.(sub{i}).(trials{j}).cycles.(leg{1}).time;
%         [rows,cols] = size(time_data)
%         pause
        % delete the strides identified to contain outliers
        torque_data(:,Gaitcycle.(sub{i}).(trials{j}).stepsout.(leg{1})) = [];
        time_data(:,Gaitcycle.(sub{i}).(trials{j}).stepsout.(leg{1})) = [];
        
        
        
%         incline_scaled = (treadmillIncline - inclineMin)/(inclineMax - inclineMin);
        
        incline_scaled = treadmillIncline;

        
        
        [rows,cols_torque] = size(torque_data);
        [rows,cols_time] = size(time_data);
        cols = min(cols_torque, cols_time);
        
        A_dataport_mat = zeros(cols * 150,4*16);
        b_dataport_torque_mat = zeros(cols * 150,1);
        
        for k = 1:cols

            torque_data_col = torque_data(:,k);
            
            time_data_col = time_data(:,k);
            
            stepDuration = time_data_col(end) - time_data_col(1);
            phaseDot = 1/stepDuration;
            stepLength = treadmillSpeed*stepDuration;
            
%             stepLength_scaled = (stepLength - stepMin)/(stepMax - stepMin);
            
            stepLength_scaled = stepLength;

            
            phase_col = time_data_col/stepDuration;
            
            
            
            for phase_idx = 1:length(phase_col)
                mat_idx = phase_idx + (k - 1)*150;
                phase_j = phase_col(phase_idx);

        %         bezier_data = [((1 - phase_i).^3), (3 * ((1 - phase_i).^2 ).*(phase_i) ), ...
        %             (3 * ((1 - phase_i) ).*((phase_i).^2) ), ((phase_i).^3)];

                bezier_coeffs = returnBezierEval3P_sL_incline(phase_j,stepLength_scaled,incline_scaled);

                if phase_j <= phaseDelins(1)
                    bezier_row = [bezier_coeffs, zeros(1,3*16)];

                elseif phase_j <= phaseDelins(2)
                    bezier_row = [zeros(1,1*16),bezier_coeffs, zeros(1,2*16)];
                elseif phase_j <= phaseDelins(3)
                    bezier_row = [zeros(1,2*16),bezier_coeffs, zeros(1,1*16)];
                elseif phase_j <= phaseDelins(4)
                    bezier_row = [zeros(1,3*16),bezier_coeffs];

                end

                %scale row for the regression on the slopes
        %         bezier_row = [bezier_row,bezier_row];
                A_dataport_mat(mat_idx,:) = bezier_row;
                b_dataport_torque_mat(mat_idx) = torque_data_col(phase_idx);
                
        
        
            end
            
    
            
            
        end
        A_dataport_mat_master = [A_dataport_mat_master;A_dataport_mat];
        b_dataport_torque_master = [b_dataport_torque_master;b_dataport_torque_mat];
            
        
        avgStepDuration = mean(time_data(end,:) - time_data(1,:) );
        avgFootAngle = mean(torque_data,2);
        avgPhase = percent_gait';
        
        
        if strcmp(sub{i}, 'AB01')

            figure(2)
            if treadmillIncline == 10
                subplot(3,1,1)
            elseif treadmillIncline == 0
                subplot(3,1,2)
            elseif treadmillIncline == -10
                subplot(3,1,3)
            end
            
            if treadmillSpeed == 1.2
                cl = 'r';
            elseif treadmillSpeed == 1
                cl = 'b';
            elseif treadmillSpeed == 0.8
                cl = 'g';
            end
            
            
            if treadmillIncline == 10 || treadmillIncline == 0 || treadmillIncline == -10
                plot_j = plot3(avgPhase,stepLength_scaled*ones(size(avgPhase)), avgFootAngle, cl,'LineWidth',2);
                plot_j.Color(4) = 1;
            end

        end
%         pause

        
        
        
%         pause
        
        % take the mean of the remaining strides and concatenate them
        
        
%         nanmean(temp_data,2)
%         concatenated_data(:, numel(trials)*(s-1)+t ) = nanmean(temp_data,2);
    end
end

% The data in Gaitcyle always has 150 points per stride, and they are
% evenly spaced with respect to percent gait.

% figure(1)
% title([leg{1}, ' ', joint{1}, ' joint angles'])
% subplot(3,1,1)
% 
% xlabel('percent gait')
% % ylabel('Step Length (m)')
% ylabel('degrees'); 
% 
% subplot(3,1,2)
% 
% % ylabel('Step Length (m)')
% xlabel('step Length'); 
% ylabel('degrees'); 
% 
% subplot(3,1,3)
% 
% % ylabel('Step Length (m)')
% xlabel('phase_dot'); 
% ylabel('degrees'); 


figure(2)
title([leg{1}, ' ', joint{1}, ' joint angles'])
xlabel('percent gait')
ylabel('Step Length (1/s)')
zlabel('degrees'); 
view(60,60)

subplot(3,1,1)
title('10 deg incline')
view(60,60)
subplot(3,1,2)
title('0 deg incline')
view(60,60)
subplot(3,1,3)
title('-10 deg incline')
view(60,60)
% figure(3)
% title([leg{1}, ' ', 'shank', ' joint angles'])
% subplot(3,1,1)
% 
% xlabel('percent gait')
% % ylabel('Step Length (m)')
% ylabel('degrees'); 
% 
% subplot(3,1,2)
% 
% % ylabel('Step Length (m)')
% xlabel('step Length'); 
% ylabel('degrees'); 
% 
% subplot(3,1,3)
% 
% % ylabel('Step Length (m)')
% xlabel('phase_dot'); 
% ylabel('degrees'); 


figure(4)
title([leg{1}, ' ', 'shank', ' joint angles'])
xlabel('percent gait')
ylabel('Step Length (m)')
zlabel('degrees'); 
view(60,60)

subplot(3,1,1)
title('10 deg incline')
view(60,60)
subplot(3,1,2)
title('0 deg incline')
view(60,60)
subplot(3,1,3)
title('-10 deg incline')
view(60,60)

%% Regress

A_eq_bezier1 = [[returnBezierEval3P_sL_incline(phaseDelins(1),0,0), -returnBezierEval3P_sL_incline(phaseDelins(1),0,0), zeros(1,16), zeros(1,16)];...
    [returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(1),0,0), -returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(1),0,0), zeros(1,16), zeros(1,16)];...
    [zeros(1,16), returnBezierEval3P_sL_incline(phaseDelins(2),0,0), -returnBezierEval3P_sL_incline(phaseDelins(2),0,0), zeros(1,16)];...
    [zeros(1,16), returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(2),0,0), -returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(2),0,0), zeros(1,16)];...
    [zeros(1,16), zeros(1,16), returnBezierEval3P_sL_incline(phaseDelins(3),0,0), -returnBezierEval3P_sL_incline(phaseDelins(3),0,0)];...
    [zeros(1,16), zeros(1,16), returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(3),0,0), -returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(3),0,0)];...
    [-returnBezierEval3P_sL_incline(0,0,0), zeros(1,16), zeros(1,16), returnBezierEval3P_sL_incline(phaseDelins(4),0,0)];...
    [-returnBezier3P_sL_incline_DerivEval_dphase(0,0,0), zeros(1,16), zeros(1,16), returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(4),0,0)]];


A_eq_bezier2 = [[returnBezierEval3P_sL_incline(phaseDelins(1),1,0), -returnBezierEval3P_sL_incline(phaseDelins(1),1,0), zeros(1,16), zeros(1,16)];...
    [returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(1),1,0), -returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(1),1,0), zeros(1,16), zeros(1,16)];...
    [zeros(1,16), returnBezierEval3P_sL_incline(phaseDelins(2),1,0), -returnBezierEval3P_sL_incline(phaseDelins(2),1,0), zeros(1,16)];...
    [zeros(1,16), returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(2),1,0), -returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(2),1,0), zeros(1,16)];...
    [zeros(1,16), zeros(1,16), returnBezierEval3P_sL_incline(phaseDelins(3),1,0), -returnBezierEval3P_sL_incline(phaseDelins(3),1,0)];...
    [zeros(1,16), zeros(1,16), returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(3),1,0), -returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(3),1,0)];...
    [-returnBezierEval3P_sL_incline(0,1,0), zeros(1,16), zeros(1,16), returnBezierEval3P_sL_incline(phaseDelins(4),1,0)];...
    [-returnBezier3P_sL_incline_DerivEval_dphase(0,1,0), zeros(1,16), zeros(1,16), returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(4),1,0)]];

A_eq_bezier3 = [[returnBezierEval3P_sL_incline(phaseDelins(1),0,1), -returnBezierEval3P_sL_incline(phaseDelins(1),0,1), zeros(1,16), zeros(1,16)];...
    [returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(1),0,1), -returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(1),0,1), zeros(1,16), zeros(1,16)];...
    [zeros(1,16), returnBezierEval3P_sL_incline(phaseDelins(2),0,1), -returnBezierEval3P_sL_incline(phaseDelins(2),0,1), zeros(1,16)];...
    [zeros(1,16), returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(2),0,1), -returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(2),0,1), zeros(1,16)];...
    [zeros(1,16), zeros(1,16), returnBezierEval3P_sL_incline(phaseDelins(3),0,1), -returnBezierEval3P_sL_incline(phaseDelins(3),0,1)];...
    [zeros(1,16), zeros(1,16), returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(3),0,1), -returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(3),0,1)];...
    [-returnBezierEval3P_sL_incline(0,0,1), zeros(1,16), zeros(1,16), returnBezierEval3P_sL_incline(phaseDelins(4),0,1)];...
    [-returnBezier3P_sL_incline_DerivEval_dphase(0,0,1), zeros(1,16), zeros(1,16), returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(4),0,1)]];


A_eq_bezier4 = [[returnBezierEval3P_sL_incline(phaseDelins(1),1,1), -returnBezierEval3P_sL_incline(phaseDelins(1),1,1), zeros(1,16), zeros(1,16)];...
    [returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(1),1,1), -returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(1),1,1), zeros(1,16), zeros(1,16)];...
    [zeros(1,16), returnBezierEval3P_sL_incline(phaseDelins(2),1,1), -returnBezierEval3P_sL_incline(phaseDelins(2),1,1), zeros(1,16)];...
    [zeros(1,16), returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(2),1,1), -returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(2),1,1), zeros(1,16)];...
    [zeros(1,16), zeros(1,16), returnBezierEval3P_sL_incline(phaseDelins(3),1,1), -returnBezierEval3P_sL_incline(phaseDelins(3),1,1)];...
    [zeros(1,16), zeros(1,16), returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(3),1,1), -returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(3),1,1)];...
    [-returnBezierEval3P_sL_incline(0,1,1), zeros(1,16), zeros(1,16), returnBezierEval3P_sL_incline(phaseDelins(4),1,1)];...
    [-returnBezier3P_sL_incline_DerivEval_dphase(0,1,1), zeros(1,16), zeros(1,16), returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(4),1,1)]];


% additional constraints for foot
% A_eq_bezier_footL1 = [[returnBezierEval3P_sL_incline(phaseDelins(1),0,0), zeros(1,16), zeros(1,16), zeros(1,16)];...
%     [returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(1),0,0), zeros(1,16), zeros(1,16), zeros(1,16)];...
%     [zeros(1,16), returnBezierEval3P_sL_incline(phaseDelins(2),0,0), zeros(1,16), zeros(1,16)];...
%     [zeros(1,16), returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(2),0,0), zeros(1,16), zeros(1,16)];...
%     [zeros(1,16), zeros(1,16), returnBezierEval3P_sL_incline(phaseDelins(3),0,0), zeros(1,16)];...
%     [zeros(1,16), zeros(1,16), returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(3),0,0), zeros(1,16)];...
%     [zeros(1,16), zeros(1,16), zeros(1,16), returnBezierEval3P_sL_incline(phaseDelins(4),0,0)];...
%     [zeros(1,16), zeros(1,16), zeros(1,16), returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(4),0,0)]];
% 
% 
% A_eq_bezier_footL2 = [[returnBezierEval3P_sL_incline(phaseDelins(1),0,10), zeros(1,16), zeros(1,16), zeros(1,16)];...
%     [returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(1),0,10), zeros(1,16), zeros(1,16), zeros(1,16)];...
%     [zeros(1,16), returnBezierEval3P_sL_incline(phaseDelins(2),0,10), zeros(1,16), zeros(1,16)];...
%     [zeros(1,16), returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(2),0,10), zeros(1,16), zeros(1,16)];...
%     [zeros(1,16), zeros(1,16), returnBezierEval3P_sL_incline(phaseDelins(3),0,10), zeros(1,16)];...
%     [zeros(1,16), zeros(1,16), returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(3),0,10), zeros(1,16)];...
%     [zeros(1,16), zeros(1,16), zeros(1,16), returnBezierEval3P_sL_incline(phaseDelins(4),0,10)];...
%     [zeros(1,16), zeros(1,16), zeros(1,16), returnBezier3P_sL_incline_DerivEval_dphase(phaseDelins(4),0,10)]];

% A_eq_bezier_zerosL3 = [[zeros(1,16),returnBezierEval3P_sL_incline(0.2,2,10), zeros(1,16), zeros(1,16)];...
%     [zeros(1,16),returnBezierEval3P_sL_incline(0.2,2,0), zeros(1,16), zeros(1,16)];...
%     [zeros(1,16),returnBezierEval3P_sL_incline(0.2,2,-10), zeros(1,16), zeros(1,16)]];

% A_eq_bezier_footL4 = [[zeros(1,16),returnBezier3P_sL_incline_DerivEval_dsL(0.2,2,10), zeros(1,16), zeros(1,16)];...
%     [zeros(1,16),returnBezier3P_sL_incline_DerivEval_dsL(0.2,2,0), zeros(1,16), zeros(1,16)];...
%     [zeros(1,16),returnBezier3P_sL_incline_DerivEval_dsL(0.2,2,-10), zeros(1,16), zeros(1,16)]];

% A_eq_foot = [A_eq_bezier1;A_eq_bezier2;A_eq_bezier3;A_eq_bezier4;A_eq_bezier_footL1;A_eq_bezier_footL2;A_eq_bezier_footL4];
A_eq_torque = [A_eq_bezier1;A_eq_bezier2;A_eq_bezier3;A_eq_bezier4];

% b_eq_foot1 = zeros(8,1)
% b_eq_foot2 = zeros(8,1)
% b_eq_foot2(1:2:8) = 10 
% b_eq_foot3 = [10;0;-10];
% b_eq_foot4 = zeros(3,1);
% 
% 
% b_eq_foot = [zeros(4*8,1);b_eq_foot1;b_eq_foot2;b_eq_foot4];
b_eq_torque = [zeros(4*8,1)];


% scale for regression against step length

best_fit_params_torque = lsqlin(A_dataport_mat_master,b_dataport_torque_master,[],[],A_eq_torque, b_eq_torque);

phase = linspace(0,1,50)';
% incline = 0:0.1:1;
stepLength = 0:0.1:2;
% stepLength_k = 1;


[X,Y] = meshgrid(phase,stepLength);

best_fit_torque_mat_311 = zeros(size(X));

best_fit_torque_mat_312 = zeros(size(X));

best_fit_torque_mat_313 = zeros(size(X));

for i = 1:length(phase)
    
    for j = 1:length(stepLength)
%         i
%         j
        phase_i = X(j,i);
        stepLength_j = Y(j,i);
        best_fit_torque_mat_311(j,i) = returnPiecewiseBezier3D(phase_i,stepLength_j,10,best_fit_params_torque, phaseDelins);
        
        best_fit_torque_mat_312(j,i) = returnPiecewiseBezier3D(phase_i,stepLength_j,0,best_fit_params_torque, phaseDelins);
        
        best_fit_torque_mat_313(j,i) = returnPiecewiseBezier3D(phase_i,stepLength_j,-10,best_fit_params_torque, phaseDelins);
        

    end
    
    
end

figure(2)
subplot(3,1,1)
surf(phase, stepLength,best_fit_torque_mat_311)
subplot(3,1,2)
surf(phase, stepLength,best_fit_torque_mat_312)
subplot(3,1,3)
surf(phase, stepLength,best_fit_torque_mat_313)


%% plot against incline

phase = linspace(0,1,50)';
incline = -10:1:10;
stepLength_k = 1;


[X,Y] = meshgrid(phase,incline);

best_fit_torque_mat_incline = zeros(size(X));

for i = 1:length(phase)
    
    for j = 1:length(incline)
%         i
%         j
        phase_i = X(j,i);
        incline_j = Y(j,i);
        best_fit_torque_mat_incline(j,i) = returnPiecewiseBezier3D(phase_i,stepLength_k,incline_j,best_fit_params_torque, phaseDelins);


    end
    
    
end



figure(5)
surf(phase, incline,best_fit_torque_mat_incline)
title('Torque')

xlabel('percent gait')
ylabel('Incline, scaled (deg)')
zlabel('torque (N-m)'); 
view(60,60)



%% save variables


save('torqueProfileCoeffs_dataport_3P','A_dataport_mat_master','b_dataport_torque_master','best_fit_params_torque')
M = [best_fit_params_torque'];
writematrix(M,'torqueProfileCoeffs_dataport3P.csv')
%% Helper function
function [speed, incline] = returnSpeedIncline(trialString)



d_idx = strfind(trialString,'d');
i_idx = strfind(trialString,'i');

isIncline = ~isempty(i_idx) & isempty(d_idx);
isDecline = isempty(i_idx) & ~isempty(d_idx);

mid_idx = [i_idx,d_idx];

speedString = trialString(2:mid_idx-1);

inclineString = trialString(mid_idx+1:end);



switch speedString
            
    case '0x8'
        speed = 0.8;

    case '1'
        speed = 1;

    case '1x2'
        speed = 1.2;

end


switch inclineString
            
    case '10'
        incline = 10;

    case '7x5'
        incline = 7.5;

    case '5'
        incline = 5;
        
    case '2x5'
        incline = 2.5;
        
    case '0'
        incline = 0;
        

end
if isDecline
    incline = incline * -1;
end



end
