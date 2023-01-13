close all;clc;clearvars -except Gaitcycle Continuous %removes all variables except for gaitcyle and continuous from the matlab workspace. Useful because loading these variables can take several minutes.
%load the InclineExperiment data from the folder location you specify
% load('Z:\your_file_location_here\InclineExperiment.mat') 
%This file is large and may take several minutes to load. We recommend not
%clearing the variable, and only loading it as many times as necessary.

%% Example: removing strides with outliers from kinematic data, and taking the mean

% load('regressionMatrices_dataport_3P.mat')
load('regressionMatrices_gaitModel_fourier_normalizedsL_linearsL.mat')


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

c = {'r','g','b'};
% c1 = {'b','b','b','b','b','r','r','r','r'}
% 
% c2 = [0.5,0.4,0.3,0.2,0.1,0.2,0.3,0.4,0.5]

% Initializing a matrix that will contain the mean joint data of all subjects,
%all tasks
% concatenated_data=NaN(150,numel(sub)*numel(trials));


phaseDelins = [0.1,0.5,0.65,1];

numInclineFuncs = 2;
numStepLengthFuncs = 2;
N_FOURIER = 20;
numPhaseFuncs = (length(1:1:N_FOURIER) * 2) + 1;
numFuncs = numInclineFuncs*numStepLengthFuncs*numPhaseFuncs;

%High Pass filter
s=tf('s');
omega = 0.2 * 2*pi;
zeta= 0.9;

HIGH_PASS_FILTER = (s^2)/(s^2+2*zeta*omega*s+omega^2);

[num,den] = tfdata(HIGH_PASS_FILTER);
[A_HPF,B_HPF,C_HPF,D_HPF] = tf2ss(num{1},den{1});
dT = 1/100;

Ad_HPF = expm(A_HPF*dT);
Bd_HPF = (Ad_HPF - eye(2)) * A_HPF^-1 *B_HPF;
HP_filter_states_x0_heel_y = [-0.300;-0.300;];
HP_filter_states_x0_heel_z = [-0.300;-0.300;];

phase = linspace(0,1,150);

A_covar_mat_master = zeros(150,1);
b_covar_11_master = zeros(150,1);
b_covar_12_master = zeros(150,1);
b_covar_13_master = zeros(150,1);
b_covar_14_master = zeros(150,1);
b_covar_15_master = zeros(150,1);
b_covar_16_master = zeros(150,1);

b_covar_22_master = zeros(150,1);
b_covar_23_master = zeros(150,1);
b_covar_24_master = zeros(150,1);
b_covar_25_master = zeros(150,1);
b_covar_26_master = zeros(150,1);

b_covar_33_master = zeros(150,1);
b_covar_34_master = zeros(150,1);
b_covar_35_master = zeros(150,1);
b_covar_36_master = zeros(150,1);

b_covar_44_master = zeros(150,1);
b_covar_45_master = zeros(150,1);
b_covar_46_master = zeros(150,1);

b_covar_55_master = zeros(150,1);
b_covar_56_master = zeros(150,1);

b_covar_66_master = zeros(150,1);


C_covars = zeros(6,6,150);
x_mean = zeros(6,1,150);

kk = 0;

isPosDef = zeros(150,1);


    
N = 0;
X_meas = [];


for i = 1:length(sub) %loop through all subjects in 'sub'
    switch sub{i}

        case 'AB01'
            footAngleZero = 85.46; %subject 1

        case 'AB02'
            footAngleZero = 85.46 + 4.506 + 1.62; %subject 2

        case 'AB03'
            footAngleZero = 85.46 + 5.564; %subject 3
        case 'AB04'
            footAngleZero = 85.46 + 7.681; %subject 4
        case 'AB05'
            footAngleZero = 85.46 + 5.405; %subject 5
        case 'AB06'
            footAngleZero = 85.46 + 4.089; %subject 6
        case 'AB07'
            footAngleZero = 85.46 + 1.523; %subject 7
        case 'AB08'
            footAngleZero = 85.46 + 3.305; %subject 8
        case 'AB09'
            footAngleZero = 85.46 + 4.396; %subject 9
        case 'AB10'
            footAngleZero = 85.46 + 6.555; %subject 10

    end


    subjLegLength = Gaitcycle.(sub{i}).subjectdetails{5,2}/1000;

    for j = 1:length(trials) %loop through all trials in 'trial'
        % store the kinematic data in a temporary variable
%         sub{i}
        trial = trials{j};
        [treadmillSpeed, treadmillIncline] = returnSpeedIncline(trial)
        foot_angle_data = Gaitcycle.(sub{i}).(trials{j}).kinematics.jointangles.(leg{1}).('foot').x;
        % adjust
        foot_angle_data = -(foot_angle_data + footAngleZero);

        ankle_angle_data = Gaitcycle.(sub{i}).(trials{j}).kinematics.jointangles.(leg{1}).('ankle').x;

        % get Heel Marker
        heel_pos_x_data = Gaitcycle.(sub{i}).(trials{j}).kinematics.markers.(leg{1}).('heel').x/ (1000);
        heel_pos_y_data = Gaitcycle.(sub{i}).(trials{j}).kinematics.markers.(leg{1}).('heel').y/ (1000);
        heel_pos_z_data = Gaitcycle.(sub{i}).(trials{j}).kinematics.markers.(leg{1}).('heel').z/ (1000);


        %adjust for treadmill frame

        if treadmillIncline < 0
            heel_pos_y_data = -heel_pos_y_data;
        end


        time_data = Gaitcycle.(sub{i}).(trials{j}).cycles.(leg{1}).time;

        % delete the strides identified to contain outliers
        foot_angle_data(:,Gaitcycle.(sub{i}).(trials{j}).stepsout.(leg{1})) = [];
        time_data(:,Gaitcycle.(sub{i}).(trials{j}).stepsout.(leg{1})) = [];
        ankle_angle_data(:,Gaitcycle.(sub{i}).(trials{j}).stepsout.(leg{1})) = [];
        heel_pos_x_data(:,Gaitcycle.(sub{i}).(trials{j}).stepsout.(leg{1})) = [];
        heel_pos_y_data(:,Gaitcycle.(sub{i}).(trials{j}).stepsout.(leg{1})) = [];
        heel_pos_z_data(:,Gaitcycle.(sub{i}).(trials{j}).stepsout.(leg{1})) = [];

        shank_angle_data = foot_angle_data - ankle_angle_data;

        incline_scaled = treadmillIncline;

        [rows,cols] = size(foot_angle_data);

        for k = 1:cols
            N = N + 1;
            kk = kk + 1;

            foot_angle_data_col = foot_angle_data(:,k);
            shank_angle_data_col = shank_angle_data(:,k);

            heel_pos_y_data_col = heel_pos_y_data(:,k);
            heel_pos_y_data_col_filt = zeros(size(heel_pos_y_data_col));

            heel_pos_z_data_col = heel_pos_z_data(:,k);
            heel_pos_z_data_col_filt = zeros(size(heel_pos_z_data_col));

            states_heel_y = zeros(2,150);
            states_heel_z = zeros(2,150);
            for ii = 1:150

                
                if ii == 1
                    
                    states_heel_y(:,ii) = (Ad_HPF) * HP_filter_states_x0_heel_y + Bd_HPF * heel_pos_y_data_col(ii);
                    states_heel_z(:,ii) = (Ad_HPF) * HP_filter_states_x0_heel_z + Bd_HPF * heel_pos_z_data_col(ii);
                    heel_pos_y_data_col_filt(ii) = heel_pos_y_data_col(ii);
                    heel_pos_z_data_col_filt(ii) = heel_pos_z_data_col(ii);

                else

                    states_heel_y(:,ii) = (Ad_HPF) * states_heel_y(:,ii-1) + Bd_HPF * heel_pos_y_data_col(ii-1);
                    states_heel_z(:,ii) = (Ad_HPF) * states_heel_z(:,ii-1) + Bd_HPF * heel_pos_z_data_col(ii-1);
                    
                    heel_pos_y_data_col_filt(ii) = C_HPF * states_heel_y(:,ii-1) + D_HPF * heel_pos_y_data_col(ii-1);
                    heel_pos_z_data_col_filt(ii) = C_HPF * states_heel_z(:,ii-1) + D_HPF * heel_pos_z_data_col(ii-1);


                end



            end
            heel_pos_y_data_col_filt(1) = heel_pos_y_data_col_filt(2);
            HP_filter_states_x0_heel_y = states_heel_y(:,end);

            heel_pos_z_data_col_filt(1) = heel_pos_z_data_col_filt(2);
            HP_filter_states_x0_heel_z = states_heel_z(:,end);
            
            time_data_col = time_data(:,k);

                        
%             figure(100)
%         
%             plot(time_data_col, heel_pos_y_data_col_filt,'r')
%             hold on
%             plot(time_data_col, heel_pos_y_data_col,'b')
% %             plot(time_data_col, HP_filter_states_x0_heel_y)
%             
%             pause

            % velocities
            foot_vel_data_col = diff(foot_angle_data_col)./diff(time_data_col);
            foot_vel_data_col = [foot_vel_data_col(1);foot_vel_data_col];

%             foot_vel_data_col = lowpass(foot_vel_data_col,90,100);

            shank_vel_data_col = diff(shank_angle_data_col)./diff(time_data_col);
            shank_vel_data_col = [shank_vel_data_col(1);shank_vel_data_col];

%             shank_vel_data_col = lowpass(shank_vel_data_col,90,100);
            
            heel_pos_y_data_col = heel_pos_y_data_col_filt;
            heel_pos_z_data_col = heel_pos_z_data_col_filt;
            
            stepDuration = time_data_col(end) - time_data_col(1);
            phaseDot = 1/stepDuration;
            stepLength = treadmillSpeed*stepDuration;


            stepLength_scaled = stepLength/subjLegLength;


            phase_col = time_data_col/stepDuration;
            for phase_idx = 1:150

                phase_j = phase_col(phase_idx);

                footAngle_measured = foot_angle_data_col(phase_idx);
                shankAngle_measured = shank_angle_data_col(phase_idx);

                footAngleVel_measured = foot_vel_data_col(phase_idx);
                shankAngleVel_measured = shank_vel_data_col(phase_idx);

                heelPosForward_measured = heel_pos_y_data_col(phase_idx);
                heelPosUp_measured = heel_pos_z_data_col(phase_idx);


                footAngle_model = best_fit_params_footAngle' * returnFourierBasis_Eval(phase_j,stepLength_scaled,incline_scaled, N_FOURIER)';
                shankAngle_model = best_fit_params_shankAngle' * returnFourierBasis_Eval(phase_j,stepLength_scaled,incline_scaled, N_FOURIER)';

                footAngleVel_model = phaseDot * best_fit_params_footAngle' * returnFourierBasis_DerivEval_dphase(phase_j,stepLength_scaled,incline_scaled, N_FOURIER)';
                shankAngleVel_model = phaseDot * best_fit_params_shankAngle' * returnFourierBasis_DerivEval_dphase(phase_j,stepLength_scaled,incline_scaled, N_FOURIER)';

                heelPosForward_model = best_fit_params_heelPosForward' * returnFourierBasis_Eval(phase_j,stepLength_scaled,incline_scaled, N_FOURIER)';
                heelPosUp_model = best_fit_params_heelPosUp' * returnFourierBasis_Eval(phase_j,stepLength_scaled,incline_scaled, N_FOURIER)';


                footAngle_residual = footAngle_measured - footAngle_model;
                shankAngle_residual = shankAngle_measured - shankAngle_model;

                footAngleVel_residual = footAngleVel_measured - footAngleVel_model;
                shankAngleVel_residual = shankAngleVel_measured - shankAngleVel_model;

                heelPosForward_residual = heelPosForward_measured - heelPosForward_model;
                heelPosUp_residual = heelPosUp_measured - heelPosUp_model;

                sample_mean = [footAngle_residual, footAngleVel_residual, ...
                    shankAngle_residual,shankAngleVel_residual,...
                    heelPosForward_residual, heelPosUp_residual]';
                
                % recursively update mean and covariance
                if kk == 0
                    x_mean(:,:,phase_idx) = sample_mean;
                    C_covars(:,:,phase_idx) = sample_mean * sample_mean';
                    
                    
                else
                    prev_x_mean = x_mean(:,:,phase_idx);
                    x_mean(:,:,phase_idx) = (kk/(kk + 1))*prev_x_mean + (1/(kk + 1))*sample_mean;
                    
                    
                    C_covars(:,:,phase_idx) = ((kk - 1)/kk)*C_covars(:,:,phase_idx) ...
                        + (prev_x_mean * prev_x_mean') ...
                        + (1/kk) * (sample_mean * sample_mean')...
                        - ((kk + 1)/kk) * (x_mean(:,:,phase_idx) * x_mean(:,:,phase_idx)');
                    
                end
                
            end



        end

    end
end
    
    
C_covars = C_covars + repmat(1e-6 * eye(6,6),1,1,150);
%     pause
    
%%
b_covar_11_master = reshape(C_covars(1,1,:),[],1);
b_covar_12_master = reshape(C_covars(1,2,:),[],1);
b_covar_13_master = reshape(C_covars(1,3,:),[],1);
b_covar_14_master = reshape(C_covars(1,4,:),[],1);
b_covar_15_master = reshape(C_covars(1,5,:),[],1);
b_covar_16_master = reshape(C_covars(1,6,:),[],1);

b_covar_22_master = reshape(C_covars(2,2,:),[],1);
b_covar_23_master = reshape(C_covars(2,3,:),[],1);
b_covar_24_master = reshape(C_covars(2,4,:),[],1);
b_covar_25_master = reshape(C_covars(2,5,:),[],1);
b_covar_26_master = reshape(C_covars(2,6,:),[],1);

b_covar_33_master = reshape(C_covars(3,3,:),[],1);
b_covar_34_master = reshape(C_covars(3,4,:),[],1);
b_covar_35_master = reshape(C_covars(3,5,:),[],1);
b_covar_36_master = reshape(C_covars(3,6,:),[],1);

b_covar_44_master = reshape(C_covars(4,4,:),[],1);
b_covar_45_master = reshape(C_covars(4,5,:),[],1);
b_covar_46_master = reshape(C_covars(4,6,:),[],1);

b_covar_55_master = reshape(C_covars(5,5,:),[],1);
b_covar_56_master = reshape(C_covars(5,6,:),[],1);

b_covar_66_master = reshape(C_covars(6,6,:),[],1);

for i =1:150
    
    
[~,p] = chol(  reshape( C_covars(:,:,i) ,6,6,[]) );

isPosDef(i) = p == 0;
end



%%

% 
% b_covar_12_master = lowpass(b_covar_12_master,10,120);
% b_covar_14_master = lowpass(b_covar_14_master,10,120);
% b_covar_22_master = lowpass(b_covar_22_master,10,120);
% b_covar_24_master = lowpass(b_covar_24_master,10,120);
% b_covar_34_master = lowpass(b_covar_34_master,10,120);
% 
% b_covar_44_master = lowpass(b_covar_44_master,10,120);



figure(1)
subplot(6,1,1)
semilogy(phase, b_covar_11_master,'LineWidth',2)
hold on
legend('11')


subplot(6,1,2)
semilogy(phase, b_covar_22_master,'LineWidth',2)
hold on
legend('22')

subplot(6,1,3)
semilogy(phase, b_covar_33_master,'LineWidth',2)
hold on
legend('33')


subplot(6,1,4)
semilogy(phase, b_covar_44_master,'LineWidth',2)
hold on
legend('44')

subplot(6,1,5)
semilogy(phase, b_covar_55_master,'LineWidth',2)
hold on
legend('55')

subplot(6,1,6)
semilogy(phase, b_covar_66_master,'LineWidth',2)
hold on
legend('66')


figure(2)

subplot(6,1,1)
plot(phase, b_covar_11_master,'LineWidth',2)
hold on
legend('11')


subplot(6,1,2)
plot(phase, b_covar_12_master,'LineWidth',2)
hold on
legend('12')

subplot(6,1,3)
plot(phase, b_covar_13_master,'LineWidth',2)
hold on
legend('13')


subplot(6,1,4)
plot(phase, b_covar_14_master,'LineWidth',2)
hold on
legend('14')


subplot(6,1,5)
plot(phase, b_covar_15_master,'LineWidth',2)
hold on
legend('15')


subplot(6,1,6)
plot(phase, b_covar_16_master,'LineWidth',2)
hold on
legend('16')


figure(3)

subplot(3,1,1)
plot(phase, b_covar_22_master,'LineWidth',2)
hold on
legend('22')


subplot(3,1,2)
plot(phase, b_covar_23_master,'LineWidth',2)
hold on
legend('23')

subplot(3,1,3)
plot(phase, b_covar_24_master,'LineWidth',2)
hold on
legend('24')

figure(4)

subplot(2,1,1)
plot(phase, b_covar_33_master,'LineWidth',2)
hold on
legend('33')


subplot(2,1,2)
plot(phase, b_covar_34_master,'LineWidth',2)
hold on
legend('34')


% subplot(3,1,2)
% plot(phase, b_covar_12_master,'LineWidth',2)
% hold on
% legend('12')


figure(20)
plot(phase, isPosDef,'LineWidth',2)
legend('PD test')






best_fit_11 = interp1(phase',b_covar_11_master,phase');
best_fit_12 = interp1(phase',b_covar_12_master,phase');
best_fit_22 = interp1(phase',b_covar_22_master,phase');



%%
save('covar_best_fit','b_covar_11_master','b_covar_12_master','b_covar_22_master')
M = [b_covar_11_master';b_covar_12_master';b_covar_13_master';b_covar_14_master';b_covar_15_master';b_covar_16_master';...
    b_covar_22_master';b_covar_23_master';b_covar_24_master';b_covar_25_master';b_covar_26_master';...
    b_covar_33_master';b_covar_34_master';b_covar_35_master';b_covar_36_master';...
    b_covar_44_master';b_covar_45_master';b_covar_46_master';...
    b_covar_55_master';b_covar_56_master';...
    b_covar_66_master';];
writematrix(M,'covar_fourier_normalizedsL_linearsL.csv')

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
