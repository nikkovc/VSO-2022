close all;clc;clearvars -except Gaitcycle Continuous %removes all variables except for gaitcyle and continuous from the matlab workspace. Useful because loading these variables can take several minutes.
%load the InclineExperiment data from the folder location you specify
% load('Z:\your_file_location_here\InclineExperiment.mat') 
%This file is large and may take several minutes to load. We recommend not
%clearing the variable, and only loading it as many times as necessary.


%load in Tyler data from VSPA
% load('gait_time_angle.mat')

gait_time_angle = readmatrix('gait_time_shankangle_speed_stridelength_stiffness.csv');
phase_data = gait_time_angle(:,1);
time_data = gait_time_angle(:,2);
shank_angles = gait_time_angle(:,3);
stride_lengths = gait_time_angle(:,4);
speeds = gait_time_angle(:,5);
stiffnesses = gait_time_angle(:,6);


[N_DATA,~] = size(gait_time_angle);


N_FOURIER = 20
numPhaseFuncs = (length(1:1:N_FOURIER) * 2) + 1;
numFuncs = numPhaseFuncs;

A_mat_master = zeros(N_DATA, numFuncs);
b_shankAngle_master = zeros(N_DATA, 1);


for i = 1:N_DATA

    phase_i = phase_data(i);

    fourier_coeffs = returnFourier(phase_i, N_FOURIER);

    A_mat_master(i,:) = fourier_coeffs;
    b_shankAngle_master(i) = shank_angles(i);


end

%% Regress

% % constraint to force constant behavior at sL = 0
% phase = linspace(0,1,100)';
% lenP = length(phase);
% 
% 
% A_eq_fourier_constAtZerosL = zeros(length(phase), numFuncs);
%    
%     
% for j = 1:length(phase)
%     
%     phase_j = phase(j);
%     fourier_deriv_coeffs = returnFourier(phase_j, N_FOURIER);
%     A_eq_fourier_constAtZerosL(j,:) = fourier_deriv_coeffs;
% 
% end
% 
% [rows,~] = size(A_eq_fourier_constAtZerosL);
% A_eq_shank = [A_eq_fourier_constAtZerosL];
% 
% %shank
% b_eq_zero_shank_at_zero_sL = zeros(rows,1);
% 
% b_eq_shank = [b_eq_zero_shank_at_zero_sL;];

A_eq_shank = [];
b_eq_shank = [];

% scale for regression against step length
best_fit_params_shankAngle = lsqlin(A_mat_master,b_shankAngle_master,[],[],A_eq_shank, b_eq_shank);

%% plot
phase = linspace(0,1,200)';

best_fit_shank_angle = zeros(size(phase));


for i = 1:length(phase)
    best_fit_shank_angle(i) = best_fit_params_shankAngle' * returnFourier(phase(i), N_FOURIER)';

end

figure(4)
hold on
plot(phase_data,shank_angles,'o')
plot(phase, best_fit_shank_angle,'r','LineWidth',2)
xlabel('Phase')
ylabel('Shank Angle (deg)')

%% save variables

M = [best_fit_shank_angle';...
    ];
writematrix(M,'VSPA_gait_model.csv')

