%%
clear()
global Kp Kd Kdd mass K_prop T_prop C_d tau_opti tau_imu tau_prop ref_val init_val bias_val square_val square_freq sin_val sin_f dist_p max_f sigma_acc sigma_vel sigma_h 

%Configure Nominal Values

%identified model:
%third order model with time delay and single integrator
% TF = 100.3270 * e(-0.0709s) / ((3.226 s + 1) (0.0988 s + 1) s)

%reconfiguring the parameters to match the simulink model
%Simulation\HeightModel.slx
mass_nominal = 1.79;%from previous knowledge from the system
C_d_nominal = mass_nominal/(2*3.226);
K_prop_nominal = 100.3270*mass_nominal/(3.226);
T_prop_nominal = 0.0988;

%based on the identified model, time delays cannot be seperately
%identified, therefore following assumptions will be made:
%in the identified model, assume all the delay was cause by the optitrack
%as it was the main source of data feedback in the loop
%ignore propellant time delay
%assume a small value for imu delay
tau_opti_nominal = 0.0709;
tau_imu_nominal = 0;
tau_prop_nominal = 0.00;

%noise 
sigma_acc = 0;
sigma_vel = 0;
sigma_h = 0.001;

%%
%mrft parameters
beta_mrft = -0.8;
h_mrft = 0.1;

%%
%simulation parameter
time_step = 0.001;
NN_time_step = 0.05;
t_final = 35;
N_timesteps = floor(t_final/time_step) + 1; %number of timesteps
NN_N_timesteps = floor(NN_time_step/time_step) + 1;
%%
%Configure optimization range and grid

%Percentage and step size of varying for each parameter
K_prop_range = 15; %range corresponds to +-15% of nominal value 
K_prop_step = 30; %step size in percentage of nominal value

T_prop_range = 15; %range corresponds to +-15% of nominal value 
T_prop_step = 30; %step size in percentage of nominal value

C_d_range = 0; %range corresponds to +-15% of nominal value 
C_d_step = 30; %step size in percentage of nominal value

%all time delays will be varied the same way
tau_opti_range = 15; %range corresponds to +-15% of nominal value 
tau_opti_step = 30; %step size in percentage of nominal value

%define parameters grid based on the range and step size defined above
K_prop_grid = (K_prop_nominal - K_prop_range*K_prop_nominal / 100) : (K_prop_step*K_prop_nominal/100) : (K_prop_nominal + K_prop_range*K_prop_nominal / 100);
T_prop_grid = (T_prop_nominal - T_prop_range*T_prop_nominal / 100) : (T_prop_step*T_prop_nominal/100) : (T_prop_nominal + T_prop_range*T_prop_nominal / 100);
C_d_grid = (C_d_nominal - C_d_range*C_d_nominal / 100) : (C_d_step*C_d_nominal/100) : (C_d_nominal + C_d_range*C_d_nominal / 100);
tau_opti_grid = (tau_opti_nominal - tau_opti_range*tau_opti_nominal / 100) : (tau_opti_step*tau_opti_nominal/100) : (tau_opti_nominal + tau_opti_range*tau_opti_nominal / 100);

N_data_points = length(K_prop_grid) * length(T_prop_grid) * length(C_d_grid) * length(tau_opti_grid); %number of data points
sim_per_model = 30; %number of times each parameter set is simulated
N_test_per_model = 5;

%%
%values to pass to NN 
Xtrain = zeros(N_timesteps, 1, 2, N_data_points*sim_per_model);
Ytrain = zeros(N_data_points*sim_per_model, 1);

%values to test
Xtest = zeros(N_timesteps, 1, 2, N_data_points*N_test_per_model);
Ytest_truth = zeros(N_data_points*N_test_per_model, 1);
%%
%iterate, simulate, and log simulation for all cases in the grid

iterator = 1;

%iterate over range of parameters
mass = mass_nominal;
for i=1:length(K_prop_grid)
    K_prop = K_prop_grid(i);
    
    for j=1:length(T_prop_grid)
        T_prop = T_prop_grid(j);
        
        for k=1:length(C_d_grid)
            C_d = C_d_grid(k);
            
            for w=1:length(tau_opti_grid)
                %lumped time delay assumption
                tau_opti = tau_opti_grid(w);
                tau_imu = tau_imu_nominal;
                tau_prop = tau_prop_nominal;
                
                disp('simulation start')
                disp('iteration')
                disp(iterator)
                
                %start simulation
                %initial and reference value  
                init_val = 0;
                ref_val = 1;
                %measurement noise (set to zero)
                sigma_h = 0*0.01^2; 
                
                for sim_i=1:sim_per_model
                    bias_val = (2*rand()-1) * 7;
                    %run simulation and log      
                    options = simset('SrcWorkspace','current','DstWorkspace','current','SignalLoggingName','logged_data');
                    simOut = sim('HeightModel_mrft',[],options);
                    Height_noise = logged_data.get('Height_noise');
                    %val_height_noise=Height_noise.Data;
                    val_height_noise=Height_noise.Values.Data;
                    u_tot = logged_data.get('u_tot');
                    %val_u=u.Data;
                    val_u_tot=u_tot.Values.Data;
                    
                    %append
                    sample_index = (iterator-1)*sim_per_model + sim_i;
                    Xtrain(:,1,1,sample_index) = val_height_noise;
                    Xtrain(:,1,2,sample_index) = val_u_tot;
                    Ytrain(sample_index, 1) = iterator;
                end
                
                for sim_i=1:N_test_per_model
                    bias_val = (2*rand()-1) * 7;                
                    %simulate one more time for testing
                    options = simset('SrcWorkspace','current','DstWorkspace','current','SignalLoggingName','logged_data');
                    simOut = sim('HeightModel_mrft.slx',[],options);
                    Height_noise = logged_data.get('Height_noise');
                    %val_height_noise=Height_noise.Data;
                    val_height_noise=Height_noise.Values.Data;
                    u_tot = logged_data.get('u_tot');
                    %val_u=u.Data;
                    val_u_tot=u_tot.Values.Data;

                    %append;
                    sample_index = (iterator-1)*N_test_per_model + sim_i;
                    Xtest(:,1,1,sample_index) = val_height_noise;
                    Xtest(:,1,2,sample_index) = val_u_tot;
                    Ytest_truth(sample_index, 1) = iterator;
                end
                
                iterator = iterator + 1;
                            
            end
        end
    end
end


%%
%downsample training data
%values to pass to NN 
Xtrain = Xtrain(1:floor(NN_time_step/time_step):end, :, :, :);

%values to test
Xtest = Xtest(1:floor(NN_time_step/time_step):end, :, :, :);

%%
%plot training data
figure()
dim = ceil(sqrt(N_data_points));
for i=1:(dim*dim)
    subplot(dim, dim, i)
    plot(Xtrain(:,1,1,(i-1)*sim_per_model+1))
    title(Ytrain((i-1)*sim_per_model+1,1))
end
%%
%save variables
save('training_set', 'Xtrain', 'Ytrain')
save('testing_set', 'Xtest', 'Ytest_truth')
%%
%divide training data into specific segments
dt_seg = 10;%size of dt segment
stride_seg = 10;%stride between segment centers
t_start = 25; %start time for segment
t_end = t_final; %end time for segment

N_timestep_seg = (dt_seg / NN_time_step) + 1; %number of timesteps per segment
N_seg = floor(N_data_points*sim_per_model*(t_end-t_start)/stride_seg); %total number of segments

Xtrain_seg = zeros(N_timestep_seg, 1, 2, N_seg);
Ytrain_seg = zeros(N_seg, 1);


segment_index = 1;
for i=1:size(Xtrain,4)
    %generate segments
    
    t_segment_start = t_start/NN_time_step + 1;
    
    for j=1:floor((t_end-t_start)/stride_seg)
        Xtrain_seg(:,1,:,segment_index) = Xtrain(t_segment_start:(t_segment_start+N_timestep_seg-1), 1, :, i);
        Ytrain_seg(segment_index,1) = Ytrain(i, 1);
        
        segment_index = segment_index + 1;
        t_segment_start = floor(t_segment_start + stride_seg/NN_time_step);
    end
end

segment_index = 1;
for i=1:size(Xtest,4)
    %generate segments
    
    t_segment_start = t_start/NN_time_step + 1;
    
    for j=1:floor((t_end-t_start)/stride_seg)
        Xtest_seg(:,1,:,segment_index) = Xtest(t_segment_start:(t_segment_start+N_timestep_seg-1), 1, :, i);
        Ytest_truth_seg(segment_index,1) = Ytest_truth(i, 1);
        
        segment_index = segment_index + 1;
        t_segment_start = floor(t_segment_start + stride_seg/NN_time_step);
    end
end
%%
%plot segmented training data
figure()
dim = ceil(sqrt(N_data_points));
for i=1:(dim*dim)
    subplot(dim, dim, i)
    plot(Xtrain_seg(:,1,1,(i-1)*floor(sim_per_model*(t_end-t_start)/stride_seg)+1))
    title(Ytrain_seg((i-1)*floor(sim_per_model*(t_end-t_start)/stride_seg)+1,1))
end

%%
%save variables 
save('training_set_seg', 'Xtrain_seg', 'Ytrain_seg')
save('testing_set_seg', 'Xtest_seg', 'Ytest_truth_seg')
%%
%train NN on full timeseries
options = trainingOptions('adam', 'Plots', 'training-progress','Shuffle','every-epoch','MaxEpochs', 120, 'MiniBatchSize',32, 'InitialLearnRate', 0.0005);
trained_network = trainNetwork(Xtrain, categorical(Ytrain), lgraph_1, options)

%save models
save('trained_model', 'lgraph_1', 'trained_network')
%%
%train NN on segmented timeseries
options = trainingOptions('adam', 'Plots', 'training-progress','Shuffle','every-epoch','MaxEpochs', 120, 'MiniBatchSize',32, 'InitialLearnRate', 0.001);
trained_network_seg = trainNetwork(Xtrain_seg, categorical(Ytrain_seg), lgraph_1_seg, options)

%save models
save('trained_model_seg', 'lgraph_1_seg', 'trained_network_seg')
%%
%run testing set on full timeseries
Ytest_resuts = classify(trained_network, Xtest)
Ytest_activations = activations(trained_network, Xtrain, 'softmax');
%%
%run testing set on segment timeseries
Ytest_seg_resuts = classify(trained_network_seg, Xtest_seg)
Ytest_seg_activations = activations(trained_network_seg, Xtest_seg, 'softmax');