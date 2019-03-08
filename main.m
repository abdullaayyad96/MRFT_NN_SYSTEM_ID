%%
%HOW TO RUN 

%   1. Specify model parameters to run MRFT test
%       1.1) Manually Select Parameters
%                           OR
%       1.2) Load automatically discritized mesh
%
%   2. Select MRFT test parameters
%
%   3. Iterate through parameter sets and apply MRFT tests
%       3.1) For manually selected parameters
%                           OR
%       3.2) For automatically discritized mesh
%       
%   4. (optional)
%       4.1) Downsample simulation data
%       4.2) Take a specific time segment of the simulation data
%
%   5. Train the neural network
%       5.1) Train using a CNN on entire simulation data
%       5.2) Train using specified time segment of the simulation data
%       5.3) Train using LSTM
%
%   6. Run the trained network on a testing set


%%

% 1.1) Manually select model partameters
clear()
%identified model:
%third order model with time delay and single integrator
% TF = 100.3270 * e(-0.0709s) / ((3.226 s + 1) (0.0988 s + 1) s)
%Select Nominal Values
gain_nominal = 100.3270;
T_prop_nominal = 0.0988;
T_body_nominal = 3.226;
tau__nominal = 0.0709;

%Configure optimization range and grid

%Percentage and step size of each parameter for in the four dimensional
%mesh
gain_range = 50; %range corresponds to +-15% of nominal value 
gain_step = 100;  %step size in percentage of nominal value

T_prop_range = 50; %range corresponds to +-15% of nominal value 
T_prop_step = 100; %step size in percentage of nominal value

T_body_range = 0; %range corresponds to +-15% of nominal value 
T_body_step = 100; %step size in percentage of nominal value

tau_range = 50; %range corresponds to +-15% of nominal value 
tau_step = 100; %step size in percentage of nominal value

%define parameters grid based on the range and step size defined above
gain_grid = (gain_nominal - gain_range*gain_nominal / 100) : (gain_step*gain_nominal/100) : (gain_nominal + gain_range*gain_nominal / 100);
T_prop_grid = (T_prop_nominal - T_prop_range*T_prop_nominal / 100) : (T_prop_step*T_prop_nominal/100) : (T_prop_nominal + T_prop_range*T_prop_nominal / 100);
T_body_grid = (T_body_nominal - T_body_range*T_body_nominal / 100) : (T_body_step*T_body_nominal/100) : (T_body_nominal + T_body_range*T_body_nominal / 100);
tau_grid = (tau__nominal - tau_range*tau__nominal / 100) : (tau_step*tau__nominal/100) : (tau__nominal + tau_range*tau__nominal / 100);


N_mesh_points = length(gain_grid) * length(T_prop_grid) * length(T_body_grid) * length(tau_grid); %number of parameter sets in the descritized mesh



%%
% 1.2) Load automatically discritized mesh
clear()
load('discrete_mesh_100.mat')

%define parameters grid based on smart discritization
gain_grid = transpose(final_points(:, 5));
T_prop_grid = transpose(final_points(:, 6));
T_body_grid = transpose(final_points(:, 7));
tau_grid = transpose(final_points(:, 8));

N_mesh_points = size(final_points, 1);
%%
% 1.2) Load automatically discritized mesh

%define parameters grid based on smart discritization
gain_grid = transpose(final_points_filtered(:, 5));
T_prop_grid = transpose(final_points_filtered(:, 6));
T_body_grid = transpose(final_points_filtered(:, 7));
tau_grid = transpose(final_points_filtered(:, 8));

N_mesh_points = size(final_points_filtered, 1);

%%
% 2. Select Simulation Parameters

time_step = 0.0001;
t_final = 35; %final simulation time
N_timesteps = floor(t_final/time_step) + 1; %total number of steps

%mrft parameters
beta_mrft = -0.8;
h_mrft = 0.1;

N_train_per_point = 20; %number of times each mesh point is simulated for the training set
N_test_per_point = 1;%number of times each mesh point is simulated for the testing set


%initialize all training timeseries
Xtrain = zeros(N_timesteps, 1, 2, N_mesh_points*N_train_per_point);
Ytrain = zeros(N_mesh_points*N_train_per_point, 1);

%initialize all testing timeseries
Xtest = zeros(N_timesteps, 1, 2, N_mesh_points*N_test_per_point);
Ytest = zeros(N_mesh_points*N_test_per_point, 1);


%%
% 3.1) Apply MRFT test for manuallly selected parameters sets

iterator = 1;

%iterate over range of parameters

for i=1:length(gain_grid)
    gain = gain_grid(i);
    
    for j=1:length(T_prop_grid)
        T_prop = T_prop_grid(j);
        
        for k=1:length(T_body_grid)
            T_body = T_body_grid(k);
            
            for w=1:length(tau_grid)
                %lumped time delay assumption
                tau = tau_grid(w);
                
                disp('simulation start')
                disp('iteration')
                disp(iterator)
                
                %start simulation
                %initial and reference value  
                init_val = 0;
                ref_val = 1;
                
                %measurement noise 
                 sigma_h = 0.001^2; 
                
                %simulate each mesh point N times for the training set
                for sim_i=1:N_train_per_point
                    
                    %set bias
                    bias_relay = 0.5 * (2*rand()-1) * h_mrft;
                    
                    %run simulation and log      
                    options = simset('SrcWorkspace','current','DstWorkspace','current','SignalLoggingName','logged_data');
                    simOut = sim('HeightModel_mrft',[],options);
                    Height_noise = logged_data.get('Height_noise');
                    val_height_noise=Height_noise.Values.Data;
                    u = logged_data.get('u');
                    val_u=u.Values.Data;
                    
                    %append data to training set
                    sample_index = (iterator-1)*N_train_per_point + sim_i;
                    Xtrain(:,1,1,sample_index) = val_height_noise; %log height
                    Xtrain(:,1,2,sample_index) = val_u; %log controller output
                    Ytrain(sample_index, 1) = iterator;
                end
                
                %simulate each mesh point N times for the testing set
                for sim_i=1:N_test_per_point
                    bias_relay = 0.5 * (2*rand()-1) * h_mrft;                
                    
                    options = simset('SrcWorkspace','current','DstWorkspace','current','SignalLoggingName','logged_data');
                    simOut = sim('HeightModel_mrft.slx',[],options);
                    Height_noise = logged_data.get('Height_noise');
                    val_height_noise=Height_noise.Values.Data;
                    u = logged_data.get('u');
                    val_u=u.Values.Data;

                    %append data to testing set
                    sample_index = (iterator-1)*N_test_per_point + sim_i;
                    Xtest(:,1,1,sample_index) = val_height_noise; %log height
                    Xtest(:,1,2,sample_index) = val_u;  %log controller output
                    Ytest(sample_index, 1) = iterator;
                end
                
                iterator = iterator + 1;
                            
            end
        end
    end
end

%%
% 3.2) Apply MRFT test for automatically discritized mesh

iterator = 1;

%iterate over range of parameters

for i=1:N_mesh_points
    gain = gain_grid(i);
    tau = tau_grid(i);
    T_prop = T_prop_grid(i);
    T_body = T_body_grid(i);
        
    disp('simulation start')
    disp('iteration')
    disp(iterator)

    %start simulation
    %initial and reference value  
    init_val = 0;
    ref_val = 1;

    %measurement noise 
     sigma_h = 0*0.0005^2; 

    %simulate each mesh point N times for the training set
    for sim_i=1:N_train_per_point

        %set bias
        bias_relay = 0.5 * (2*rand()-1) * h_mrft;

        %run simulation and log      
        options = simset('SrcWorkspace','current','DstWorkspace','current','SignalLoggingName','logged_data');
        simOut = sim('HeightModel_mrft',[],options);
        Height_noise = logged_data.get('Height_noise');
        val_height_noise=Height_noise.Values.Data;
        %val_height_noise=Height_noise.Data;
        u = logged_data.get('u');
        val_u=u.Values.Data;
        %val_u=u.Data;
        
        %append data to training set
        sample_index = (iterator-1)*N_train_per_point + sim_i;
        Xtrain(:,1,1,sample_index) = val_height_noise; %log height
        Xtrain(:,1,2,sample_index) = val_u; %log controller output
        Ytrain(sample_index, 1) = iterator;
    end
    
    %simulate each mesh point N times for the testing set
    for sim_i=1:N_test_per_point
        bias_relay = 0.5 * (2*rand()-1) * h_mrft;               

        options = simset('SrcWorkspace','current','DstWorkspace','current','SignalLoggingName','logged_data');
        simOut = sim('HeightModel_mrft.slx',[],options);
        Height_noise = logged_data.get('Height_noise');
        val_height_noise=Height_noise.Values.Data;
        %val_height_noise=Height_noise.Data;
        u_tot = logged_data.get('u');
        val_u=u.Values.Data;
        %val_u_tot=u_tot.Data;


        %append data to testing set
        sample_index = (iterator-1)*N_test_per_point + sim_i;
        Xtest(:,1,1,sample_index) = val_height_noise; %log height
        Xtest(:,1,2,sample_index) = val_u;  %log controller output
        Ytest(sample_index, 1) = iterator;
    end

    iterator = iterator + 1;

end


%%
% 4.1) (Optional) downsample training data

% reduce the timestep of the height and controller output time series
NN_time_step = 0.0001; %time step of timeseries passed to the Deep Neural Network
NN_N_timesteps = floor(NN_time_step/time_step) + 1; %total number of steps of passed to the DNN

%training set
Xtrain = Xtrain(1:floor(NN_time_step/time_step):end, :, :, :);

%testing set
Xtest = Xtest(1:floor(NN_time_step/time_step):end, :, :, :);


%%
%plot samples from training data
figure()
dim = ceil(sqrt(N_mesh_points));
for i=1:(dim*dim)
    subplot(dim, dim, i)
    plot(Xtrain(:,1,1,(i-1)*N_train_per_point+1))
    title(Ytrain((i-1)*N_train_per_point+1,1))
end


%%
%save training and testing set
save('training_set', 'Xtrain', 'Ytrain')
save('testing_set', 'Xtest', 'Ytest')


%%

% 3.2) (Optional) Take a specific time segment of the simulation data

%Take only steady state response
t_start = 29;         %start time for segmenting timeseries
t_end = t_final;     %end time for segmenting timeseries
dt_seg = 6;        %size of each segment in seconds
stride_seg = 6;   %stride between segment centers

N_timestep_seg = (dt_seg / NN_time_step) + 1; %number of timesteps per segment

%segment training set
N_seg = floor(N_mesh_points*N_train_per_point*(t_end-t_start)/stride_seg); %total number of segments for training set
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

%segment testing set
N_seg = floor(N_mesh_points*N_test_per_point*(t_end-t_start)/stride_seg); %total number of segments for testing set
Xtest_seg = zeros(N_timestep_seg, 1, 2, N_seg);
Ytest_seg = zeros(N_seg, 1);

segment_index = 1;
for i=1:size(Xtest,4)
    %generate segments
    
    t_segment_start = t_start/NN_time_step + 1;
    
    for j=1:floor((t_end-t_start)/stride_seg)
        Xtest_seg(:,1,:,segment_index) = Xtest(t_segment_start:(t_segment_start+N_timestep_seg-1), 1, :, i);
        Ytest_seg(segment_index,1) = Ytest(i, 1);
        
        segment_index = segment_index + 1;
        t_segment_start = floor(t_segment_start + stride_seg/NN_time_step);
    end
end


%%
%plot samples from segmented training data
figure()
dim = ceil(sqrt(N_mesh_points));
for i=1:(dim*dim)
    subplot(dim, dim, i)
    plot(Xtrain_seg(:,1,1,(i-1)*floor(N_train_per_point*(t_end-t_start)/stride_seg)+1))
    title(Ytrain_seg((i-1)*floor(N_train_per_point*(t_end-t_start)/stride_seg)+1,1))
end


%%
%save segmented training and testing set
save('training_set_seg', 'Xtrain_seg', 'Ytrain_seg')
save('testing_set_seg', 'Xtest_seg', 'Ytest_seg')

%%
%Downsample segmented data

% reduce the timestep of the height and controller output time series
NN_time_step = 0.005; %time step of timeseries passed to the Deep Neural Network
NN_N_timesteps = floor(NN_time_step/time_step) + 1; %total number of steps of passed to the DNN

%training set
Xtrain_seg = Xtrain_seg(1:floor(NN_time_step/time_step):end, :, :, :);

%testing set
Xtest_seg = Xtest_seg(1:floor(NN_time_step/time_step):end, :, :, :);

%%
%take half of segment

%training set
Xtrain_seg = Xtrain_seg(end/2:end, :, :, :);

%testing set
Xtest_seg = Xtest_seg(end/2:end, :, :, :);

%%
%take last cycle 
Xtrain_cell_seg = {};
for i=1:size(Xtrain_seg, 4)
    u_temp = Xtrain_seg(:, 1, 2, i);
    t_cycle = 0;
    iterator = 0;
    for j=1:length(u_temp)-5
        if ((u_temp(end-j+1) - u_temp(end-j) > 1.9 * h_mrft) && ...
                (u_temp(end-j+1) - u_temp(end-j-1) > 1.9 * h_mrft) && ...
                (u_temp(end-j+1) - u_temp(end-j-2) > 1.9 * h_mrft) && ...
                (u_temp(end-j+1) - u_temp(end-j-3) > 1.9 * h_mrft) && ...
                (u_temp(end-j+1) - u_temp(end-j-4) > 1.9 * h_mrft))
                
            iterator = iterator + 1;
        end
        if iterator > 1
            t_cycle = length(u_temp) - j;
            break
        end
    end
    
    Xtrain_cell_seg{i, 1} = transpose( reshape(Xtrain_seg(t_cycle:end,1,:,i), [length(u_temp)-t_cycle+1, size(Xtrain_seg,3)]));
end

Xtest_cell_seg = {};
for i=1:size(Xtest_seg, 4)
    u_temp = Xtest_seg(:, 1, 2, i);
    t_cycle = 0;
    iterator = 0;
    for j=1:length(u_temp)-5
        if ((u_temp(end-j+1) - u_temp(end-j) > 1.9 * h_mrft) && ...
                (u_temp(end-j+1) - u_temp(end-j-1) > 1.9 * h_mrft) && ...
                (u_temp(end-j+1) - u_temp(end-j-2) > 1.9 * h_mrft) && ...
                (u_temp(end-j+1) - u_temp(end-j-3) > 1.9 * h_mrft) && ...
                (u_temp(end-j+1) - u_temp(end-j-4) > 1.9 * h_mrft))
                
            iterator = iterator + 1;
        end
        if iterator > 1
            t_cycle = length(u_temp) - j;
            break
        end
    end
    
    Xtest_cell_seg{i, 1} = transpose( reshape(Xtest_seg(t_cycle:end,1,:,i), [length(u_temp)-t_cycle+1, size(Xtest_seg,3)]));
end
%%
%.5.1) Train using a CNN on entire simulation data

%Load saved model / comment out if the model is being altered
% load('trained_model.mat')

options = trainingOptions('adam', 'Plots', 'training-progress','Shuffle','every-epoch','MaxEpochs', 120, 'MiniBatchSize',32, 'InitialLearnRate', 0.0005);
trained_network = trainNetwork(Xtrain, categorical(Ytrain), lgraph_1, options)

%save models
save('trained_model', 'lgraph_1', 'trained_network')

%%
%.5.2) Train using a CNN on specific time segment of simulation data

%Load saved model / comment out if the model is being altered
% load('trained_model_seg.mat')

options = trainingOptions('adam', 'Plots', 'training-progress','Shuffle','every-epoch','MaxEpochs', 100, 'LearnRateSchedule','piecewise', 'MiniBatchSize',128, 'InitialLearnRate', 0.001, 'LearnRateDropPeriod',50,'LearnRateDropFactor',0.05);
trained_network_seg = trainNetwork(Xtrain_seg, categorical(Ytrain_seg), lgraph_1_seg, options)

%save models
save('trained_model_seg', 'lgraph_1_seg', 'trained_network_seg')

%%
%.5.1) Train using LSTM 

% Load saved model / comment out if the model is being altered
% load('trained_model_lstm.mat')

%convert Xtrain to cell array
% Xtrain_cell_seg = {};
% for i=1:size(Xtrain_seg, 4)
%     %Xtrain_cell_seg{i, 1} = transpose( reshape(Xtrain_seg(:,1,:,i), [size(Xtrain_seg, 1), size(Xtrain_seg,3)]));
%     Xtrain_cell_seg{i, 1} = transpose( reshape(Xtrain_seg(end-50:end,1,:,i), [51, size(Xtrain_seg,3)]));
% end
options = trainingOptions('adam', 'Plots', 'training-progress','Shuffle','every-epoch','MaxEpochs', 100, 'MiniBatchSize',64, 'LearnRateSchedule','piecewise', 'GradientThreshold',1e5, 'SequenceLength','longest', 'InitialLearnRate', 0.001, 'LearnRateDropPeriod',10,'LearnRateDropFactor',0.9);
trained_network_lstm = trainNetwork(Xtrain_cell_seg, categorical(Ytrain_seg), layers_1_lstm, options)

%save models
save('trained_model_lstm', 'layers_1_lstm', 'trained_network_lstm')

%%
% 6. run testing set on full timeseries

Ytest_resuts = classify(trained_network, Xtest)
Ytest_activations = activations(trained_network, Xtrain, 'softmax');


%%
% run testing set on segment timeseries

Ytest_seg_resuts = classify(trained_network_seg, Xtest_seg)
Ytest_seg_activations = activations(trained_network_seg, Xtest_seg, 'softmax');

correct=0;
wrong=0;
for i=1:length(Ytest_seg)
    if (double(Ytest_seg_resuts(i)) == Ytest_seg(i))
        correct = correct+1;
    else
        wrong = wrong+1;
    end
end
accuracy = 100 * correct / (correct+wrong)
%%
% run testing set on lstm segment timeseries

Xtest_cell_seg = {};
for i=1:size(Xtest_seg, 4)
    %Xtrain_cell_seg{i, 1} = transpose( reshape(Xtrain_seg(:,1,:,i), [size(Xtrain_seg, 1), size(Xtrain_seg,3)]));
    Xtest_cell_seg{i, 1} = transpose( reshape(Xtest_seg(end-50:end,1,:,i), [51, size(Xtest_seg,3)]));
end
Ytest_lest_seg_resuts = classify(trained_network_lstm, Xtest_cell_seg)

correct=0;
wrong=0;
for i=1:length(Ytest_seg)
    if (double(Ytest_lest_seg_resuts(i)) == Ytest_seg(i))
        correct = correct+1;
    else
        wrong = wrong+1;
    end
end
accuracy = 100 * correct / (correct+wrong)