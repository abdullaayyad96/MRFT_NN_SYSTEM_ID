%%
%HOW TO RUN 

%   1. Load automatically discritized mesh
%
%   2. Select MRFT test parameters
%
%   3. Iterate through parameter sets and apply MRFT tests
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
% 1. Load automatically discritized mesh
clear()
load('discrete_mesh_normalized_10_2.mat')

%define parameters grid based on smart discritization
ratio_1_grid = transpose(final_points_normalized(randi([1, size(final_points_normalized,1)], 1, 50), 3));
ratio_2_grid = transpose(final_points_normalized(randi([1, size(final_points_normalized,1)], 1, 50), 4));

N_mesh_points = 50;%size(final_points_normalized, 1);


%%
% 2. Select Simulation Parameters

time_step = 0.001;
t_final = 35; %final simulation time
N_timesteps = floor(t_final/time_step) + 1; %total number of steps

%mrft parameters
beta_mrft = -0.73;
h_mrft = 0.1;

N_train_per_point = 1; %number of times each mesh point is simulated for the training set
N_test_per_point = 0;%number of times each mesh point is simulated for the testing set


%initialize all training timeseries
Xtrain = zeros(N_timesteps, 1, 2, N_mesh_points*N_train_per_point);
Ytrain = zeros(N_mesh_points*N_train_per_point, 1);

%initialize all testing timeseries
Xtest = zeros(N_timesteps, 1, 2, N_mesh_points*N_test_per_point);
Ytest = zeros(N_mesh_points*N_test_per_point, 1);


%%
% 3.2) Apply MRFT test for automatically discritized mesh

iterator = 1;

%iterate over range of parameters

for i=1:N_mesh_points
    gain = final_points_normalized(1,5);
    tau = (0.1-0.0005) * rand + 0.0005;%final_points_normalized(1,6);
    T_prop = ratio_1_grid(i) * tau;
    T_body = ratio_2_grid(i) * tau;
    
    if(T_prop > 0.3)
        T_prop = 0.3;
    elseif(T_prop < 0.015)
        T_prop = 0.015;
    end
    
    if(T_body > 2)
        T_body = 2;
    elseif(T_body < 0.2)
        T_body = 0.2;
    end
        
    disp('simulation start')
    disp('iteration')
    disp(iterator)

    %start simulation
    %initial and reference value  
    init_val = 0.001;
    ref_val = 0;

    %measurement noise 
     sigma_h = 0*0.0005^2; 

    %simulate each mesh point N times for the training set
    for sim_i=1:N_train_per_point

        %set bias
        bias_relay = 0*0.5 * (2*rand()-1) * h_mrft;

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
        bias_relay = 0*0.5 * (2*rand()-1) * h_mrft;               

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
NN_time_step = 0.001; %time step of timeseries passed to the Deep Neural Network
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
save('training_set_norm', 'Xtrain', 'Ytrain')
save('testing_set_norm', 'Xtest', 'Ytest')


%%

% 3.2) (Optional) Take a specific time segment of the simulation data

%Take only steady state response
t_start = 32;         %start time for segmenting timeseries
t_end = t_final;     %end time for segmenting timeseries
dt_seg = 35;        %size of each segment in seconds
stride_seg = 35;   %stride between segment centers

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
save('training_set_seg_norm', 'Xtrain_seg', 'Ytrain_seg')
save('testing_set_seg_norm', 'Xtest_seg', 'Ytest_seg')

%%
%Downsample segmented data

% reduce the timestep of the height and controller output time series
NN_time_step = 0.001; %time step of timeseries passed to the Deep Neural Network
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
%Xtrain_cell_seg = {};
t_cycle_start = zeros(1, size(Xtrain, 4));
t_cycle_end = zeros(1, size(Xtrain, 4));
longest_time = 0;
for i=1:size(Xtrain, 4)
    u_temp = Xtrain(:, 1, 2, i);
    iterator = 0;
    first_edge_detected = 0;
    for j=1:length(u_temp)-7
        if ((u_temp(end-j+1) - u_temp(end-j) > 1.95 * h_mrft) && ...
                (u_temp(end-j+1) - u_temp(end-j-1) > 1.95 * h_mrft) && ...
                (u_temp(end-j+1) - u_temp(end-j-2) > 1.95 * h_mrft) && ...
                (u_temp(end-j+1) - u_temp(end-j-3) > 1.95 * h_mrft) && ...
                (u_temp(end-j+1) - u_temp(end-j-4) > 1.95 * h_mrft) && ...
                (u_temp(end-j+1) - u_temp(end-j-5) > 1.95 * h_mrft) && ...
                (u_temp(end-j+1) - u_temp(end-j-6) > 1.95 * h_mrft))
            iterator = iterator + 1;            
        end
        
        if (iterator == 1 && first_edge_detected == 0) 
            t_cycle_end(i) = length(u_temp) - j;
            first_edge_detected = 1;
        elseif (iterator >= 2)
            t_cycle_start(i) = length(u_temp) - j;            
            break
        end
    end
    
    if longest_time < (t_cycle_end(i) - t_cycle_start(i))
        longest_time = t_cycle_end(i) - t_cycle_start(i);
    end
end

%%
% Generate data with only one cycle for each point in the mesh
Xtrain_cycle =  zeros(longest_time, 1, 2, N_mesh_points*N_train_per_point);
Ytrain_cycle = Ytrain;

for i=1:size(Xtrain, 4)
    Xtrain_cycle(end-(t_cycle_end(i)-t_cycle_start(i))+1:end, :, :, i) = Xtrain(t_cycle_start(i)+1:t_cycle_end(i), :, :, i);
end


%%
%plot samples from cycle training data
figure()
dim = ceil(sqrt(N_mesh_points));

for i=1:(dim*dim)
    subplot(dim, dim, i)
    plot(Xtrain_cycle(:,1,1,(i-1)*floor(N_train_per_point)+1))
    title(Ytrain_cycle((i-1)*floor(N_train_per_point)+1,1))
end


%%
%Scaling amplitude to one
c2 = zeros(size(Xtrain_cycle, 4));

for i=1:size(Xtrain_cycle, 4)
    h_temp = Xtrain_cycle(:, 1, 1, i);
    
    amplitude = (max(h_temp(:)) - min(h_temp(:))) / 2;
    
    c2(i) = 1 / amplitude;
    
    Xtrain_cycle(:, 1, 1, i) = h_temp / amplitude;
    
end

%plot samples from cycle training data
figure()
dim = ceil(sqrt(N_mesh_points));

for i=1:(dim*dim)
    subplot(dim, dim, i)
    plot(Xtrain_cycle(:,1,1,(i-1)*floor(N_train_per_point)+1))
    hold on    
    plot(Xtrain_cycle(:,1,2,(i-1)*floor(N_train_per_point)+1))
    
    title(Ytrain_cycle((i-1)*floor(N_train_per_point)+1,1))
end
%%
%save segmented training and testing set
save('training_set_cycle_norm', 'Xtrain_cycle', 'Ytrain_cycle')
save('testing_set_cycle_norm', 'Xtest_cycle', 'Ytest_cycle')

%%
%.5.1) Train using a CNN on entire simulation data

%Load saved model / comment out if the model is being altered
% load('trained_model_norm.mat')

options = trainingOptions('adam', 'Plots', 'training-progress','Shuffle','every-epoch','MaxEpochs', 120, 'MiniBatchSize',32, 'InitialLearnRate', 0.0005);
trained_network = trainNetwork(Xtrain, categorical(Ytrain), lgraph_1, options)

%save models
save('trained_model_norm', 'lgraph_1', 'trained_network')

%%
%.5.2) Train using a CNN on specific time segment of simulation data

%Load saved model / comment out if the model is being altered
%load('trained_model_seg_norm.mat')

options = trainingOptions('adam', 'Plots', 'training-progress','Shuffle','every-epoch','MaxEpochs', 500, 'LearnRateSchedule','none', 'MiniBatchSize',128, 'InitialLearnRate', 0.001, 'LearnRateDropPeriod',50,'LearnRateDropFactor',0.05);
trained_network_seg = trainNetwork(Xtrain_cycle, categorical(Ytrain_cycle), lgraph_1_seg, options)

%save models
save('trained_model_seg_norm', 'lgraph_1_seg', 'trained_network_seg')

%%
%.5.1) Train using LSTM 

% Load saved model / comment out if the model is being altered
% load('trained_model_lstm_norm.mat')

%convert Xtrain to cell array
% Xtrain_cell_seg = {};
% for i=1:size(Xtrain_seg, 4)
%     %Xtrain_cell_seg{i, 1} = transpose( reshape(Xtrain_seg(:,1,:,i), [size(Xtrain_seg, 1), size(Xtrain_seg,3)]));
%     Xtrain_cell_seg{i, 1} = transpose( reshape(Xtrain_seg(end-50:end,1,:,i), [51, size(Xtrain_seg,3)]));
% end
options = trainingOptions('adam', 'Plots', 'training-progress','Shuffle','every-epoch','MaxEpochs', 200, 'MiniBatchSize',128, 'LearnRateSchedule','piecewise', 'GradientThreshold',1e5, 'SequenceLength','longest', 'InitialLearnRate', 0.001, 'LearnRateDropPeriod',10,'LearnRateDropFactor',0.9);
trained_network_lstm = trainNetwork(Xtrain_cell_seg, categorical(Ytrain_seg), layers_1_lstm, options)

%save models
save('trained_model_lstm_norm', 'layers_1_lstm', 'trained_network_lstm')

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