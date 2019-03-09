%%
%HOW TO RUN 

%   1. Load automatically discritized mesh
%
%   2. Select MRFT test parameters
%
%   3. Iterate through parameter sets and apply MRFT tests
%   
%   5. Take a specific time segment of the simulation data, one cycle
%
%   5. Train the neural network
%
%   6. Run the trained network on a testing set



%%
% 1. Load automatically discritized mesh
clear()
load('discrete_mesh_normalized.mat')

%define parameters grid based on smart discritization
tau_grid = transpose(final_points_normalized(:, 4));
T_prop_grid = transpose(final_points_normalized(:, 5));
T_body_grid = transpose(final_points_normalized(:, 6));

N_mesh_points = size(final_points_normalized, 1);


%%
% 2. Select Simulation Parameters

time_step = 0.001;
t_final = 35; %final simulation time
N_timesteps = floor(t_final/time_step) + 1; %total number of steps

%mrft parameters
beta_mrft = -0.73;
h_mrft = 0.1;

N_train_per_point = 50; %number of times each mesh point is simulated for the training set
N_test_per_point = 1;%number of times each mesh point is simulated for the testing set


%initialize all training timeseries
Xtrain = zeros(N_timesteps, 1, 2, N_mesh_points*N_train_per_point);
Ytrain = zeros(N_mesh_points*N_train_per_point, 1);

%initialize all testing timeseries
Xtest = zeros(N_timesteps, 1, 2, N_mesh_points*N_test_per_point);
Ytest = zeros(N_mesh_points*N_test_per_point, 1);

expected_frequencies = zeros(N_train_per_point * N_mesh_points, 1);


%%
% 3.2) Apply MRFT test for automatically discritized mesh

iterator = 1;

%iterate over range of parameters

for i=1:N_mesh_points
    gain = final_points_normalized(1,7);
    tau = tau_grid(i);
    T_prop = T_prop_grid(i);
    T_body = T_body_grid(i); 
        
    disp('simulation start')
    disp('iteration')
    disp(iterator)

    %start simulation
    %initial and reference value  
    init_val = 0.001;
    ref_val = 0;
    
    height_model = tf([gain], [T_prop*T_body, T_prop+T_body, 1, 0], 'IODelay', tau);
    [amplitude, w] = get_MRFT_amplitude(height_model, h_mrft, beta_mrft);
    %measurement noise 
    sigma_h = amplitude*0.05;
    
    %simulate each mesh point N times for the training set
    for sim_i=1:N_train_per_point
        
        expected_frequencies(i) = w;
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


save('training_set_norm', 'Xtrain', 'Ytrain')
save('testing_set_norm', 'Xtest', 'Ytest')
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
%take last cycle of mrft test for each timeseries of generated data

t_cycle_start = zeros(1, size(Xtrain, 4));
t_cycle_end = zeros(1, size(Xtrain, 4));
longest_time = 0;
for i=1:size(Xtrain, 4)
    u_temp = Xtrain(:, 1, 2, i);
    iterator = 0;
    first_edge_detected = 0;
    
     for j=1:length(u_temp)-100
        accept = true;        
        for z=0:ceil((0.03*2*pi*expected_frequencies(i).^-1/NN_time_step))
            if (u_temp(end-j+1) - u_temp(end-j-z) > -1.95 * h_mrft)
                accept = false;
                break
            end
        end
        if accept
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
% 5. Train using a CNN on specific time segment of simulation data

%Load saved model / comment out if the model is being altered
%load('trained_model_seg_norm.mat')

options = trainingOptions('adam', 'Plots', 'training-progress','Shuffle','every-epoch','MaxEpochs', 500, 'LearnRateSchedule','none', 'MiniBatchSize',128, 'InitialLearnRate', 0.001, 'LearnRateDropPeriod',50,'LearnRateDropFactor',0.05);
trained_network_seg = trainNetwork(Xtrain_cycle, categorical(Ytrain_cycle), lgraph_1_seg, options)

%save models
save('trained_model_seg_norm', 'lgraph_1_seg', 'trained_network_seg')


%%
% 6. run testing set on segment timeseries

Ytest_cycle_resuts = classify(trained_network_seg, Xtest_cycle)
Ytest_cycle_activations = activations(trained_network_seg, Xtest_cycle, 'softmax');

correct=0;
wrong=0;
for i=1:length(Ytest_cycle)
    if (double(Ytest_cycle_resuts(i)) == Ytest_cycle(i))
        correct = correct+1;
    else
        wrong = wrong+1;
    end
end
accuracy = 100 * correct / (correct+wrong)
