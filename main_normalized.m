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
load('discrete_mesh_normalized_2D_to_3D.mat')
final_points_normalized = final_points_normalized(end-9:end, :);

%define parameters grid based on smart discritization
tau_grid = transpose(final_points_normalized(:, 4));
T_prop_grid = transpose(final_points_normalized(:, 5));
T_body_grid = transpose(final_points_normalized(:, 6));

N_mesh_points = size(final_points_normalized, 1);


%%
% 2. Select Simulation Parameters

time_step = 0.001;
NN_time_step = 0.001;
t_final = 35; %final simulation time
N_timesteps = floor(t_final/time_step) + 1; %total number of steps

%mrft parameters
beta_mrft = -0.73;
h_mrft = 0.1;

N_train_per_point = 50; %number of times each mesh point is simulated for the training set
N_test_per_point = 5;%number of times each mesh point is simulated for the testing set


%initialize all training timeseries
Xtrain = zeros(N_timesteps, 1, 2, N_mesh_points*N_train_per_point);
Ytrain = zeros(N_mesh_points*N_train_per_point, 1);

%initialize all testing timeseries
Xtest = zeros(N_timesteps, 1, 2, N_mesh_points*N_test_per_point);
Ytest = zeros(N_mesh_points*N_test_per_point, 1);

expected_frequencies = zeros(N_mesh_points, 1);

%relay bias percentage
bias_mag = 0.5;

%noise percentage
noise_mag = 0.05;


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
    
    expected_frequencies(iterator) = w;
    %measurement noise 
    sigma_h = amplitude*noise_mag;
    
    %simulate each mesh point N times for the training set
    for sim_i=1:N_train_per_point
        
        %set bias
        bias_relay = bias_mag * (2*rand()-1) * h_mrft;

        %run simulation and log      
        %options = simset('SrcWorkspace','current','DstWorkspace','current','SignalLoggingName','logged_data');
        set_param('HeightModel_mrft','FastRestart','on');
        options = simset('SrcWorkspace','current');
        simOut = sim('HeightModel_mrft.slx',[],options);
        %Height_noise = logged_data.get('Height_noise');
        Height_noise = simOut.logsout.get('Height_noise');
        val_height_noise=Height_noise.Values.Data;
        %val_height_noise=Height_noise.Data;
        %u_tot = logged_data.get('u');
        u_tot = simOut.logsout.get('u');
        val_u= u_tot.Values.Data;
        %val_u_tot=u_tot.Data;
        
        %append data to training set
        sample_index = (iterator-1)*N_train_per_point + sim_i;
        Xtrain(:,1,1,sample_index) = val_height_noise; %log height
        Xtrain(:,1,2,sample_index) = val_u; %log controller output
        Ytrain(sample_index, 1) = iterator;
    end
    
    %simulate each mesh point N times for the testing set
    for sim_i=1:N_test_per_point
        bias_relay = bias_mag * (2*rand()-1) * h_mrft;               

        %options = simset('SrcWorkspace','current','DstWorkspace','current','SignalLoggingName','logged_data');
        options = simset('SrcWorkspace','current');
        simOut = sim('HeightModel_mrft.slx',[],options);
        %Height_noise = logged_data.get('Height_noise');
        Height_noise = simOut.logsout.get('Height_noise');
        val_height_noise=Height_noise.Values.Data;
        %val_height_noise=Height_noise.Data;
        %u_tot = logged_data.get('u');
        u_tot = simOut.logsout.get('u');
        val_u= u_tot.Values.Data;
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
save('training_set_norm', 'Xtrain', 'Ytrain', 'expected_frequencies')
save('testing_set_norm', 'Xtest', 'Ytest')

%%
%take last cycle of mrft test for each timeseries of generated data
%(training set)

t_cycle_start = zeros(1, size(Xtrain, 4));
t_cycle_end = zeros(1, size(Xtrain, 4));

longest_time = 0;
for i=1:size(Xtrain, 4)
    u_temp = Xtrain(:, 1, 2, i);
    iterator = 0;
    first_edge_detected = 0;
    
    for j=1:length(u_temp)-100
        accept = true;     
        
        i_expected_frequency = floor( (i-1) / N_train_per_point) + 1;
        
        for z=0:ceil(( (0.3*2*pi) / (expected_frequencies(i_expected_frequency)*NN_time_step)))
            if (u_temp(end-j+1) - u_temp(end-j-z) < 1.95 * h_mrft)
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


%%
%take last cycle of mrft test for each timeseries of generated data
%(Testing set)

t_cycle_start_test = zeros(1, size(Xtrain, 4));
t_cycle_end_test = zeros(1, size(Xtrain, 4));

for i=1:size(Xtest, 4)
    u_temp = Xtest(:, 1, 2, i);
    iterator = 0;
    first_edge_detected = 0;
    
    for j=1:length(u_temp)-100
        accept = true;     
        
        i_expected_frequency = floor( (i-1) / N_test_per_point) + 1;
        
        for z=0:ceil(( (0.3*2*pi) / (expected_frequencies(i_expected_frequency)*NN_time_step)))
            if (u_temp(end-j+1) - u_temp(end-j-z) < 1.95 * h_mrft)
                accept = false;
                break
            end
        end
        if accept
            iterator = iterator + 1;
        end
        
        if (iterator == 1 && first_edge_detected == 0) 
            t_cycle_end_test(i) = length(u_temp) - j;
            first_edge_detected = 1;
        elseif (iterator >= 2)
            t_cycle_start_test(i) = length(u_temp) - j;            
            break
        end
    end
    if longest_time < (t_cycle_end_test(i) - t_cycle_start_test(i))
        longest_time = t_cycle_end_test(i) - t_cycle_start_test(i);
    end
end

%%
% Generate data with only one cycle for each point in the mesh (training)
Xtrain_cycle =  zeros(longest_time, 1, 2, N_mesh_points*N_train_per_point);
Ytrain_cycle = Ytrain;

for i=1:size(Xtrain, 4)
    Xtrain_cycle(end-(t_cycle_end(i)-t_cycle_start(i))+1:end, :, :, i) = Xtrain(t_cycle_start(i)+1:t_cycle_end(i), :, :, i);
end

% Generate data with only one cycle for each point in the mesh (testing)
Xtest_cycle =  zeros(longest_time, 1, 2, N_mesh_points*N_test_per_point);
Ytest_cycle = Ytest;

for i=1:size(Xtest, 4)
    Xtest_cycle(end-(t_cycle_end_test(i)-t_cycle_start_test(i))+1:end, :, :, i) = Xtest(t_cycle_start_test(i)+1:t_cycle_end_test(i), :, :, i);
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
%plot samples from cycle testing data
figure()
dim = ceil(sqrt(N_mesh_points));

for i=1:(dim*dim)
    subplot(dim, dim, i)
    plot(Xtest_cycle(:,1,1,(i-1)*floor(N_test_per_point)+1))
    title(Ytest_cycle((i-1)*floor(N_test_per_point)+1,1))
end


%%
%Scaling amplitude to one (training)
c2 = zeros(size(Xtrain_cycle, 4), 1);

for i=1:size(Xtrain_cycle, 4)
    h_temp = Xtrain_cycle(:, 1, 1, i);
    
    amplitude = (max(h_temp(:)) - min(h_temp(:))) / 2;
    
    c2(i) = 1 / amplitude;
    
    Xtrain_cycle(:, 1, 1, i) = h_temp / amplitude;
    
end

%%
%Scaling amplitude to one (testing)
c2 = zeros(size(Xtest_cycle, 4), 1);

for i=1:size(Xtest_cycle, 4)
    h_temp = Xtest_cycle(:, 1, 1, i);
    
    amplitude = (max(h_temp(:)) - min(h_temp(:))) / 2;
    
    c2(i) = 1 / amplitude;
    
    Xtest_cycle(:, 1, 1, i) = h_temp / amplitude;
    
end

%%
%plot samples from normalized cycle training data
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
%plot samples from normalized cycle testing data
figure()
dim = ceil(sqrt(N_mesh_points));

for i=1:(dim*dim)
    subplot(dim, dim, i)
    plot(Xtest_cycle(:,1,1,(i-1)*floor(N_test_per_point)+1))
    hold on    
    plot(Xtest_cycle(:,1,2,(i-1)*floor(N_test_per_point)+1))
    
    title(Ytest_cycle((i-1)*floor(N_test_per_point)+1,1))
end
%%
%save segmented training and testing set
save('training_set_cycle_norm', 'Xtrain_cycle', 'Ytrain_cycle')
save('testing_set_cycle_norm', 'Xtest_cycle', 'Ytest_cycle')


%%
% 5. Train using a CNN on specific time segment of simulation data

%Load saved model / comment out if the model is being altered
%load('trained_model_seg_norm.mat')

options = trainingOptions('adam', 'Plots', 'training-progress','Shuffle','every-epoch','MaxEpochs', 500, 'LearnRateSchedule','piecewise', 'MiniBatchSize', 350, 'InitialLearnRate', 0.005, 'LearnRateDropPeriod',100,'LearnRateDropFactor',0.5);

lgraph_1_seg = [ 
    imageInputLayer([3008, 1, 2])
    convolution2dLayer([300, 1], 50, 'Stride', [100, 1])
    reluLayer()
    fullyConnectedLayer(3000)
    reluLayer()
    dropoutLayer(0.5)
    fullyConnectedLayer(1000)
    reluLayer()
    dropoutLayer(0.5)
    fullyConnectedLayer(10, 'name', 'ok')
    softmaxLayer()
    classificationLayer];

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
