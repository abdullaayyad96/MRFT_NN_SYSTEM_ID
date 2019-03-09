%%
%HOW TO RUN 

% 1. Select model Parameter ranges and 
%     deterioration limit to run discritization on
%
% 2. Initialize Parameters for the discritization process
%
% 3. Run the discritization and save data

%%                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
% 1. Select model Parameter ranges and 
%     deterioration limit to run discritization on

clear() 
%define mesh range for ratios
%ratio_1 is T_prop/tau 
%ratio_2 is T_body/tau
ratio_1_min = 0.15; ratio_1_max = 600;
ratio_2_min =   2; ratio_2_max = 4000;

%number of initial discritized points for each parameter
ratio_1_N_points = 250;
ratio_2_N_points = 250;

%generate mesh (constant step)
% ratio_1_mesh = linspace(ratio_1_min, ratio_1_max, ratio_1_N_points);%gain values for the initial mesh
% ratio_2_mesh = linspace(ratio_2_min, ratio_2_max, ratio_2_N_points);%gain values for the initial mesh


%generaye mesh (logarithmic step)
ratio_1_mesh = exp(linspace(log(ratio_1_min), log(ratio_1_max), ratio_1_N_points));%ratio 1 values for the initial mesh
ratio_2_mesh = exp(linspace(log(ratio_2_min), log(ratio_2_max), ratio_2_N_points));%ratio 2 values for the initial mesh

%a list to contain the final discritized points in the mesh, 
%discritization occurs once a point exceeding the deterioration limit is
%found
final_points_normalized = [];

%the deterioration limit for discritization, once a point in the mesh
%exceeds this limit a new point is added to the final discritization list
deterioration_limit = 10;

%the mesh will be classified to into classes, each class correspond to the deterioration limit of a discritized point
%these classes are stored in mesh_class
mesh_class = zeros([ratio_1_N_points, ratio_2_N_points]);

%an array to store intermediate deterioration value during discritization
deterioration_mesh = 200 * ones([ratio_1_N_points, ratio_2_N_points]);


%%
% 2. Initialize Parameters for the discritization process

global Kp Kd  t_final time_step gain T_prop T_body tau ref_val init_val bias_acc  sigma_h  cost_criteria
Kp = 0; 
Kd = 0;         
ref_val = 1;    
init_val = 0;
bias_acc = 0;
sigma_h = 0;    
t_final = 20;
time_step = 0.0001;
cost_criteria = 'ISE'; %optimization critirea, can be: 'IAE', 'ISE', 'ITAE', 'ITSE'
gain = 1;
tau = 0.01;

%intermediate variables to use during discriziation
finish = false;
class = size(final_points_normalized, 1);
initial = (class==0);
last_save = 0;

initial_ratio_1 = 1;%ratio_1_N_points;
initial_ratio_2 = 1;%ratio_2_N_points;


%%
% 3. Run the discritization and save data
%iterate through initial mesh and discritize according to deterioration
%difined initial parameters for the simulations
%the simulations are done using 'HeightModel_pid'

while(~finish)
    
    if (initial)
        %select first point in the mesh
        i_ratio_1 = initial_ratio_1;
        i_ratio_2 = initial_ratio_2;  
        initial = false;
        
    else
        %select a point in the mesh with deterioration exceeding 5%
        temp_mesh_class = zeros(size(mesh_class)); 
        temp_mesh_class(mesh_class == 0) = 1;        
        temp_deterioration_mesh = deterioration_mesh .* temp_mesh_class;
        [i_ratio_1, i_ratio_2] = ind2sub(size(temp_deterioration_mesh), find(temp_deterioration_mesh >= deterioration_limit));
        
        
        %find point closest to initial point
        distance = 1e10;
        for k=1:length(i_ratio_1)
            temp_i_ratio_1 = i_ratio_1(k);
            temp_i_ratio_2 = i_ratio_2(k);
            
            temp_distance = (temp_i_ratio_1-initial_ratio_1)^2 + (temp_i_ratio_2-initial_ratio_2)^2;
            
            if (temp_distance < distance)
                distance = temp_distance;
                min_i_ratio_1 = temp_i_ratio_1;
                min_i_ratio_2 = temp_i_ratio_2;
            end
        end
            
        i_ratio_1 = min_i_ratio_1;
        i_ratio_2 = min_i_ratio_2;
    end
    
    %obtain values from the mesh that correspond to desired index
    T_prop = ratio_1_mesh(i_ratio_1) * tau;
    T_body = ratio_2_mesh(i_ratio_2) * tau;
    
    %to save time and ensure flow accurate solution, change simulation time
    %range and time step size according to time constants
    t_final = max([3, 10*max([T_body, T_prop])]);
    time_step = min([tau, 0.1*min([T_body, T_prop])]);
        
    
    %find optimal PD params    
    warning ('off','all');
    
    ref_cost = 1e6;
    optimize_itr = 0;    
    %optimimize until a reasonable error cost is obtained or the max number
    %of iterations is exceeded4
    disp(sprintf('optimizing point [%d, %d] ...', i_ratio_1, i_ratio_2));

    while(ref_cost >  0.3 * t_final * ref_val^2 && optimize_itr < 3)  
        if(optimize_itr == 0)
            %optimize using the previous PD parameters as initial guess
            [K,ref_cost,~] = fminsearchbnd((@get_step_cost_identified),[Kp Kd], [0 0],[],optimset('Algorithm', 'SQP', 'MaxFunEvals',1000,'MaxIter',100,'TolFun', t_final * ref_val^2 * 1e-3,'TolX',2e-4,'Display','on'));
        else            
            %optimize using zeroes as initial guess
            [K,ref_cost,~] = fminsearchbnd((@get_step_cost_identified),[0 0], [0 0],[],optimset('Algorithm', 'SQP', 'MaxFunEvals',1000,'MaxIter',100,'TolFun', t_final * ref_val^2 * 1e-3,'TolX',2e-4,'Display','on'));
        end
        optimize_itr = optimize_itr + 1;
    end
    Kp = K(1);
    Kd = K(2);
    disp(sprintf('optimal PD parameters found, referance cost is %d', ref_cost));
    
    
    %log point
    final_points_normalized = [final_points_normalized ; [i_ratio_1, i_ratio_2, ratio_1_mesh(i_ratio_1), ratio_2_mesh(i_ratio_2), gain, tau]];
    %deterioration is zero at the optimized point
    deterioration_mesh(i_ratio_1, i_ratio_2) = 0;
    
    %for each optimized point, a range in Euclidean space is defined as its
    %deterioration limit. 
    %initializing the Euclidean ranges
    i_ratio_1_limit_low = i_ratio_1;
    i_ratio_2_limit_low = i_ratio_2;

    i_ratio_1_limit_high = i_ratio_1;
    i_ratio_2_limit_high = i_ratio_2;

    for directions= 1:4
        %iterate through 8 possible directions in four dimensional mesh             
        
        n_steps = 0;        
        deterioration_exceed = false;     
        while(~deterioration_exceed)
            %keep going in the same direction untul the deterioration limit
            %is exceeded
            
            n_steps = n_steps + 1; %increment the number of steps
            
            %determine current point of evaluation based on direction and
            %number of steps
            i2_ratio_1 = i_ratio_1 + n_steps * (directions==1);
            i2_ratio_1 = i2_ratio_1 - n_steps * (directions==2);

            i2_ratio_2 = i_ratio_2 + n_steps * (directions==3);
            i2_ratio_2 = i2_ratio_2 - n_steps * (directions==4); 
            
            if ( (i2_ratio_1>0) && (i2_ratio_2>0) && ...
                (i2_ratio_1<=ratio_1_N_points) && (i2_ratio_2<=ratio_2_N_points))
                    %check that the current point of evaluation is within
                    %the initial mesh
               
                    if (mesh_class(i2_ratio_1, i2_ratio_2) == 0)
                        %check that the current point is not assigned to a
                        %class
                        
                        %obtain parameter values
                        T_prop = ratio_1_mesh(i2_ratio_1) * tau;
                        T_body = ratio_2_mesh(i2_ratio_2) * tau;
                        
                        %obtain cost and deterioration at current point
                        cost = get_step_cost_identified([Kp, Kd]);
                        deterioration = abs((cost - ref_cost) / ref_cost) * 100;
                        
                        %update the deterioration value in the
                        %deterioration mesh
                        deterioration_mesh(i2_ratio_1, i2_ratio_2) = deterioration;
                        
                        %check whether deterioration limit has been
                        %exceeded
                        if (deterioration > deterioration_limit)
                            deterioration_exceed = true;
                        end

                    elseif(mesh_class(i2_ratio_1, i2_ratio_2) ~= 0)
                        %if the current point of evaluation has been
                        %assigned to another class, exit
                        deterioration_exceed = true;
                    end
            else
                %exit if the current point is outside the mesh
                deterioration_exceed = true;
            end
        end
        
        %update deterioration margins based on most recent direction and
        %number of steps
        i_ratio_1_limit_high = i_ratio_1_limit_high + (n_steps-1) * (directions==1);                
        i_ratio_1_limit_low = i_ratio_1_limit_low - (n_steps-1) * (directions==2);

        i_ratio_2_limit_high = i_ratio_2_limit_high + (n_steps-1) * (directions==3);
        i_ratio_2_limit_low = i_ratio_2_limit_low - (n_steps-1) * (directions==4);               
    end
    
    %update the mesh_class according to the deterioration ranges found
    %above
    class = class + 1;
    mesh_class(i_ratio_1_limit_low:i_ratio_1_limit_high, i_ratio_2_limit_low:i_ratio_2_limit_high) = class;
    
    %print these limits
    disp('low limit:')
    disp([i_ratio_1_limit_low  i_ratio_2_limit_low]);
    disp('high limit:')
    disp([i_ratio_1_limit_high  i_ratio_2_limit_high]);

    
    if(min(mesh_class(:)) ~=0)
        %finish when all points in the original mesh have been assigned to
        %a class
        finish = true;
    end     
    
    %display the percentage completed
    completed = 100 * size(find(mesh_class(:)~=0)) ./ size(mesh_class(:));
    disp(sprintf('completed: %d %', completed(1)));
    if ((completed(1) - last_save) > 10)
        last_save = completed(1);
        save('discrete_mesh_normalized_10_2', 'ratio_1_mesh', 'ratio_2_mesh', 'final_points_normalized', 'mesh_class', 'deterioration_mesh')
    end
end

save('discrete_mesh_normalized_10_2', 'ratio_1_mesh', 'ratio_2_mesh', 'final_points_normalized', 'mesh_class', 'deterioration_mesh')
    
 