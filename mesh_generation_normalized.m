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
tau_min = 0.0005; tau_max = 0.1;
T_prop_min = 0.015; T_prop_max = 0.1;
T_body_min =   0.2; T_body_max = 2;

%number of initial discritized points for each parameter
tau_N_points     = 100;
T_prop_N_points  = 150;
T_body_N_points = 150;

%generate mesh (constant step)
% tau_mesh = linspace(tau_min, tau_max, tau_N_points);%tau values for the initial mesh
% T_prop_mesh = linspace(T_prop_min, T_prop_max, T_prop_N_points);%T_prop values for the initial mesh
% T_body_mesh = linspace(T_body_min, T_body_max, T_body_N_points);%T_body values for the initial mesh


%generate mesh (logarithmic step)
tau_mesh = exp(linspace(log(tau_min), log(tau_max), tau_N_points));%ratio 1 values for the initial mesh
T_prop_mesh = exp(linspace(log(T_prop_min), log(T_prop_max), T_prop_N_points));%ratio 1 values for the initial mesh
T_body_mesh = exp(linspace(log(T_body_min), log(T_body_max), T_body_N_points));%ratio 2 values for the initial mesh

%a list to contain the final discritized points in the mesh, 
%discritization occurs once a point exceeding the deterioration limit is
%found
final_points_normalized = [];

%the deterioration limit for discritization, once a point in the mesh
%exceeds this limit a new point is added to the final discritization list
deterioration_limit = 10;

%the mesh will be classified to into classes, each class correspond to the deterioration limit of a discritized point
%these classes are stored in mesh_class
mesh_class = zeros([tau_N_points, T_prop_N_points, T_body_N_points]);

%an array to store intermediate deterioration value during discritization
deterioration_mesh = 200 * ones([tau_N_points, T_prop_N_points, T_body_N_points]);


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

%intermediate variables to use during discriziation
finish = false;
class = size(final_points_normalized, 1);
initial = (class==0);
last_save = 0;

initial_tau = 1;%tau_N_points
initial_T_prop = 1;%T_prop_N_points;
initial_T_body = 1;%T_body_N_points;


%%
% 3. Run the discritization and save data
%iterate through initial mesh and discritize according to deterioration
%difined initial parameters for the simulations
%the simulations are done using 'HeightModel_pid'

while(~finish)
    
    if (initial)
        %select first point in the mesh
        i_tau = initial_tau;
        i_T_prop = initial_T_prop;
        i_T_body = initial_T_body;  
        initial = false;
        
    else
        %select a point in the mesh with deterioration exceeding 5%
        temp_mesh_class = zeros(size(mesh_class)); 
        temp_mesh_class(mesh_class == 0) = 1;        
        temp_deterioration_mesh = deterioration_mesh .* temp_mesh_class;
        [i_tau, i_T_prop, i_T_body] = ind2sub(size(temp_deterioration_mesh), find(temp_deterioration_mesh >= deterioration_limit));
        
        
        %find point closest to initial point
        distance = 1e10;
        for k=1:length(i_T_prop)
            temp_i_tau = i_tau(k);
            temp_i_T_prop = i_T_prop(k);
            temp_i_T_body = i_T_body(k);
            
            temp_distance = (temp_i_tau-initial_tau)^2 + (temp_i_T_prop-initial_T_prop)^2 + (temp_i_T_body-initial_T_body)^2;
            
            if (temp_distance < distance)
                distance = temp_distance;
                min_i_tau = temp_i_tau;
                min_i_T_prop = temp_i_T_prop;
                min_i_T_body = temp_i_T_body;
            end
        end
        
        i_tau = min_i_tau;
        i_T_prop = min_i_T_prop;
        i_T_body = min_i_T_body;
    end
    
    %obtain values from the mesh that correspond to desired index
    tau = tau_mesh(i_tau);
    T_prop = T_prop_mesh(i_T_prop);
    T_body = T_body_mesh(i_T_body);
    
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
    disp(sprintf('optimizing point [%d, %d, %d] ...', i_tau, i_T_prop, i_T_body));

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
    
       
    %for each optimized point, a range in Euclidean space is defined as its
    %deterioration limit. 
    %initializing the Euclidean ranges
    i_tau_limit_low = i_tau;
    i_T_prop_limit_low = i_T_prop;
    i_T_body_limit_low = i_T_body;

    i_tau_limit_high = i_tau;
    i_T_prop_limit_high = i_T_prop;
    i_T_body_limit_high = i_T_body;

    for directions= 1:6
        %iterate through 8 possible directions in four dimensional mesh             
        
        n_steps = 0;        
        deterioration_exceed = false;     
        while(~deterioration_exceed)
            %keep going in the same direction untul the deterioration limit
            %is exceeded
            
            n_steps = n_steps + 1; %increment the number of steps
            
            %determine current point of evaluation based on direction and
            %number of steps
            i2_tau = i_tau + n_steps * (directions==1);
            i2_tau = i2_tau - n_steps * (directions==2);
            
            i2_T_prop = i_T_prop + n_steps * (directions==3);
            i2_T_prop = i2_T_prop - n_steps * (directions==4);

            i2_T_body = i_T_body + n_steps * (directions==5);
            i2_T_body = i2_T_body - n_steps * (directions==6); 
            
            if ( (i2_tau>0) && (i2_T_prop>0) && (i2_T_body>0) && ...
                (i2_tau<=tau_N_points) && (i2_T_prop<=T_prop_N_points) && (i2_T_body<=T_body_N_points))
                    %check that the current point of evaluation is within
                    %the initial mesh
               
                    if (mesh_class(i2_tau, i2_T_prop, i2_T_body) == 0)
                        %check that the current point is not assigned to a
                        %class
                        
                        %obtain parameter values
                        tau = tau_mesh(i2_tau);
                        T_prop = T_prop_mesh(i2_T_prop);
                        T_body = T_body_mesh(i2_T_body);
                        
                        %obtain cost and deterioration at current point
                        cost = get_step_cost_identified([Kp, Kd]);
                        deterioration = abs((cost - ref_cost) / ref_cost) * 100;
                        
                        %update the deterioration value in the
                        %deterioration mesh
                        deterioration_mesh(i2_tau, i2_T_prop, i2_T_body) = deterioration;
                        
                        %check whether deterioration limit has been
                        %exceeded
                        if (deterioration > deterioration_limit)
                            deterioration_exceed = true;
                        end

                    elseif(mesh_class(i2_tau, i2_T_prop, i2_T_body) ~= 0)
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
        i_tau_limit_high = i_tau_limit_high + (n_steps-1) * (directions==1);                
        i_tau_limit_low = i_tau_limit_low - (n_steps-1) * (directions==2);
        
        i_T_prop_limit_high = i_T_prop_limit_high + (n_steps-1) * (directions==3);                
        i_T_prop_limit_low = i_T_prop_limit_low - (n_steps-1) * (directions==4);

        i_T_body_limit_high = i_T_body_limit_high + (n_steps-1) * (directions==5);
        i_T_body_limit_low = i_T_body_limit_low - (n_steps-1) * (directions==6);               
    end
        
    %log point
    final_points_normalized = [final_points_normalized ; [i_tau, i_T_prop, i_T_body, tau_mesh(i_tau), T_prop_mesh(i_T_prop), T_body_mesh(i_T_body), gain]];
    %deterioration is zero at the optimized point
    deterioration_mesh(i_tau, i_T_prop, i_T_body) = 0;
    %update the mesh_class according to the deterioration ranges found
    %above
    class = class + 1;
    mesh_class(i_tau_limit_low:i_tau_limit_high, i_T_prop_limit_low:i_T_prop_limit_high, i_T_body_limit_low:i_T_body_limit_high) = class;
    
    %print these limits
    disp('low limit:')
    disp([i_tau_limit_low i_T_prop_limit_low  i_T_body_limit_low]);
    disp('high limit:')
    disp([i_tau_limit_high i_T_prop_limit_high  i_T_body_limit_high]);

    
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
        save('discrete_mesh_normalized_10', 'tau_mesh', 'T_prop_mesh', 'T_body_mesh', 'final_points_normalized', 'mesh_class', 'deterioration_mesh')
    end
end

save('discrete_mesh_normalized_10', 'tau_mesh', 'T_prop_mesh', 'T_body_mesh', 'final_points_normalized', 'mesh_class', 'deterioration_mesh')
    
 