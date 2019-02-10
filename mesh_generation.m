%%
%TODO: add HOW TO RUN 


%%
clear() 

%define mesh range for each parameter
gain_min = 25; gain_max = 400;
T_prop_min = 0.01; T_prop_max = 0.35;
T_body_min = 0.02; T_body_max = 150;
tau_min = 0.0005; tau_max = 0.1;

%number of initial discritized points for each parameter
gain_N_points = 50;
T_prop_N_points = 100;
T_body_N_points = 150;
tau__N_points = 150;

%generate mesh (constant step)
% gain_mesh = linspace(gain_min, gain_max, gain_N_points);%gain values for the initial mesh
% T_prop_mesh = linspace(T_prop_min, T_prop_max, T_prop_N_points);%gain values for the initial mesh
% T_body_mesh = linspace(T_body_min, T_body_max, T_body_N_points);%gain values for the initial mesh
% tau_mesh = linspace(tau_min, tau_max, tau__N_points);%gain values for the initial mesh


%generaye mesh (logarithmic step)
<<<<<<< HEAD
gain_mesh = exp(linspace(log(gain_min), log(gain_max), gain_N_points));%gain values for the initial mesh
=======
gain_meshgain_meshgain_mesh = exp(linspace(log(gain_min), log(gain_max), gain_N_points));%gain values for the initial mesh
>>>>>>> ae0590e8fd354fd5c7ef1710e994e4d5186c95b4
T_prop_mesh = exp(linspace(log(T_prop_min), log(T_prop_max), T_prop_N_points));%gain values for the initial mesh
T_body_mesh = exp(linspace(log(T_body_min), log(T_body_max), T_body_N_points));%gain values for the initial mesh
tau_mesh = exp(linspace(log(tau_min), log(tau_max), tau__N_points));%gain values for the initial mesh

%a list to contain the final discritized points in the mesh, 
%discritization occurs once a point exceeding the deterioration limit is
%found
final_points = [];

%the deterioration limit for discritization, once a point in the mesh
%exceeds this limit a new point is added to the final discritization list
deterioration_limit = 5;

%the mesh will be classified to into classes, each class correspond to the deterioration limit of a discritized point
%these classes are stored in mesh_class
mesh_class = zeros([gain_N_points, T_prop_N_points, T_body_N_points, tau__N_points]);

%an array to store intermediate deterioration value during discritization
deterioration_mesh = 100 * ones([gain_N_points, T_prop_N_points, T_body_N_points, tau__N_points]);



%%
%iterate through initial mesh and discritize according to deterioration

%difine initial parameters for the simulations
%the simulations are done using 'HeightModel_pid'
global Kp Kd  t_final time_step gain T_prop T_body tau ref_val init_val bias_acc  sigma_h  cost_criteria
Kp = 0; 
Kd = 0;         
ref_val = 1;    
init_val = 0;
bias_acc = 0;
sigma_h = 0;    
t_final = 20;
time_step = 0.0001;
cost_criteria = 'ITAE'; %optimization critirea, can be: 'IAE', 'ISE', 'ITAE', 'ITSE'

%intermediate variables to use during discriziation
finish = false;
class = 0;
initial = true;

%start iteration/discritization process
while(~finish)
    
    if (initial)
        %select first point in the mesh
        i_gain = 1;
        i_T_prop = 1;
        i_T_body = 1;
        i_tau = 1;        
        initial = false;
        
    else
        %select a point in the mesh with deterioration exceeding 5%
        temp_deterioration_mesh = deterioration_mesh;
        temp_deterioration_mesh(temp_deterioration_mesh < deterioration_limit) = 100;
        min_deterioration = min(temp_deterioration_mesh(:));
        [i_gain, i_T_prop, i_T_body, i_tau] = ind2sub(size(temp_deterioration_mesh), find(temp_deterioration_mesh == min_deterioration));
        i_gain = i_gain(1);
        i_T_prop = i_T_prop(1);
        i_T_body = i_T_body(1);
        i_tau = i_tau(1);
    end
    
    %obtain values from the mesh that correspond to desired index
    gain = gain_mesh(i_gain);
    T_prop = T_prop_mesh(i_T_prop);
    T_body = T_body_mesh(i_T_body);
    tau = tau_mesh(i_tau);
    
    %to save time and ensure flow accurate solution, change simulation time
    %range and time step size according to time constants
    t_final = min([5, 10*max([T_body, T_prop])]);
    time_step = min([0.5*tau, 0.1*min([T_body, T_prop])]);
        
    
    %find optimal PD params    
    warning ('off','all');
    
    ref_cost = 1e6;
    optimize_itr = 0;    
    %optimimize until a reasonable error cost is obtained or the max number
    %of iterations is exceeded
<<<<<<< HEAD
    disp(sprintf('optimizing point [%d, %d, %d, %d] ...', i_gain, i_T_prop, i_T_body, i_tau));
=======
    fprintf('optimizing point %a %b %c %d ...\n', i_gain, i_T_prop, i_T_body, i_tau);
>>>>>>> ae0590e8fd354fd5c7ef1710e994e4d5186c95b4
    while(ref_cost > 50 && optimize_itr < 3)  
        if(optimize_itr == 0)
            %optimize using the previous PD parameters as initial guess
            [K,ref_cost,~] = fminsearchbnd((@get_step_cost_identified),[Kp Kd], [0 0],[],optimset('MaxFunEvals',1000,'MaxIter',1000,'TolFun',1e-4,'TolX',1e-4,'Display','off'));
        else            
            %optimize using zeroes as initial guess
            [K,ref_cost,~] = fminsearchbnd((@get_step_cost_identified),[0 0], [0 0],[],optimset('MaxFunEvals',1000,'MaxIter',1000,'TolFun',1e-4,'TolX',1e-4,'Display','off'));
        end
        optimize_itr = optimize_itr + 1;
    end
    Kp = K(1);
    Kd = K(2);
<<<<<<< HEAD
    disp(sprintf('optimal PD parameters found, referance cost is %d', ref_cost));
=======
    fprintf('optimal PD parameter found, referance cost is %a', ref_cost);
>>>>>>> ae0590e8fd354fd5c7ef1710e994e4d5186c95b4
    
    
    %log point
    final_points = [final_points ; [gain, T_prop, T_body, tau]];
    %deterioration is zero at the optimized point
    deterioration_mesh(i_gain, i_T_prop, i_T_body, i_tau) = 0;
    
    %for each optimized point, a range in Euclidean space is defined as its
    %deterioration limit. 
    %initializing the Euclidean ranges
    i_gain_limit_low = i_gain;
    i_T_prop_limit_low = i_T_prop;
    i_T_body_limit_low = i_T_body;
    i_tau_limit_low = i_tau;

    i_gain_limit_high = i_gain;
    i_T_prop_limit_high = i_T_prop;
    i_T_body_limit_high = i_T_body;
    i_tau_limit_high = i_tau;


    for directions= 1:8
        %iterate through 8 possible directions in four dimensional mesh
              
        
        n_steps = 0;        
        deterioration_exceed = false;     
        while(~deterioration_exceed)
            %keep going in the same direction untul the deterioration limit
            %is exceeded
            
            n_steps = n_steps + 1; %increment the number of steps
            
            %determine current point of evaluation based on direction and
            %number of steps
            i2_gain = i_gain + n_steps * (directions==1);
            i2_gain = i2_gain - n_steps * (directions==2);

            i2_T_prop = i_T_prop + n_steps * (directions==3);
            i2_T_prop = i2_T_prop - n_steps * (directions==4);

            i2_T_body = i_T_body + n_steps * (directions==5);
            i2_T_body = i2_T_body - n_steps * (directions==6);

            i2_tau = i_tau + n_steps * (directions==7);    
            i2_tau = i2_tau - n_steps * (directions==8);   
            
            
            if ( (i2_gain>0) && (i2_T_prop>0) && (i2_T_body>0) && (i2_tau>0) && ...
                (i2_gain<=gain_N_points) && (i2_T_prop<=T_prop_N_points) && (i2_T_body<=T_body_N_points) && (i2_tau<=tau__N_points) )
                    %check that the current point of evaluation is within
                    %the initial mesh
               
                    if (mesh_class(i2_gain, i2_T_prop, i2_T_body, i2_tau) == 0)
                        %check that the current point is not assigned to a
                        %class
                        
                        %obtain parameter values
                        gain = gain_mesh(i2_gain);
                        T_prop = T_prop_mesh(i2_T_prop);
                        T_body = T_body_mesh(i2_T_body);
                        tau = tau_mesh(i2_tau);
                        
                        %obtain cost and deterioration at current point
                        cost = get_step_cost_identified([Kp, Kd]);
                        deterioration = abs((cost - ref_cost) / ref_cost) * 100;
                        
                        %update the deterioration value in the
                        %deterioration mesh
                        deterioration_mesh(i2_gain, i2_T_prop, i2_T_body, i2_tau) = deterioration;
                        
                        %check whether deterioration limit has been
                        %exceeded
                        if (deterioration > deterioration_limit)
                            deterioration_exceed = true;
                        end

                    elseif(mesh_class(i2_gain, i2_T_prop, i2_T_body, i2_tau) ~= 0)
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
        i_gain_limit_high = i_gain_limit_high + (n_steps-1) * (directions==1);                
        i_gain_limit_low = i_gain_limit_low - (n_steps-1) * (directions==2);

        i_T_prop_limit_high = i_T_prop_limit_high + (n_steps-1) * (directions==3);
        i_T_prop_limit_low = i_T_prop_limit_low - (n_steps-1) * (directions==4);

        i_T_body_limit_high = i_T_body_limit_high + (n_steps-1) * (directions==5);
        i_T_body_limit_low = i_T_body_limit_low - (n_steps-1) * (directions==6);

        i_tau_limit_high = i_tau_limit_high + (n_steps-1) * (directions==7);
        i_tau_limit_low = i_tau_limit_low - (n_steps-1) * (directions==8);                
    end
    
    %update the mesh_class according to the deterioration ranges found
    %above
    class = class + 1;
    mesh_class(i_gain_limit_low:i_gain_limit_high, i_T_prop_limit_low:i_T_prop_limit_high, ...
        i_T_body_limit_low:i_T_body_limit_high, i_tau_limit_low:i_tau_limit_high) = class;
    
    %print these limits
    disp('low limit:')
<<<<<<< HEAD
    disp([i_gain_limit_low  i_T_prop_limit_low i_T_body_limit_low i_tau_limit_low]);
    disp('high limit:')
    disp([i_gain_limit_high  i_T_prop_limit_high i_T_body_limit_high i_tau_limit_high]);
=======
    [i_gain_limit_low  i_T_prop_limit_low i_T_body_limit_low i_tau_limit_low]    
    disp('high limit:')
    [i_gain_limit_high  i_T_prop_limit_high i_T_body_limit_high i_tau_limit_high]
>>>>>>> ae0590e8fd354fd5c7ef1710e994e4d5186c95b4
    
    
    if(min(mesh_class(:)) ~=0)
        %finish when all points in the original mesh have been assigned to
        %a class
        finish = true;
    end     
    
    %display the percentage completed
    completed = 100 * size(find(mesh_class(:)~=0)) ./ size(mesh_class(:));
<<<<<<< HEAD
    disp(sprintf('completed: %d', completed(1)));
=======
    fprintf('completed: %a', completed(1));
>>>>>>> ae0590e8fd354fd5c7ef1710e994e4d5186c95b4
    
end

save('discrete_mesh', 'gain_mesh', 'T_prop_mesh', 'T_body_mesh', 'tau_mesh', 'final_points', 'mesh_class')
    
    