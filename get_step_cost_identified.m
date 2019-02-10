function cost=get_step_cost_identified(x,varagin)
global Kp Kd  t_final time_step gain T_prop T_body tau ref_val init_val bias_acc  sigma_h cost_criteria

Kp=x(1);
Kd=x(2);

options = simset('SrcWorkspace','current','DstWorkspace','current','SignalLogging','on','SignalLoggingName','logged_data');

simOut = sim('HeightModel_pid',[],options);

Error = logged_data.get('Error');
%val_err=Error.Data;
val_err=Error.Values.Data;
%t=Error.Time;
t=Error.Values.Time;
u = logged_data.get('u');
%val_u = abs(u.Data);
val_u = abs(u.Values.Data); 
max_u = max(val_u);

if strcmp(cost_criteria,'IAE')
    Q=trapz(t,abs(val_err));
elseif strcmp(cost_criteria,'ISE')
    Q=trapz(t,val_err.*val_err);
elseif strcmp(cost_criteria,'ITSE')
    Q=trapz(t,t.*val_err.*val_err);
elseif strcmp(cost_criteria,'ITAE')
    Q=trapz(t,t.*abs(val_err));
end

cost=Q;