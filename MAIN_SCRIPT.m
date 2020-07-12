%% MAIN SCRIPT - Trajectory Tracking Using MPC
% 1. Initialization
%       -setting the driving scenario and initial conditions
% MAIN LOOP
% 2. Control reference
%       -using camera sensor obtaining all the necessary reference data
% 3. Longitudinal dynamics controller
%       -longitudinal dynamics are modelled using a simplified linear model
%       -an open loop controller is being used
% 4. Lateral dynamics controller
%       -for lower speed a kinematic lateral model and for higher
%       speed a dynamic lateral model are being used
%       -depending on the model, a corresponding MPC controller is being used
% 5. Implementation of the control signal
%       -update of vehicle parameters
% After this step the algorithm goes back to 2.


%%

%initialization script
start;

%setting the scenario
[scenario, egoVehicle, scenario_ref_x, scenario_ref_y] = create_scenario();

%setting the sensor
sensor = create_sensor(scenario);

%reference - used only for testing
scenario_waypoints=[scenario_ref_x,scenario_ref_y,zeros(size(scenario_ref_x))];

%starting the simulation
clf;
figure(1)
hAxes = axes;
chasePlot(egoVehicle,'Parent',hAxes);

%reset the scenario and the sensor
restart(scenario);
release(sensor);

%vehicle data
allPoses = actorPoses(scenario);

while (STOP==0)
    
    tic;
    % 2. Control reference
    [y_ref, dot_y_ref, yaw_ref, dot_yaw_ref,dot_YAW_des,YAW_des,X_des,Y_des,Curvature] = sensor_reference(egoVehicle, sensor, scenario);
    
    
    % logging the reference data
    yaw_ref_(i,1)=yaw_ref(1,1);
    dot_yaw_ref_(i,1)=dot_yaw_ref(1,1);
    y_ref_(i,1)=y_ref(1,1); 
    dot_y_ref_(i,1)=dot_y_ref(1,1);
    
    Curvature_(i,1) = Curvature(10,1);
    
    dot_YAW_des_(i,1)=dot_YAW_des(1,1);
    YAW_des_(i,1)=YAW_des(1,1);
    X_des_(i,1)=X_des(1,1);
    Y_des_(i,1)=Y_des(1,1);
  
           
    
    % 3. Longitudinal dynamics controller
    [a_x,v_x,v_ref_cal]=longitudinal_controller(yaw_ref,v_x_t,a_x_t,i,Curvature,T);
    
    % logging longitudinal controller data
    v_x_(i,1)=v_x;
    a_x_(i,1)=a_x;
    v_ref_cal_(i,1)=v_ref_cal;
    
    % 4. Lateral dynamics controller
    % if the speed is lower then 12 m/s, use the kinematic model and the corresponding controller
    % else use the dynamic model and the corresponding controller   
    
    if v_x<12
        
        
        % check that this model is being used
        is_kin_(i,1) = 1;
        
        
        % MPC controller for lateral kinematic model
        [delta_f,problem]=lateral_controller_kinematic(v_x, X_des, Y_des, yaw_ref,i, scenario);
        
        % logging lateral kinematic controller data
        delta_f_deg = delta_f*180/pi;
        delta_f_(i,1)=delta_f_deg;
        problem_(i,1)=problem;


        % get state vector
        [x_i] = get_state_kinematic(v_x, delta_f, xi_1_kin, i, scenario);
        X_kin_i_(i,1) = x_i(1,1);
        Y_kin_i_(i,1) = x_i(2,1);
        YAW_kin_i_(i,1) = x_i(3,1);
    
        
        
        % calculate vehicle parameters
        YAW=YAW+(x_i(3,1)*180/pi);
        
        V_y= v_x*sin(YAW*pi/180); 
        V_x= v_x*cos(YAW*pi/180);

        X=X+V_x*T;
        Y=Y+V_y*T;

        % update vehicle object
        egoVehicle.Position = [X Y 0];
        egoVehicle.Yaw = YAW;
        egoVehicle.Velocity = [V_x V_y 0];
        
        xi_1_kin=x_i;
        
    else
        
        % check that this model is being used
        is_dyn_(i,1) = 1;
        
        
        % MPC controller for lateral dynamic model
        [delta_f, problem]=lateral_controller_dynamic(v_x, y_ref, dot_y_ref, yaw_ref, dot_yaw_ref,i,scenario);
        
        % logging lateral dynamic controller data
        delta_f_deg = delta_f*180/pi;
        delta_f_(i,1)=delta_f_deg;
        problem_(i,1)=problem;


        % get state vector
        [x_i] = get_state_dynamic(v_x, delta_f, xi_1, i);
        y_i_(i,1) = x_i(1,1);
        dot_y_i_(i,1) = x_i(2,1);
        yaw_i_(i,1) = x_i(3,1);
        dot_yaw_i_(i,1) = x_i(4,1);
    
        
        % calculate vehicle parameters
        YAW=YAW-(x_i(3,1)*180/pi);
        V_y= v_x*sin(YAW*pi/180); 
        V_x= v_x*cos(YAW*pi/180);

        X=X+V_x*T;
        Y=Y+V_y*T;

        % update vehicle object
        egoVehicle.Position = [X Y 0];
        egoVehicle.Yaw = YAW;
        egoVehicle.Velocity = [V_x V_y 0];
        
        xi_1=x_i;
    
    end
        
    
    % update the simulation
    updatePlots(scenario);
    
    %logging the vehicle data
    allPoses = actorPoses(scenario);
    vehicle_yaw_(i,1)=allPoses.Yaw;
    
    vehicle_y_(i,1)=allPoses.Position(1,2);
    vehicle_x_(i,1)=allPoses.Position(1,1);
    
    vehicle_v_y_(i,1)=allPoses.Velocity(1,2);
    vehicle_v_x_(i,1)=allPoses.Velocity(1,1);
        
       
    % terminal condition using the terminal step N
    if i>=N
        STOP=1;
    end
    
    % terminal condition using the terminal step N and a check on if the
    % vehicle has passed the track
    
%     if i>=N && (vehicle_x_(i,1) >= (X_0-2)) && (vehicle_x_(i,1) < (X_0+2)) && (vehicle_y_(i,1) >= (Y_0-2)) && (vehicle_y_(i,1) < (Y_0+2))
%        STOP=1; 
%     end   
    


    % preparations for a new step
    i=i+1;
    
    a_x_t=a_x;
    v_x_t=v_x;
            
    %delay
    pause(0.01);
    
    %time
    T=toc;
    runtime=runtime+T;
    T_(i,1)= T;
    
end

release(sensor);

%calculate the derivative of the steering angle
delta_delta_f_=diff(delta_f_);
