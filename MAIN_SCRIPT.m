%GLAVNA SKRIPTA
%koriste se kinematièki i dinamièki modeli za lateralnu dinamiku
%koristi se linearni model za longitudinalnu dinamiku
%skripta sama proraèunava parametre vozila

%1. postavljanje scenarija i poèetnih uvjeta
%2. petlja koja obuhvaæa
%     -izraèun reference iz podataka senzora
%     -izraèun brzine 
%     -mpc za kut zakreta volana
%     -primjena upravljackih signala na sustav
%3. testiranje odziva


%%

%skripta inicijalizacije
start;

%postavljanje scenarija
[scenario, egoVehicle, scenario_ref_x, scenario_ref_y] = create_scenario();

%izrada senzora
sensor = create_sensor(scenario);

%referenca koja služi samo za testiranje
scenario_waypoints=[scenario_ref_x,scenario_ref_y,zeros(size(scenario_ref_x))];

%simulacija
clf;
figure(1)
hAxes = axes;
chasePlot(egoVehicle,'Parent',hAxes);

%resetiranje prije poèetka glavne petlje
restart(scenario);
release(sensor);

%podataci vozila
allPoses = actorPoses(scenario);

while (STOP==0)
    
    tic;
    %izraèun reference iz podataka senzora   
    [y_ref, dot_y_ref, yaw_ref, dot_yaw_ref,dot_YAW_des,YAW_des,X_des,Y_des,Curvature] = sensor_reference(egoVehicle, sensor, scenario);
    
    
    %logiranje podataka reference
    yaw_ref_(i,1)=yaw_ref(1,1);
    dot_yaw_ref_(i,1)=dot_yaw_ref(1,1);
    y_ref_(i,1)=y_ref(1,1); 
    dot_y_ref_(i,1)=dot_y_ref(1,1);
    
    Curvature_(i,1) = Curvature(10,1);
    
    dot_YAW_des_(i,1)=dot_YAW_des(1,1);
    YAW_des_(i,1)=YAW_des(1,1);
    X_des_(i,1)=X_des(1,1);
    Y_des_(i,1)=Y_des(1,1);
  
           
    
    %izraèun brzine
    [a_x,v_x,v_ref_cal]=longitudinal_controller(yaw_ref,v_x_t,a_x_t,i,Curvature,T);
    
    %logiranje podataka longitudinalnog kontrolera
    v_x_(i,1)=v_x;
    a_x_(i,1)=a_x;
    v_ref_cal_(i,1)=v_ref_cal;
    
    
    
    %ako je brzina manja od postavljene koristi kinematièki model, inaèe dinamièki    
    
    if v_x<12
        
        %KINEMATIÈKI MODEL LATERALNE DINAMIKE
        %bilježenje koji se model lateralne dinamike koristi 
        is_kin_(i,1) = 1;
        
        
        %mpc kontroler za kut zakreta volana
        [delta_f,problem]=lateral_controller_kinematic(v_x, X_des, Y_des, yaw_ref,i, scenario);
        
        %logiranje podataka lateralnog kontrolera
        delta_f_deg = delta_f*180/pi;
        delta_f_(i,1)=delta_f_deg;
        problem_(i,1)=problem;


        %novi vektor stanja
        [x_i] = get_state_kinematic(v_x, delta_f, xi_1_kin, i, scenario);
        X_kin_i_(i,1) = x_i(1,1);
        Y_kin_i_(i,1) = x_i(2,1);
        YAW_kin_i_(i,1) = x_i(3,1);
    
        
        
        %novi paramateri vozila

        YAW=YAW+(x_i(3,1)*180/pi);
        
        V_y= v_x*sin(YAW*pi/180); 
        V_x= v_x*cos(YAW*pi/180);

        X=X+V_x*T;
        Y=Y+V_y*T;

        %ažuriranje objekta vozila
        egoVehicle.Position = [X Y 0];
        egoVehicle.Yaw = YAW;
        egoVehicle.Velocity = [V_x V_y 0];
        
        xi_1_kin=x_i;
        
    else
        
        %DINAMIÈKI MODEL LATERALNE DINAMIKE
        %bilježenje koji se model lateralne dinamike koristi 
        is_dyn_(i,1) = 1;
        
        
        %mpc kontroler za kut zakreta volana
        [delta_f, problem]=lateral_controller_dynamic(v_x, y_ref, dot_y_ref, yaw_ref, dot_yaw_ref,i,scenario);
        
        %logiranje podataka lateralnog kontrolera
        delta_f_deg = delta_f*180/pi;
        delta_f_(i,1)=delta_f_deg;
        problem_(i,1)=problem;


        %novi vektor stanja
        [x_i] = get_state_dynamic(v_x, delta_f, xi_1, i);
        y_i_(i,1) = x_i(1,1);
        dot_y_i_(i,1) = x_i(2,1);
        yaw_i_(i,1) = x_i(3,1);
        dot_yaw_i_(i,1) = x_i(4,1);
    
        %novi paramateri vozila

        YAW=YAW-(x_i(3,1)*180/pi);
        V_y= v_x*sin(YAW*pi/180); %za beta = 0
        V_x= v_x*cos(YAW*pi/180);

        X=X+V_x*T;
        Y=Y+V_y*T;

        %ažuriranje objekta vozila
        egoVehicle.Position = [X Y 0];
        egoVehicle.Yaw = YAW;
        egoVehicle.Velocity = [V_x V_y 0];
        
        xi_1=x_i;
    
    end
        
    
    %osvježavanje odziva
    updatePlots(scenario);
    
    %logiranje podataka vozila
    allPoses = actorPoses(scenario);
    vehicle_yaw_(i,1)=allPoses.Yaw;
    
    vehicle_y_(i,1)=allPoses.Position(1,2);
    vehicle_x_(i,1)=allPoses.Position(1,1);
    
    vehicle_v_y_(i,1)=allPoses.Velocity(1,2);
    vehicle_v_x_(i,1)=allPoses.Velocity(1,1);
        
       
%   uvjet prekida za ogranièen broj koraka
    if i>=N
        STOP=1;
    end
    
    %   uvjet prekida za prolazak kroz cijelu stazu
%     if i>=N && (vehicle_x_(i,1) >= (X_0-2)) && (vehicle_x_(i,1) < (X_0+2)) && (vehicle_y_(i,1) >= (Y_0-2)) && (vehicle_y_(i,1) < (Y_0+2))
%        STOP=1; 
%     end   
    


    %pripreme za novu iteraciju
    i=i+1;
    
    a_x_t=a_x;
    v_x_t=v_x;
            
    %delay
    pause(0.01);
    
    %vrijeme
    T=toc;
    runtime=runtime+T;
    T_(i,1)= T;
    
end

release(sensor);

%nakon prolaska petlje logiranje promjene kuta zakreta volana
delta_delta_f_=diff(delta_f_);


%% odziv putanje vozila u odnosu na linije ceste i na referencu


RB=cell2mat(roadBoundaries(scenario));
center_line_x=(RB(:,1)+wrev(RB(:,4)))/2;
center_line_y=(RB(:,2)+wrev(RB(:,5)))/2;

figure
plot(vehicle_y_,vehicle_x_)
hold on
plot(scenario_waypoints(1:end,2),scenario_waypoints(1:end,1))
hold on
plot(center_line_y,center_line_x)
hold on
plot(RB(:,2),RB(:,1))
hold on
plot(RB(:,5),RB(:,4))
legend('vehicle','reference');

%% scatter
figure
scatter(vehicle_y_,vehicle_x_,50,v_x_,'filled','MarkerFaceAlpha',.75)
colorbar
legend('Trajektorija vozila')
test_x0=10;
test_y0=10;
width=750;
height=600;
set(gcf,'color','w','position',[test_x0,test_y0,width,height]);
hXLabel = xlabel('{\it Y}[m]');
hYLabel = ylabel('{\it X}[m]');
c = colorbar;
c.Label.String = '{\it v_x}[m/s]';
%(1:end-1,1)
grid on



