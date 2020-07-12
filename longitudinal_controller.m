function [a_x,v_x,v_ref_cal]=longitudinal_controller(yaw_ref,v_x_t,a_x_t,i,Curvature,T)
%% LONGITUDINAL DYNAMICS CONTROLLER


a_max=3;
a_min=-5;
v_max= 18;
v_min= 3;
v_ref_cal=0;

% first few steps take too long because initial YALMIP usage
% time is limited so that the calculated speed isn't too high
if T>1
    T=1;
end

% at the first step use max acceleration
if i==1
    a_x=a_max;
    v_x=0;

% at the second step calculate speed
else if i==2
    v_x=a_x_t*T;
    a_x=a_max;

% at other steps use the longitudinal model to calculate speed
else 
    
    % simplified linear longitudinal model
    v_x=a_x_t*T + v_x_t

    % test for track interrupt  
    if Curvature==zeros(100,1)
        v_ref=v_x_t;
    else
        
        % speed reference calculated as the ratio between derivative of the yaw
        % reference and the curvature reference
        
        % best performances accomplished with these "embelishments"
        Curvature_index_1 =1+ 10*ceil(abs(v_x*T/30*10)); 
        dot_yaw_index_1 =1+ ceil(abs(v_x*T/30*10));

        Curvature_index_2 =2+ 10*ceil(abs(v_x*T/30*10)); 
        dot_yaw_index_2 =2+ ceil(abs(v_x*T/30*10));

        
        v_ref_1 = abs(yaw_ref(dot_yaw_index_1,1)/Curvature(Curvature_index_1,1));
        
        v_ref_2 = abs(yaw_ref(dot_yaw_index_2,1)/Curvature(Curvature_index_2,1));
        
        % speed reference
        v_ref = (v_ref_1 + v_ref_2)/2
        
        % soft saturations
        if v_ref>16
                    
            v_ref = 0.8*v_ref
        end
        
        if v_ref<3 && v_ref>1
                    
            v_ref = 2*v_ref
        end

        
        v_ref_cal=v_ref;
        
        % speed constraints
        if v_ref>v_max
            v_ref=v_max;
        end
        if v_ref<v_min
            v_ref=v_min;
        end
    end
    
    a_x=(v_ref-v_x)/T;

    % acceleration constraints
    if a_x>a_max
        a_x=a_max;
    end
    if a_x<a_min
        a_x=a_min;
    end



end


end

