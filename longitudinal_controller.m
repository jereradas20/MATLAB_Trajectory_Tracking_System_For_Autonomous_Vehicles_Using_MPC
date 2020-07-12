function [a_x,v_x,v_ref_cal]=longitudinal_controller(yaw_ref,v_x_t,a_x_t,i,Curvature,T)
%LONGITUDINAL_CONTROLLER 


a_max=3;
a_min=-5;
v_max= 18;
v_min= 3;
v_ref_cal=0;

%radi prvih par koraka koji traju predugo zbog početka korištenja yalmipa, da brzina ne bude prevelika
if T>1
    T=1;
end

%inicijalno postavi akcelraciju na maksimalnu vrijednost, a brzinu u 0
if i==1
    a_x=a_max;
    v_x=0;

%u drugom koraku izračunaj novu brzinu
else if i==2
    v_x=a_x_t*T;
    a_x=a_max;

%ostale korake izračunaj brzinu i akceleraciju
else 
    
    v_x=a_x_t*T + v_x_t

    %ako je prekid ceste nastavi istom brzinom voziti   
    if Curvature==zeros(100,1)
        v_ref=v_x_t;
    else
        
        %ovisno o zakrivljenju ceste odabir referemce
        
        %odabir indeksa podataka reference
        Curvature_index_1 =1+ 10*ceil(abs(v_x*T/30*10)); %zaokruži na veći; odabir indexa
        dot_yaw_index_1 =1+ ceil(abs(v_x*T/30*10));

        Curvature_index_2 =2+ 10*ceil(abs(v_x*T/30*10)); %zaokruži na veći; odabir indexa
        dot_yaw_index_2 =2+ ceil(abs(v_x*T/30*10));

        %izračun referentnih brzina
        v_ref_1 = abs(yaw_ref(dot_yaw_index_1,1)/Curvature(Curvature_index_1,1));
        
        v_ref_2 = abs(yaw_ref(dot_yaw_index_2,1)/Curvature(Curvature_index_2,1));
        
        %najbolje vladanje postignuto na ovaj način
        v_ref = (v_ref_1 + v_ref_2)/2
        
        %blago zasićenje izračunatih brzina
        if v_ref>16
                    
            v_ref = 0.8*v_ref
        end
        
        if v_ref<3 && v_ref>1
                    
            v_ref = 2*v_ref
        end

        
        v_ref_cal=v_ref;
        
        %ograničenja brzine
        if v_ref>v_max
            v_ref=v_max;
        end
        if v_ref<v_min
            v_ref=v_min;
        end
    end
    
    a_x=(v_ref-v_x)/T;

    %ograničenja akceleracije
    if a_x>a_max
        a_x=a_max;
    end
    if a_x<a_min
        a_x=a_min;
    end



end


end

