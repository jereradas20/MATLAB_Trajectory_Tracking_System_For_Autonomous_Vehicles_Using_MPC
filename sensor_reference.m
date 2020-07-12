function [y_ref, dot_y_ref, yaw_ref, dot_yaw_ref,dot_YAW_des,YAW_des,X_des,Y_des,Curvature] = sensor_reference(egoVehicle, sensor, scenario)

%%FUNKCIJA KOJA VRAÆA POTREBNE PODATKE SENZORA

%podaci o vozilu

allPoses = actorPoses(scenario);



%oèitanje traka ceste na horizontu senzora; jedna je krajnja traka, druga je srednja

lanes = laneBoundaries(egoVehicle, 'XDistance', linspace(0, sensor.MaxRange, 100));



%zakrivljenje ceste
Curvature = (lanes(2).Curvature(:,1)+lanes(1).Curvature(:,1))/2;


%reference na horizontu senzora
ego_reference_x = (lanes(2).Coordinates(:,1)+lanes(1).Coordinates(:,1))/2;

ego_reference_y = (lanes(2).Coordinates(:,2)+lanes(1).Coordinates(:,2))/2;


%REFERENCA U KOORDINATAMA VOZILA
x_ref=ego_reference_x(11:9:100,1);
x_ref2=ego_reference_x(13:8:100,1);
dot_x_ref=diff(ego_reference_x(13:8:100,1));


%izraèun vektora reference y_ref i dot_y_ref dimenzije (10;1)

y_ref=ego_reference_y(11:9:100,1);
y_ref2=ego_reference_y(13:8:100,1);
dot_y_ref=diff(ego_reference_y(13:8:100,1));


%izraèun yaw_ref i dot_yaw_ref dimenzije (10;1) iz podataka senzora i podataka vozila

ego_yaw_ref=atan2(ego_reference_y(11:9:100,1),ego_reference_x(11:9:100,1));

yaw_ref=ego_yaw_ref;

ego_yaw_ref2=atan2(-ego_reference_y(13:8:100,1),ego_reference_x(13:8:100,1));

dot_yaw_ref=diff(ego_yaw_ref2);

v_ref = sqrt((y_ref.^2+x_ref.^2));
v_ref2 = sqrt((y_ref2.^2+x_ref2.^2));


%REFERENCE U GLOBALNIM KOORDINATAMA
X_c=allPoses.Position(1);
Y_c=allPoses.Position(2);
YAW_c=pi/180*allPoses.Yaw;


X_des = X_c + v_ref.*cos(ego_yaw_ref+YAW_c);

Y_des = Y_c + v_ref.*sin(ego_yaw_ref+YAW_c);

YAW_des= 180/pi*atan2(Y_des-Y_c,X_des-X_c);


%potrebno element više zbog izraèuna dot_YAW_des


X_des2 = X_c + v_ref2.*cos(ego_yaw_ref2+YAW_c);

Y_des2 = Y_c + v_ref2.*sin(ego_yaw_ref2+YAW_c);

YAW_ref2 =  180/pi*atan2(Y_des2-Y_c,X_des2-X_c);

dot_YAW_des=diff(-YAW_ref2);







%kompenzacija prekida ceste
%ako senzor krene èitati Nan vrijednosti, postavi sve podatke u 0
if sum((isnan(y_ref)))>0 || sum((isnan(dot_y_ref)))>0 || sum((isnan(yaw_ref)))>0 || sum((isnan(dot_yaw_ref)))>0
    y_ref=zeros(10,1);
    yaw_ref=zeros(10,1);
    dot_y_ref=zeros(10,1);
    dot_yaw_ref=zeros(10,1);
    dot_YAW_des=zeros(10,1);
    YAW_des=zeros(10,1);
    X_des=zeros(10,1);
    Y_des=zeros(10,1);
end


if sum((isnan(Curvature)))>0
    Curvature=zeros(100,1);
end
    
end
