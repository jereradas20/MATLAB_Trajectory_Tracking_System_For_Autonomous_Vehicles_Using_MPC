function [y_ref, dot_y_ref, yaw_ref, dot_yaw_ref,dot_YAW_des,YAW_des,X_des,Y_des,Curvature] = sensor_reference(egoVehicle, sensor, scenario)
%% CONTROL REFERENCE

% vehicle data

allPoses = actorPoses(scenario);


% reading lines on the sensor horizon; first object is the lane boundary
% and the second object is the centerline

lanes = laneBoundaries(egoVehicle, 'XDistance', linspace(0, sensor.MaxRange, 100));



% road curvature
Curvature = (lanes(2).Curvature(:,1)+lanes(1).Curvature(:,1))/2;



ego_reference_x = (lanes(2).Coordinates(:,1)+lanes(1).Coordinates(:,1))/2;

ego_reference_y = (lanes(2).Coordinates(:,2)+lanes(1).Coordinates(:,2))/2;


% reference in vehicle coordinates; each vector is 10x1
x_ref=ego_reference_x(11:9:100,1);
x_ref2=ego_reference_x(13:8:100,1);
dot_x_ref=diff(ego_reference_x(13:8:100,1));



y_ref=ego_reference_y(11:9:100,1);
y_ref2=ego_reference_y(13:8:100,1);
dot_y_ref=diff(ego_reference_y(13:8:100,1));



ego_yaw_ref=atan2(ego_reference_y(11:9:100,1),ego_reference_x(11:9:100,1));

yaw_ref=ego_yaw_ref;

ego_yaw_ref2=atan2(-ego_reference_y(13:8:100,1),ego_reference_x(13:8:100,1));

dot_yaw_ref=diff(ego_yaw_ref2);

v_ref = sqrt((y_ref.^2+x_ref.^2));
v_ref2 = sqrt((y_ref2.^2+x_ref2.^2));


% reference in global coordinates; each vector is 10x1
X_c=allPoses.Position(1);
Y_c=allPoses.Position(2);
YAW_c=pi/180*allPoses.Yaw;


X_des = X_c + v_ref.*cos(ego_yaw_ref+YAW_c);

Y_des = Y_c + v_ref.*sin(ego_yaw_ref+YAW_c);

YAW_des= 180/pi*atan2(Y_des-Y_c,X_des-X_c);


% extra element need for calculating dot_YAW_des

X_des2 = X_c + v_ref2.*cos(ego_yaw_ref2+YAW_c);

Y_des2 = Y_c + v_ref2.*sin(ego_yaw_ref2+YAW_c);

YAW_ref2 =  180/pi*atan2(Y_des2-Y_c,X_des2-X_c);

dot_YAW_des=diff(-YAW_ref2);






% taking into account the track interrupt
%       - in MATLAB Driving Scenario Designer the road start and the end
%       can't be connect
%       - this is a problem for the sensor because when an interrupt
%       appears on the sensor horizon, sensor puts NAN values on its output
%       - this is compensated so that the interrupt is placed on a straight
%       part of the track and for the vehicle at that part a constant speed
%       and course(delta_f=0) have been set
% when the sensor starts to read the interrupt, reset all reference vectors
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
