function [ CurveInfo ] = generateCurve( CurveInfo, sensorState)
%This function generates the curve point cloud based on the 
%sensor state.

CurveInfo.t=0:0.001:3*pi/2;

%read the information of the curve
Am=CurveInfo.Am; % the radius of the arc
A=CurveInfo.A;
t=CurveInfo.t;
u=CurveInfo.u;
D=CurveInfo.D;

%Height Limit 
heightLimit=40;

%Process the sensor state
%Naive way to find the sensor triggered by the ground
sumRow=sum(sensorState);
[~,IdxBottomSens]=max(sumRow);

%Current Curve = Last Curve + Current sensor state - Bottom Sensor State
CurveInfo.alpha=CurveInfo.alpha+sum(sensorState,2)-sensorState(:,IdxBottomSens);
% disp(['Bottom Sensor is: ',num2str(IdxBottomSens)]);

for i=1:17
    CurveInfo.alpha(i)=CurveInfo.alpha(i)-min(0.1, CurveInfo.alpha(i));
    
    if CurveInfo.alpha(i)>heightLimit
        CurveInfo.alpha(i)=heightLimit;
    end
end
disp(num2str(CurveInfo.alpha(3:15)'));

% CurveInfo.alpha = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]';

%generate point cloud
CurveInfo.r.x=Am*cos(t);
CurveInfo.r.y=Am*sin(t);
CurveInfo.r.z=0;

for i=1:17 %build up CurveInfo.r.z base on sensorState
    CurveInfo.r.z=CurveInfo.r.z+CurveInfo.alpha(i)*A*exp(-(t-u(i)).^2./D(i)^2);
end

end

