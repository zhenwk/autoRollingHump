%%%%%%%%%%%%%%%%%%
%  Rolling Hump  %
%%%%%%%%%%%%%%%%%%
clear all;
close all;
clc
%%
%Setup 
start=0;
count=0;
[CurveInfo,SnakeInfo,TimeStruct]=setupInfo();

%%
%initialize
Initialize();

%check and wait for the simulator
FullsensorState = load('sensorData_cp.txt'); 
disp('Wait for simulator...');
while isempty(FullsensorState)
    FullsensorState = load('sensorData_cp.txt'); 
    pause(0.001);
end
disp('Wait 8s for the first P-control...')
pause(8);

disp('Rolling Started!');
FullsensorState = load('sensorData_cp.txt'); 
TimeCompensate = FullsensorState(1,1);

%old joint angles
oldJointAngle=zeros(1,16);

%%
while true%toc<20%
    start=1;
    %read sensorstate
    FullsensorState = load('sensorData_cp.txt');
    if isempty(FullsensorState)
        pause(0.001);
        continue;
    end
    sensorState = FullsensorState(:,2:end);
    SimTime = FullsensorState(1,1) - TimeCompensate;
    
    %Generate curve
    [CurveInfo]=generateCurve(CurveInfo, sensorState);
    
    %Convert xyz point cloud into curvature and torsion
    [CurveInfo]=xyz2ct(CurveInfo,SnakeInfo);
    
    %Calculate joint angles
    [jointAngle]=ct2jointAngle(TimeStruct, CurveInfo, SnakeInfo);
    
    angleDiff=jointAngle-oldJointAngle;
    oldJointAngle=jointAngle;
    if any(abs(angleDiff) > 0.2)
        count=count+1;
        if count>1
            return
        end
    end
    %save jointangles and make a copy
    saveTxt(jointAngle);
    
    TimeStruct.Phi_0=5*SimTime;
    pause(0.005);
    
end