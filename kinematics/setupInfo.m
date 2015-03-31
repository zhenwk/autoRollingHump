function [ CurveInfo,SnakeInfo,TimeStruct ] = setupInfo( )
%This function setup the initial value of some important parameters
%SnakeInfo:
    %------------------
    %length of each module: l = 0.051 meter
    %------------------
    %
    %
%CurveInfo:
    %------------------
    %about D:
    %D=0.5 | Module:9 // might be final choice
    %D=0.4 | Module:8 // should be final choice
    %D=0.3 | Module:8
    %------------------

SnakeInfo.l=0.051;%Length of Module
SnakeInfo.numModule=16;%16 Modules


CurveInfo.Am=0.35;   %Radius of the arc
CurveInfo.A=0.002;  %Height of the bump
% CurveInfo.D=0.3;    %Width of the bump
CurveInfo.D=zeros(1,SnakeInfo.numModule+1);
CurveInfo.u=zeros(1,SnakeInfo.numModule+1);%Position of the bump

    
for i=1:SnakeInfo.numModule+1
    theta=abs((i-9)*SnakeInfo.l/CurveInfo.Am);
%     theta=0;
    CurveInfo.D(i)=0.35/cos(theta);
    CurveInfo.u(i)=(-SnakeInfo.l/2+i*SnakeInfo.l)/CurveInfo.Am;
end
% disp(num2str(CurveInfo.D));
CurveInfo.alpha=zeros(SnakeInfo.numModule+1,1);


TimeStruct.t=0;%Time
TimeStruct.Phi_0=0;%initial phase

disp('Initialization done!');
end

