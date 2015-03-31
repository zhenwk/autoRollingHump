function [ angle ] = ct2jointAngle( TimeStruct,CurveInfo, SnakeInfo )
%This function takes Curvature and Torsion as input and gives joint angles

   
    angle=zeros(1,SnakeInfo.numModule);    %joint angle initialize

    %Integrate of torsion
    numNode=1000;
    Integ_tor=zeros(1,numNode+1);
    x=0;
    dx=17*SnakeInfo.l/numNode;
    ds=0.001;                    %step size of s
    i=2;
    while x<17*SnakeInfo.l  
        Integ_tor(i)=Integ_tor(i-1)+tor(CurveInfo.Torsion, CurveInfo.S, x)*dx;
        x=x+dx;
        i=i+1;
    end
    
    %Generate angle
    for i=1:SnakeInfo.numModule
        s=(i-1)*SnakeInfo.l;
        angle(i)=0;
        if mod(i,2)==0
            while s<(i+1)*SnakeInfo.l
                s=s+ds;
                k=cur(CurveInfo.Curvature, CurveInfo.S, s)*cos(Phi...
                    (TimeStruct.Phi_0,Integ_tor,s,SnakeInfo.l,numNode));
                angle(i)=angle(i)+k*ds;
            end
        elseif mod(i,2)==1
            while s<(i+1)*SnakeInfo.l
                s=s+ds;
                k=cur(CurveInfo.Curvature, CurveInfo.S, s)*sin(Phi...
                    (TimeStruct.Phi_0,Integ_tor,s,SnakeInfo.l,numNode));
                angle(i)=angle(i)+k*ds;
            end
        end
    end
%     figure
%     i=size(Integ_tor,2);
%     for n=1:i
%         plot(n,Integ_tor(n),'.');
%         hold on;
%     end
%     grid on

end

