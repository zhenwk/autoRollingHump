function [CurveInfo] = xyz2ct( CurveInfo,SnakeInfo )
%This function take the info of x, y, z coordinates in cart and
%calculate the Curvature and Torsion at each point.
%
%        S:  length along the snake body;
% EndIndex:  End index of the snake along the spacial curve;

    r.x=CurveInfo.r.x;
    r.y=CurveInfo.r.y;
    r.z=CurveInfo.r.z;

    numNode=size(r.x,2);
    S=zeros(1,numNode);
    for i=2:size(S,2)
        S(i)=S(i-1)+...
             ((r.x(i)-r.x(i-1))^2+...
              (r.y(i)-r.y(i-1))^2+...
              (r.z(i)-r.z(i-1))^2)^0.5;
        if S(i)<=17*SnakeInfo.l
           EndIndex=i; 
        end
    end

    %Build up vector T which is the first derivative of r
    %Elements number of N is 157
    T.x=difference(r.x,S);
    T.y=difference(r.y,S);
    T.z=difference(r.z,S);

    %Build up vector N which is the nomalized first derivative of T
    dt_ds(1,:)=difference(T.x,S);
    dt_ds(2,:)=difference(T.y,S);
    dt_ds(3,:)=difference(T.z,S);
    N.norm=(dt_ds(1,:).^2+...
          dt_ds(2,:).^2+...
          dt_ds(3,:).^2).^0.5;
    N.x=dt_ds(1,:)./N.norm;
    N.y=dt_ds(2,:)./N.norm;
    N.z=dt_ds(3,:)./N.norm;
    
%     norm=((difference(T.x,S)).^2+...
%           (difference(T.y,S)).^2+...
%           (difference(T.z,S)).^2).^0.5;
% 
%     N.x=difference(T.x,S)./norm;
%     N.y=difference(T.y,S)./norm;
%     N.z=difference(T.z,S)./norm;


    %Build up vector B
    T=trunc(T, N);
    B.x=T.y.*N.z-T.z.*N.y;
    B.y=T.z.*N.x-T.x.*N.z;
    B.z=T.x.*N.y-T.y.*N.x;


    %Calculate curvature and build up its table
    Curvature=N.norm;

    %Calculate torsion and build up its table
    dB.x=difference(B.x,S);
    dB.y=difference(B.y,S);
    dB.z=difference(B.z,S); 

    N=trunc(N, dB);
    %Torsion=(dB.x./N.x+dB.y./N.y+dB.z./N.z)/3;
    %Torsion=(dB.x./N.x+dB.y./N.y)/2;
    Torsion=dB.x./N.x;
    
    CurveInfo.Curvature=Curvature;
    CurveInfo.Torsion=medfilt1(Torsion,11);
    CurveInfo.S=S;
    CurveInfo.EndIndex=EndIndex;
    
%%
%Torsion info processing and plot

%     i=1:size(N.x,2);
%     plot(i,N.x);
%     hold on;
%     plot(i,dB.x);
%     grid on;

%     nTorsion=1:size(Torsion,2);
%     plot(1:nTorsion,Torsion);
%     drawnow
    
%     epsilon=0.5;
%     for i=2:size(Torsion,2)
%         if (abs(dB.x(i))<epsilon) && (abs(N.x(i))<epsilon)
%             Torsion(i)=Torsion(i-1)*2-Torsion(i-2);
%         end
%     end

%     disp('Curvature & Torsion');
%     i=1:size(Torsion,2);
%     plot(i,Torsion(i),'r');
%     hold on;
end

