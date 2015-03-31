function [ OUT ] = trunc( Origin, IN )
%This function truncate the last elements of y to make it as large as x 

    sizeX=size(IN.x,2);
    OUT.x=zeros(1,sizeX);
    for i=1:sizeX
        OUT.x(i)=Origin.x(i);
    end
    
    sizeY=size(IN.x,2);
    OUT.y=zeros(1,sizeY);
    for i=1:sizeY
        OUT.y(i)=Origin.y(i);
    end
    
    sizeZ=size(IN.x,2);
    OUT.z=zeros(1,sizeZ);
    for i=1:sizeZ
        OUT.z(i)=Origin.z(i);
    end

end

