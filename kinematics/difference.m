function [ dy ] = difference( y, x )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    dim_y=size(y,2);
    dim_x=size(x,2);
    if dim_y<=dim_x
        dim_dy=dim_y-1;
    else
        dim_dy=dim_x-1;
    end

    dy=zeros(1,dim_dy);

    for i=1:dim_dy
        dy(i)=(y(i+1)-y(i))./(x(i+1)-x(i));
    end

end

