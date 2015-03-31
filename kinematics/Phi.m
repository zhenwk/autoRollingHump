function [ Phi ] = Phi( Phi_0,Integ, s ,l,numNode)
%This calculates the value of Phi
    if(s<=16*l)
        Phi=Phi_0+Integ(int16(s/(16*l)*numNode));
    else
        Phi=Phi_0+Integ(numNode);
    end

end
