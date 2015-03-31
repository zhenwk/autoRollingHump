function [ cur ] = cur( Curvature, S, Length )
%This function calculate the value of curvature at given s.

%cur=5;   %2D arc
Index=1;
while (Length>S(Index))
    Index=Index+1;
end

cur=Curvature(Index);

end