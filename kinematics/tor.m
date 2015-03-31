function [ tor ] = tor( Torsion, S, Length )
%This function gives the value of torsion at given s
%tor=0;

Index=1;
while (Length>S(Index))
    Index=Index+1;
end

tor=Torsion(Index);

end

