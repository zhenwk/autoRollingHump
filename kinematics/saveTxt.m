function [  ] = saveTxt( jointAngle )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
fid=fopen('desiredAngles.txt','w+');
fprintf(fid,'%.6f\t',jointAngle);
fprintf(fid,'\n');
fclose(fid);
copyfile('desiredAngles.txt','desiredAngles_cp.txt');

end

