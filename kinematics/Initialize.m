function []=Initialize()
    jointAngle=0.25*[0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1];
    saveTxt(jointAngle);

    %clear up sensorData_cp.txt
    fid=fopen('sensorData_cp.txt','w+');
    fprintf(fid,'\t');
    fclose(fid);
end