function [mark, markNum, markSize, pt_corner_mark, odo, odoNum, dataSerialNum] = init()

%% init
clear;

%% define symbol
UNIT_MM_RAD = 0;
UNIT_M_RAD = 1;
UNIT_MM_DEG = 2;
UNIT_M_DEG = 3;
odoRecDataType = UNIT_MM_RAD;
markRecDataType = UNIT_MM_RAD;

RowCountReadingFile = 0;
%% load landmark txt files
dataSerialNum = '201512211433';
landmarkFileName = ['data/thb_local_record_' dataSerialNum '/thb_local_record_landmark_' dataSerialNum '.txt'];

markFileId = fopen(landmarkFileName,'r');
mark.stamp = [];
mark.id = [];
mark.rvec = [];
mark.tvec = [];
while ~feof(markFileId)
    line_temp = fgetl(markFileId);
    Cell_temp = strsplit(line_temp,{':',';','[',']'});
    if size(Cell_temp) == [1 13]
        line_vec = str2double(Cell_temp);
        stamp = line_vec(2);
        id = line_vec(4);
        rvec = line_vec(6:8);
        tvec = line_vec(10:12);
        if id ~= 1023
            mark.stamp = [mark.stamp; stamp];
            mark.id = [mark.id; id];
            mark.rvec = [mark.rvec; rvec];
            mark.tvec = [mark.tvec; tvec];
        end
    else
        error('Input file error: format error in mark record file!')
    end
end
fclose(markFileId);

%% load odometry record file
odoFileName = ['data/thb_local_record_' dataSerialNum '/thb_local_record_odo_' dataSerialNum '.txt'];
odoFileId = fopen(odoFileName,'r');
odo.stamp = [];
odo.x = [];
odo.y = [];
odo.theta = [];
while ~feof(odoFileId)
    line_temp = fgetl(odoFileId);
    Cell_temp = strsplit(line_temp,{':',';','='});
    if size(Cell_temp) == [1 15]
        line_vec = str2double(Cell_temp);
        stamp = line_vec(2);
        x = line_vec(4);
        y = line_vec(6);
        if odoRecDataType == UNIT_MM_DEG
            theta = line_vec(8)*pi/180;
        else
            theta = line_vec(8);
        end        
        if id ~= 1023
            odo.stamp = [odo.stamp; stamp];
            odo.x = [odo.x; x];
            odo.y = [odo.y; y];            
            odo.theta = [odo.theta; theta];
        end
        RowCountReadingFile = RowCountReadingFile + 1;
        if(mod(RowCountReadingFile,500) == 0)
            RowCountReadingFile
        end
    else
        error('Input file error: format error in odometry record file!')
    end
end
fclose(odoFileId);

% load record file
% recordFileName = 'data/thb_local_record_201412161323/thb_local_record_201412161323.mat';
% load(recordFileName);

%% set parameters
temp = size(mark.stamp);
markNum = temp(1);
temp = size(odo.stamp);
odoNum = temp(1);
markSize = 60; % length of the tag, mm

% define corner w.r.t. mark frame
% y axis perpendicular to the gound
pt_corner_mark = [
    0.5*markSize 0 0.5*markSize 1;
    0.5*markSize 0 -0.5*markSize 1;
    -0.5*markSize 0 0.5*markSize 1;
    -0.5*markSize 0 -0.5*markSize 1;
    ];

