function [ output ] = writeYmlMatrix( outputFileId, matrixInput, matrixName, type )
%WRITEYMLMATRIX Summary of this function goes here
%   Detailed explanation goes here
space = '    ';
sz = size(matrixInput);
if type == 'f'
    fprintf(outputFileId,[matrixName ': !!opencv-matrix\n']);
    fprintf(outputFileId,[space 'rows: %d\n' space 'cols: %d\n' space 'dt: f\n'],sz(1),sz(2));
    fprintf(outputFileId,[space 'data: [ ']);
    sz = size(matrixInput);
    for i = 1:sz(1)
        for j = 1:sz(2)
            if i == sz(1) && j == sz(2)
                fprintf(outputFileId,'%f',matrixInput(i,j));
            else
                fprintf(outputFileId,'%f, ',matrixInput(i,j));
            end
        end
        fprintf(outputFileId,['\n' space space]);
    end
    fprintf(outputFileId,']\n');
elseif type == 'd'
    fprintf(outputFileId,[matrixName ': !!opencv-matrix\n']);
    fprintf(outputFileId,[space 'rows: %d\n' space 'cols: %d\n' space 'dt: i\n'],sz(1),sz(2));
    fprintf(outputFileId,[space 'data: [ ']);
    sz = size(matrixInput);
    for i = 1:sz(1)
        for j = 1:sz(2)
            if i == sz(1) && j == sz(2)
                fprintf(outputFileId,'%d',matrixInput(i,j));
            else
                fprintf(outputFileId,'%d, ',matrixInput(i,j));
            end
        end
        fprintf(outputFileId,['\n' space space]);
    end
    fprintf(outputFileId,']\n');
end
output = 0;
end

