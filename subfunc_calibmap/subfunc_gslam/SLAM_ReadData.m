function [ odo, mark ] = SLAM_ReadData( odo, mark )
%SLAM_READDATA Summary of this function goes here
%   Detailed explanation goes here
mark = Pick_Mark(mark);
[odo,mark] = Pick_Odo(odo,mark);
end

