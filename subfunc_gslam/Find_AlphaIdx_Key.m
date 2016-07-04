function [ AlphaIdx_key, LocalIdx_key ] = Find_AlphaIdx_Key( GraphDecomposeInfo, Idx_submap )
%FIND_ALPHAIDX_KEY Summary of this function goes here
%   Detailed explanation goes here

vecIdx_tmp = find(GraphDecomposeInfo.IfKey);

AlphaIdx_key = GraphDecomposeInfo.AlphaIdx(vecIdx_tmp);
SubmapIdx_key = GraphDecomposeInfo.SubmapIdx(vecIdx_tmp);
LocalIdx_key = GraphDecomposeInfo.LocalIdx(vecIdx_tmp);

Idx_tmp = find(SubmapIdx_key == Idx_submap);
AlphaIdx_key = AlphaIdx_key(Idx_tmp);
LocalIdx_key = LocalIdx_key(Idx_tmp);

end

