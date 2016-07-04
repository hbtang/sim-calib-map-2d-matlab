function [ ifConnect ] = If_Submap_Connect( GraphDecomposeInfo, Constraint_Hash, Idx_submap_1, Idx_submap_2 )
%IF_SUBMAP_CONNECT find out if two submaps are connected

Indice_Alpha_submap_1 = find(GraphDecomposeInfo.SubmapIdx == Idx_submap_1);
Indice_Alpha_submap_2 = find(GraphDecomposeInfo.SubmapIdx == Idx_submap_2);

Constraint_12 = Constraint_Hash(Indice_Alpha_submap_1,Indice_Alpha_submap_2);
Constraint_21 = Constraint_Hash(Indice_Alpha_submap_2,Indice_Alpha_submap_1);

ifConnect = ~(isempty(find(Constraint_12,1)) & isempty(find(Constraint_21,1)));

end

