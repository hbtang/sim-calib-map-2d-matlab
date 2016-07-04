function [ GraphDecomposeInfo ] = Graph_Dcmps_by_Mark_with_Bndry( Constraint_Pool, Constraint_Hash,...
    Alpha, hash_Alpha_odoIdx, hash_Alpha_markIdx)
%GRAPH_DECOMPOSE_BY_MARK Summary of this function goes here


Constraint_Hash_sp = sparse(Constraint_Hash);
GraphDecomposeInfo = [];
num_Alpha = numel(Alpha)/3;
GraphDecomposeInfo.AlphaIdx = (1:num_Alpha)';
GraphDecomposeInfo.SubmapIdx = zeros(num_Alpha,1);
GraphDecomposeInfo.LocalIdx = zeros(num_Alpha,1);
GraphDecomposeInfo.IfKey = zeros(num_Alpha,1);
GraphDecomposeInfo.IfBoundary = zeros(num_Alpha,1);
[sz_feature,tmp] = size(hash_Alpha_markIdx);

%% set all features into different submaps
for i = (1:sz_feature)
    Idx_Alpha_markNow = hash_Alpha_markIdx(i,1);
    % set mark into submap, and as key node
    GraphDecomposeInfo.SubmapIdx(Idx_Alpha_markNow) = i;
    GraphDecomposeInfo.IfKey(Idx_Alpha_markNow) = 1;
    
    % set pose when mark detected into submap
    Indices_odo = find(Constraint_Hash_sp(:,Idx_Alpha_markNow));
    GraphDecomposeInfo.SubmapIdx(Indices_odo) = i;
end

%% set all poses into submaps
Idx_submap_now = 1;
[sz_odo, tmp] = size(hash_Alpha_odoIdx);
for i = (1:sz_odo)
    if (GraphDecomposeInfo.SubmapIdx(i) == 0)
        GraphDecomposeInfo.SubmapIdx(i) = Idx_submap_now;
    else
        Idx_submap_now = GraphDecomposeInfo.SubmapIdx(i);
    end
end

%% set boundary nodes
num_submap = max(GraphDecomposeInfo.SubmapIdx);
for i = (1:num_submap)
    for j = (1:num_submap)
        if j ~= i            
            IdxVec_Alpha_submap_i = find(GraphDecomposeInfo.SubmapIdx == i);
            IdxVec_Alpha_submap_j = find(GraphDecomposeInfo.SubmapIdx == j);
            Constraint_Hash_ij = Constraint_Hash(IdxVec_Alpha_submap_i,IdxVec_Alpha_submap_j);
            [rows, cols] = find(Constraint_Hash_ij);
            if numel(rows) ~= 0
                IdxVec_Alpha_boundary_now = IdxVec_Alpha_submap_i(rows);
                GraphDecomposeInfo.IfBoundary(IdxVec_Alpha_boundary_now) = 1;
                % set boundary node belong to submap 0
                GraphDecomposeInfo.SubmapIdx(IdxVec_Alpha_boundary_now) = 0;
            end
        end
    end
end
% set first node boundary node
GraphDecomposeInfo.IfBoundary(1) = 1;
GraphDecomposeInfo.SubmapIdx(1) = 0;

%% set localIdx for all the nodes
for i = (1:sz_feature)
    Indice_submap_now = find(GraphDecomposeInfo.SubmapIdx == i);
    GraphDecomposeInfo.LocalIdx(Indice_submap_now) = (1:numel(Indice_submap_now));
end

end

