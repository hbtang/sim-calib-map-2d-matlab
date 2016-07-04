function [ Omega, Xi ] = Add_InfoMatVec( Omega, Xi, Omega_add, Xi_add, Idx_1_add, Idx_2_add )
%ADD_INFOMATVEC Summary of this function goes here
%   Detailed explanation goes here

if size(Omega_add) ~= [6 6]
    error('ERROR!!!');
end

if size(Xi_add) ~= [6 1]
    error('ERROR!!!');
end

IdxVec_1_add = 3*Idx_1_add-2 : 3*Idx_1_add;
IdxVec_2_add = 3*Idx_2_add-2 : 3*Idx_2_add;

Omega(IdxVec_1_add,IdxVec_1_add) = Omega(IdxVec_1_add,IdxVec_1_add) + Omega_add(1:3,1:3);
Omega(IdxVec_1_add,IdxVec_2_add) = Omega(IdxVec_1_add,IdxVec_2_add) + Omega_add(1:3,4:6);
Omega(IdxVec_2_add,IdxVec_1_add) = Omega(IdxVec_2_add,IdxVec_1_add) + Omega_add(4:6,1:3);
Omega(IdxVec_2_add,IdxVec_2_add) = Omega(IdxVec_2_add,IdxVec_2_add) + Omega_add(4:6,4:6);

Xi(IdxVec_1_add) = Xi(IdxVec_1_add) + Xi_add(1:3);
Xi(IdxVec_2_add) = Xi(IdxVec_2_add) + Xi_add(4:6);

end

