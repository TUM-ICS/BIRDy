function [g] = pullback_metric(Xhi, P_inv, numParam)

% Construct pullback form of the affine-invariant Riemannian metric defined on P(4) to R^10 coordinate
% 2019 Taeyoon Lee

nbDOF = size(P_inv,3);
g = zeros(numel(Xhi));

counter = 0;
for k = 1:nbDOF
    for i = 1:numParam(k)
        for j = 1:numParam(k)
            e_i = zeros(13,1);
            e_i(i) = 1;
            e_j = zeros(13,1);
            e_j(j) = 1;
            V_i = pseudoInertiaMatrix_Xhi(e_i);
            V_j = pseudoInertiaMatrix_Xhi(e_j);
            g(counter + i,counter + j) = trace( P_inv(:,:,k) * V_i * P_inv(:,:,k) * V_j );
        end
    end
    counter = counter + numParam(k);
end

end

