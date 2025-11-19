function [e1, e2] = epipoles(F)
    [U, ~, V] = svd(F);
    e1 = V(:, end); 
    e2 = U(:, end);

end