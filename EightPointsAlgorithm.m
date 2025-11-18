function [F] = EightPointsAlgorithm(P1,P2)

    [~, n1] = size(P1);
    [~, n2] = size(P2);
    
    if (n1 ~= n2)
        F = 0;
        return
    end

    A = zeros(n1,9);
    
    for i=1: n1
        x1 = P1(1,i);
        y1 = P1(2,i);
        x2 = P2(1,i);
        y2 = P2(2,i);
        r = [x2*x1, x2*y1, x2, y2*x1, y2*y1, y2, x1, y1, 1];
        A(i,:) = r;

    end

    [U, D, V]=svd(A);
    [rr,cc] = size(V);
    last_col_V = V(:,cc);

    F = zeros(3,3);

    % reshape riempe per colonne
    F = reshape(last_col_V,3,3);
    F = F';

    [U, D, V]=svd(F);
    
    D(3,3) = 0;

    F = U*D*(V');

end

