function [verified] = check_epipolar_constraint(F,P1,P2)
    verified = true;
    [~,n1] = size(P1);


    for i=1:n1
        if(P2(:,i)'*F*P1(:,i) > 10^-2) % matlab è stronzo
            verified = false;
            break
        end
    end

end