function [verified] = check_epipole(F,e)
    verified = true;
    %P = set di punti nel nella camera sinistra C
    %e = epipolo nel piano destro della camera C'

    %Fx = l epipolar line
    %l'epipole deve stare nelle epipolar line di x per ogni x
    %e quindi il loro prodotto scalare deve dare zero (l*e =0)

    if(abs(e'*F) >= 10^-9) 
        verified = false;
    end

end