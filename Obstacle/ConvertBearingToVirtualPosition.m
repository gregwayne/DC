function xnew = ConvertBearingToVirtualPosition(E,dp,x)

    % a supposed position of the truck itself
    xnew     = zeros(4,1);            
    ba       = atan2(dp(2),dp(3));
 
    xnew(3) = 0;
    xnew(4) = x(4) - x(3);
        
    xnew(1) = 100*dp(1)*cos(ba);
    xnew(2) = -100*dp(1)*sin(ba);
        
end