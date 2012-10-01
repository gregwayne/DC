function dps = ConvertPositionToBearing(E,x,pos)

    dps     = zeros(3,1);
    dps(1)  = norm(x(1:2) - pos)/100;
    dx      = x(1:2) - pos;
    
    ang     = atan2(dx(2),dx(1));
    bear    = x(3) - ang;
    
    dps(2:3)  = [sin(bear);cos(bear)];

end