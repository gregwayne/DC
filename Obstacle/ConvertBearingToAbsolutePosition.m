function pos = ConvertBearingToAbsolutePosition(E,m2,x)

    x_fic       = ConvertBearingToVirtualPosition(E,m2,x);
    rot_x3      = [cos(x(3)),sin(-x(3));sin(x(3)),cos(x(3))];    

    xy_diff     = -rot_x3*x_fic(1:2);
    pos         = x(1:2) + xy_diff;

end