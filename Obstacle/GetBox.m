function corner = GetBox(E,x)

    theta   = x(3);    
    width   = E.w;
    length  = E.l;
    
    e1      = [cos(theta);sin(theta)];
    e2      = [-sin(theta);cos(theta)];
    
    corner      = zeros(4,2);
    corner(1,:) = [x(1);x(2)] - width*e2/2;
    corner(2,:) = corner(1,:)' + length*e1;
    corner(3,:) = corner(2,:)' + width*e2;
    corner(4,:) = corner(3,:)' - length*e1;
    
end