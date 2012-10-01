function [serx,sery] = DoubleSeries(xs,ys1,ys2,col)
    
    xs      = xs(:);
    ys1     = ys1(:);
    ys2     = ys2(:);
    
    serx    = [xs;flipud(xs);xs(1)];
    sery    = [ys1;flipud(ys2);ys1(1)];
        
end