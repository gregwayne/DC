function [d0,obj] = BProp(obj,d2)

    f2 = 1;
    e2 = d2.*f2;
    d1 = obj.W2'*e2;
    f1 = 1-obj.r1.^2;
    e1 = d1.*f1;
    d0 = obj.W1'*e1;

end