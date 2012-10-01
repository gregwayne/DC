function [r2,obj] = ParFProp(obj,r0)

    x1 = obj.W1*r0 + obj.b1;
    r1 = tanh(x1);
    x2 = obj.W2*r1 + obj.b2;
    r2 = x2;

    obj.r0 = r0;
    obj.r1 = r1;
    obj.r2 = r2;            

end