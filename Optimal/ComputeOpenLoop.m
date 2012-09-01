function [ms,cst]       = ComputeOpenLoop(Forward,siml,y,ms,...
                                            LossX,LossM,options,flgs)

    for i=1:size(flgs,1)
       
        LX          = @(x,nil) LossX(x,flgs(i,1));
        LM          = @(m,nil) LossM(m,flgs(i,2));
        
        [ms,cst]    = minFunc(@(par) ...
            ControlCost(Forward,siml,y,par,LX,LM),ms(:),options);        
        
    end

end