function [cst,grad] = AdjointCost(env,LT,NM,ms,psense,gsense,osense,pforward,gforward,oforward,ocdecoder,rels)

    % Define data structures for forward and backward computations
    ms          = reshape(ms,[NM,LT]);
    
    NP          = size(psense,1);
    NG          = size(gsense,1);
    NO          = size(osense,1);
    
    psenses     = zeros(NP,LT+1);
    gsenses     = zeros(NG,LT+1);
    osenses     = zeros(NO,LT+1);
    
    psenses(:,1) = psense;
    gsenses(:,1) = gsense;
    osenses(:,1) = osense;
    
    delps       = zeros(NP,1);
    delgs       = zeros(NG,1);
    delos       = zeros(NO,1);    
    
    gradms      = zeros(NM,LT);
    gradpsenses = zeros(NP,LT+1);
    gradgsenses = zeros(NG,LT+1);
    gradosenses = zeros(NO,LT+1);
    
    phids       = zeros(pforward.N1,LT);
    ghids       = zeros(gforward.N1,LT);
    ohids       = zeros(oforward.N1,env.O.mapN,LT);
        
    % Define Loss Functions with Parameters
    relg        = rels(1);
    relo        = rels(2);
    LossMotFn   = @(mot,nil) LossMot(mot);
    LossPropFn  = @(psense,nil) LossProp(psense);
    LossGoalFn  = @(gsense,nil) relg*LossGoal(gsense);
    LossObsFn   = @(osense,nil) relo*FProp(ocdecoder,osense);
    LossAllFn   = @(mot,psense,gsense,osense,rosense,nil) (LossPropFn(psense) ...
                                            + LossGoalFn(gsense) ...
                                            + LossObsFn(osense) ...
                                            + LossMotFn(mot));
    % precompute indices
    midxs   = 1:NM;
    pidxs   = (NM+1):(NM+NP);
    gidxs   = (NM+NP+1):(NM+NP+NG);
    oidxs   = (NM+NP+1):(NM+NP+NO);   
    
    % For model error minimization
    rpsenses    = zeros(size(psenses));
    rgsenses    = zeros(size(gsenses));
    rosenses    = zeros(size(osenses));
                   
    cst     = 0;
                                                
    % Forward Pass    
    for t=1:LT
        
        % Motor commands
        mot             = ms(:,t);                 
        gradms(:,t)     = NumericalGradient(LossMotFn,mot,[]);
        
        %% Proprioceptive Model
        delps   = FProp(pforward,[mot;psenses(:,t)]);
        psenses(:,t+1) = psenses(:,t) + delps;
                
        % store hidden unit values
        phids(:,t) = pforward.r1;
        
        % instantaneous gradient
        gradpsenses(:,t+1) = NumericalGradient(LossPropFn,psenses(:,t+1),[]);
        
        %% Goal Model
        delgs   = FProp(gforward,[mot;psenses(:,t);gsenses(:,t)]);
        gsenses(:,t+1) = gsenses(:,t) + delgs;
        
        % store hidden unit values
        ghids(:,t) = gforward.r1;        
                
        % instantaneous gradient
        gradgsenses(:,t+1) = relg*NumericalGradient(LossGoalFn,gsenses(:,t+1),[]);
        
        %% Obstacle Model        
        delos          = FProp(oforward,[mot;psenses(:,t);osenses(:,t)]);
        osenses(:,t+1) = osenses(:,t) + delos;
        
        % store hidden unit values
        ohids(:,t) = oforward.r1;
        
        % instantaneous gradient
        FProp(ocdecoder,osenses(:,t+1));
        gradosenses(:,t+1) = relo*BProp(ocdecoder,1);
        
        %% Calculate Total Cost
        cst = cst + LossAllFn(mot,psenses(:,t+1),gsenses(:,t+1),...
                                osenses(:,t+1),[]);
                
    end
              
    % Backward Pass    
    for t=(LT+1):-1:2
        
        % reset hidden units and backpropagate
        pforward.r1     = phids(:,t-1);
        gforward.r1     = ghids(:,t-1);
        oforward.r1     = ohids(:,t-1);
                                            
        dbp             = BProp(pforward,gradpsenses(:,t));
        dbg             = BProp(gforward,gradgsenses(:,t));               
        dbo             = BProp(oforward,gradosenses(:,t));
                
        % Update error gradients
        gradms(:,t-1)       = gradms(:,t-1) ...
                                + dbp(midxs) + dbg(midxs) + dbo(midxs);
        
        gradpsenses(:,t-1)  = gradpsenses(:,t-1) + gradpsenses(:,t) ...
                                + dbp(pidxs) + dbg(pidxs) + dbo(pidxs);

        gradgsenses(:,t-1)  = gradgsenses(:,t-1) + gradgsenses(:,t) ...
                                + dbg(gidxs);
                                                   
        gradosenses(:,t-1)  = gradosenses(:,t-1) + gradosenses(:,t) ...
                                + dbo(oidxs);
        
    end
    
    cst  = cst/LT;
    grad = gradms(:)/LT;
            
end

function loss = LossMot(mot)

    % add cost due to not satisfying constraints
    % mot(1) in (0,1)
    % norm(mot(2:3))^2 = 1

    loss = 0;
    
    mot1lb  = 0.1;
    mot1ub  = 0.9;
    if mot(1) < mot1lb
        loss = loss + 10*(mot(1) - mot1lb)^2;
    elseif mot(1) > mot1ub
        loss = loss + 10*(mot(1) - mot1ub)^2;
    end
    
    loss = loss + 10*(norm(mot(2:3))^2 - 1)^2;

end

function loss = LossProp(psense)

    loss = 0;
    
end

function loss = LossGoal(gsense)

    loss = gsense(1);

end