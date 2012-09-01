function [cst,grad] = DHCost(env,ps,psense,gsense,osense,pforward,gforward,oforward)

    LT      = numel(ps)/3;
    ps      = reshape(ps,3,LT);   
    psenses = zeros(size(psense,1),LT+1);
    gsenses = zeros(size(gsense,1),LT+1);
    %osenses = zeros(size(osense,1),LT+1);

    gradp      = zeros(size(ps));
    gradpsense = zeros(size(psense,1),LT+1);
    gradgsense = zeros(size(gsense,1),LT+1);
    %gradosense = zeros(size(osense,1),LT+1);    
    
    phids   = zeros(pforward.N1,LT);
    ghids   = zeros(gforward.N1,LT);
    %ohids   = zeros(oforward.N1,LT);
    
    psenses(:,1) = psense;
    gsenses(:,1) = gsense;
    %osenses(:,1) = osense;
    
    co          = env.E.co;
    LossPFn     = @(p,nn) LossP(p);
    LossPropFn  = @(psense,nn) LossProp(psense);
    LossGoalFn  = @(gsense,nn) LossGoal(gsense,co);
    LossObsFn   = @(osense,nn) LossObs(osense,co);
    LossAllFn   = @(p,psense,gsense,osense,nn) (LossPropFn(psense) ...
                                            + LossGoalFn(gsense) ...
                                            + LossObsFn(osense) ...
                                            + LossPFn(p));
                                            
    cst         = 0;
                
    for t=1:LT
       
        p               = ps(:,t);
        
        dps             = FProp(pforward,[p;psense]);
        phids(:,t)      = pforward.r1;
        dgs             = FProp(gforward,[p;psense;gsense]); 
        ghids(:,t)      = gforward.r1;        
        %dos             = FProp(oforward,[p;psense;osense]);
        %ohids(:,t)      = oforward.r1;
        
        psense = psense + dps;
        gsense = gsense + dgs;
        %osense = osense + dos;
        
        psenses(:,t+1)  = psense;
        gsenses(:,t+1)  = gsense;
        %osenses(:,t+1)  = osense;
        
        gradpsense(:,t+1)   = NumericalGradient(LossPropFn,psense,[]);
        gradgsense(:,t+1)   = NumericalGradient(LossGoalFn,gsense,[]);
        %gradosense(:,t+1)   = NumericalGradient(LossObsFn,osense,[]);
        gradp(:,t)          = NumericalGradient(LossPFn,p,[]);
                        
        cst             = cst + LossAllFn(p,psense,gsense,osense);
        
    end
        
    for t=(LT+1):-1:2
       
        pforward.r1     = phids(:,t-1);
        gforward.r1     = ghids(:,t-1);
        %oforward.r1     = ohids(:,t-1);
        
        gradpforward    = BProp(pforward,gradpsense(:,t));
        gradgforward    = BProp(gforward,gradgsense(:,t));
        %gradoforward    = BProp(oforward,gradosense(:,t));
        
        gradp(:,t-1)    = gradp(:,t-1) ...
                            + gradpforward(1:3) ...
                            + gradgforward(1:3);% ...
                            %+ gradoforward(1:3);     
        
        gradpsense(:,t-1) = gradpsense(:,t-1) + gradpsense(:,t) ...
                            + gradpforward(4:5) ...
                            + gradgforward(4:5);% ...
                            %+ gradoforward(4:5);
                        
        gradgsense(:,t-1) = gradgsense(:,t-1) + gradgsense(:,t) ...
                            + gradgforward(6:end);
                        
%         gradosense(:,t-1) = gradosense(:,t-1) + gradosense(:,t) ...
%                             + gradoforward(6:end);                         
        
    end
        
    grad        = gradp(:)/LT;
    cst         = cst/LT;
    
end

function loss = LossP(p)

    % add cost due to not satisfying constraints
    % p(1) in (0,1)
    % norm(p(2:3))^2 = 1

    loss = 0;
    
    p1lb  = 0.1;
    p1ub  = 0.9;
    if p(1) < p1lb
        loss = loss + 10*(p(1) - p1lb)^2;
    elseif p(1) > p1ub
        loss = loss + 10*(p(1) - p1ub)^2;
    end
    
    loss = loss + 10*(norm(p(2:3))^2 - 1)^2;

end

function loss = LossProp(psense)

    loss = 0;
    
end

function loss = LossGoal(gsense,co)

    loss = 2*gsense(1);

end

function loss = LossObs(osense,co)

    %loss = 5*exp(-(osense(1)*200)^2/(2*co^2));
    distances   = 10.^osense-1;
    loss        = 5*sum(exp(-distances.^2/(2*co^2))); 
    
end