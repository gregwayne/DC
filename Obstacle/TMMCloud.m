addpath ..;

nex         = 1e3;

MakeParameters;

NS          = 8; % number of stages
M           = {};%zeros(3+2+env.O.mapN,nex*NS);
V           = {};%zeros(3,nex*NS);

options.Method      = 'lbfgs'; 
options.maxIter     = 1e3;
options.maxFunEvals = 1e3;
options.display     = 'off';

load( 'hcontroller.mat'   );
hcontroller0 = MakeStructFromObject(hcontroller);
load( 'pforward.mat'      );
pforward0 = MakeStructFromObject(pforward);
load( 'gforward.mat'      );
gforward0 = MakeStructFromObject(gforward);
load( 'oforward.mat'      );
oforward0 = MakeStructFromObject(oforward);
load( 'ocdecoder.mat'     );
ocdecoder0 = MakeStructFromObject(ocdecoder);

for k=1:nex
   
    hcontrollers{k} = hcontroller0;
    pforwards{k}    = pforward0;
    gforwards{k}    = gforward0;    
    oforwards{k}    = oforward0;
    ocdecoders{k}   = ocdecoder0;
    
end

ps  = zeros(3,NS);

parfor k=1:nex
     
    warning off;
    
    Mk          = zeros(3+2+env.O.mapN,NS);
    Vk          = zeros(3,NS);

    [eb,y]      = init_sim(env);
    ps          = zeros(3,NS);
            
    psense0     = SenseProprioceptive(y);
    gsense0     = SenseGoal(y);        
    osense0     = SenseObstacles(eb,y);

    psense      = psense0;
    gsense      = gsense0;
    osense      = osense0;
    
    hcontroller = hcontrollers{k};
    pforward    = pforwards{k};
    gforward    = gforwards{k};    
    oforward    = oforwards{k};
    ocdecoder   = ocdecoders{k};
            
    for t=1:NS

        ps(:,t)     = ParFProp(hcontroller,[psense;gsense;osense]); 
        
        [psense,pforward]     = ParFProp(pforward,[ps(:,t);psense]);            
        [gsense,gforward]     = ParFProp(gforward,[ps(:,t);psense;gsense]);
        [osense,oforward]     = ParFProp(oforward,[ps(:,t);psense;osense]);              

    end

    [ps,cst]    = minFunc(@(par) ParAdjointCost(eb,NS,3,par,...
                                psense0,gsense0,osense0,...
                                pforward,gforward,oforward,...
                                ocdecoder),ps(:),options);
    p       = ps(1:3); 

    M{k}    = [psense0;gsense0;osense0];
    V{k}    = p;

end

MF  = [];
VF  = [];

for k=1:nex
    
    MF  = [MF,M{k}];
    VF  = [VF,V{k}];
    
end

save('patterns_hc.mat','MF','VF');