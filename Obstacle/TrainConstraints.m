nex = 1e3;
    
MakeParameters;    
    
MP  = zeros(2,2*nex);
VP  = zeros(1,2*nex);
MG  = zeros(3,2*nex);
VG  = zeros(1,2*nex);
MO  = zeros(env.O.mapN,2*nex);
VO  = zeros(1,2*nex);

for k=1:nex
    
%     if mod(k,1e2)==1
%         k
%     end
    
    [env,y]     = init_sim(env);
    
    psense      = SenseProprioceptive(y);
    gsense      = SenseGoal(y);
    osense      = SenseObstacles(env,y);
    
    pnonsense   = psense + 0.1*randn(length(psense),1);
    gnonsense   = gsense + 0.1*randn(length(gsense),1);
    ononsense   = osense + 0.1*randn(length(osense),1);
    
    MP(:,k)     = psense;
    VP(:,k)     = 0;
    MP(:,nex+k) = pnonsense;
    VP(:,nex+k) = 1;
    
    MG(:,k)     = gsense;
    VG(:,k)     = 0;
    MG(:,nex+k) = gnonsense;
    VG(:,nex+k) = 1;
    
    MO(:,k)     = osense;
    VO(:,k)     = 0;
    MO(:,nex+k) = ononsense; 
    VO(:,nex+k) = 1;
    
end

save('constraints.mat','MP','VP','MG','VG','MO','VO');