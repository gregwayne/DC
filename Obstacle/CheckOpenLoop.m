y  = yorig;
y0 = yorig;

psenses     = zeros(length(psense),NS+1);
gsenses     = zeros(length(gsense),NS+1);
osenses     = zeros(length(osense),NS+1);

rpsenses    = zeros(length(psense),NS+1);
rgsenses    = zeros(length(gsense),NS+1);
rosenses    = zeros(length(osense),NS+1);

gcosts      = zeros(1,NS+1);
ocosts      = zeros(1,NS+1);

rgcosts     = zeros(1,NS+1);
rocosts     = zeros(1,NS+1); 
rfocosts    = zeros(1,NS+1); 

rocosts(1)  = ObstacleCost(env,y);
ocosts(1)   = FProp(ocdecoder,osense0);
rfocosts(1) = FProp(ocdecoder,osense0);

psenses(:,1)      = psense0;
gsenses(:,1)      = gsense0;
osenses(:,1)      = osense0;

rpsenses(:,1)       = psense0;
rgsenses(:,1)       = gsense0;
rosenses(:,1)       = osense0;

pdevs       = zeros(1,NS+1);
gdevs       = zeros(1,NS+1);
odevs       = zeros(1,NS+1);
opdevs      = zeros(1,NS+1);

ps          = reshape(ps,[3,NS]);
yplan       = zeros(2,NS+1);
yplan(:,1)  = yorig(1:2);

for t=1:NS

    psenses(:,t+1)     = FProp(pforward,[ps(:,t);psenses(:,t)]);            
    gsenses(:,t+1)     = FProp(gforward,[ps(:,t);psenses(:,t);gsenses(:,t)]);
    osenses(:,t+1)     = FProp(oforward,[ps(:,t);psenses(:,t);osenses(:,t)]);             
    
    p           = ps(:,t);
    
    y0          = y;
    y_fic       = ConvertBearingToVirtualPosition(E,p,y);

    % Generate Controls Here
    for ct=1:env.O.periods 
        iy      = InternalState(y_fic);
        u       = FProp(lcontroller,iy);
        us(:,ct) = u;
        ys(:,ct) = y;      
        y_fic   = SimDynamics(env,y_fic,u,env.O.simCoarse);
        y       = SimDynamics(env,y,u,env.O.simCoarse);
    end

    rpsenses(:,t+1)     = SenseProprioceptive(y);
    rgsenses(:,t+1)     = SenseGoal(y);        
    rosenses(:,t+1)     = SenseObstacles(env,y);
    
    gcosts(1,t)         = gsenses(1,t);
    ocosts(1,t+1)       = FProp(ocdecoder,osenses(:,t+1));
    
    rgcosts(1,t+1)      = rgsenses(1,t+1);
    rocosts(1,t+1)      = ObstacleCost(env,y);
    rfocosts(1,t+1)     = FProp(ocdecoder,rosenses(:,t+1));
        
    yplan(:,t+1)        = y(1:2);
    
    pdevs(t+1)          = norm(psenses(:,t+1)-rpsenses(:,t+1))^2;
    gdevs(t+1)          = norm(gsenses(:,t+1)-rgsenses(:,t+1))^2;    
    odevs(t+1)          = norm(osenses(:,t+1)-rosenses(:,t+1))^2;

end

gcosts(1,NS+1)         = gsenses(1,NS+1);

y = yorig;

% disp(sprintf('real %f',FProp(ocon,rosenses(:,end))));
% disp(sprintf('fake %f',FProp(ocon,osenses(:,end))));

disp(sprintf('one step obstacle error %f',norm(osenses(:,2)-rosenses(:,2))^2));

figure(6);
subplot(2,1,1);
plot(ocosts,'r');hold on;plot(rocosts,'g');plot(rfocosts,'b');plot(gcosts,'m');hold off;
legend('predicted obstacle costs','real obstacle costs','estimated costs along real traj.','goal costs');
ylim([-5 5]);
subplot(2,1,2);
plot(odevs,'r');hold on;plot(pdevs,'b');plot(gdevs,'g');hold off;
legend('obstacle error','proprioceptive error','goal error');