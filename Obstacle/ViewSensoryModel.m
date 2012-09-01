nex = 5e4;

MakeParameters;

Forward         = @(x,u) SimDynamics(env.E,x,u,env.O.simCoarse);
addpath ../Truck/;
addpath ..;
load 'lcontroller.mat';
load 'pforward.mat';
load 'gforward.mat';
load 'oforward.mat';
load 'ocdecoder.mat';
G           = [];

MP          = zeros(3+2,nex);
VP          = zeros(2,nex);
MG          = zeros(3+2+3,nex);
VG          = zeros(3,nex);
MO          = zeros(3+2+env.O.mapN,nex);
VO          = zeros(env.O.mapN,nex);
MC          = zeros(env.O.mapN,nex);
VC          = zeros(1,nex);

show_on     = 1;

for k=1:nex  
    
    [env2,y]     = init_sim(env);
    y0          = y;
     
    p           = zeros(3,1);
    p(1)        = rand;   
    rangle      = rand*2*pi;
    p(2)        = sin(rangle);
    p(3)        = cos(rangle);
    
    psense0     = SenseProprioceptive(y0);
    gsense0     = SenseGoal(y0);        
    odists0     = SenseObstacles(env2,y0);
        
    y_fic       = ConvertBearingToVirtualPosition(env2.E,p,y);
    subgoal     = ConvertBearingToAbsolutePosition(env2.E,p,y);
              
    % Generate Controls Here
    for t=1:env2.O.periods 
        iy      = InternalState(y_fic);
        u       = FProp(lcontroller,iy); 
        y       = SimDynamics(env2,y,u,env2.O.simCoarse);
        y_fic   = SimDynamics(env2,y_fic,u,env2.O.simCoarse);
                
        if show_on
            map                             = zeros(env2.O.mapN,1);
            [map,env.E.obstacles_detected]  = ShootBeams(env2,y,map,1);         
            G       = SimAnimation(env2,G,y,u);
            set(G.plan,'position',subgoal(1:2));

        end
    end            
           
    MP(:,k) = [p;psense0];
    %VP(:,k) = SenseProprioceptive(y)-psense0;
    VP(:,k) = SenseProprioceptive(y);
    
    MG(:,k) = [p;psense0;gsense0];
    %VG(:,k) = SenseGoal(y)-gsense0;
    VG(:,k) = SenseGoal(y);    
    
    odists  = SenseObstacles(env2,y);    
    
    oc      = ObstacleCost(env2,y);
    MC(:,k) = odists;    
    VC(k)   = oc;
  
    %dodists = odists-odists0; 
    
    MO(:,k) = [p;psense0;odists0];
    %VO(:,k) = dodists;
    VO(:,k) = odists;    
        
    if show_on
        
%         dodists_pred = FProp(oforward,[p;psense0;odists0]);
%         dp_pred      = FProp(pforward,[p;psense0]);
%         dg_pred      = FProp(gforward,[p;psense0;gsense0]);
%         c_pred       = FProp(ocdecoder,odists);
        odists_pred = FProp(oforward,[p;psense0;odists0]);
        p_pred      = FProp(pforward,[p;psense0]);
        g_pred      = FProp(gforward,[p;psense0;gsense0]);
        c_pred      = FProp(ocdecoder,odists);
        
        figure(2);
        subplot(6,1,1);        
        %bar([SenseObstacles(env2,y),dodists_pred+odists0]);
        bar([SenseObstacles(env2,y),odists_pred]);        
        axis([1 env2.O.mapN -2 2]);
        
        subplot(6,1,2);
        %plot(1:env2.O.mapN,dodists,'b',1:env2.O.mapN,dodists_pred,'r');
        plot(1:env2.O.mapN,odists,'b',1:env2.O.mapN,odists_pred,'r');        
        axis([1 env2.O.mapN -2 2]);
        
        subplot(6,1,3);
        plot(1:2,VP(:,k),1:2,p_pred);
        ylim([-1 1]);
        
        subplot(6,1,4);
        plot(1:3,VG(:,k),1:3,g_pred);
        ylim([-1 1]);
        
        subplot(6,1,5);
        bar([VC(:,k),c_pred],'b');
        
        subplot(6,1,6);
        bar([oc,FProp(ocdecoder,odists_pred)]);
        drawnow;        
        
%         disp(sprintf('one step proprio. error %f',norm(p_pred-VP(:,k))^2));
%         disp(sprintf('one step goal error %f',norm(g_pred-VG(:,k))^2));
%         disp(sprintf('one step obstacle error %f\n',norm(odists_pred-odists)^2));
%         norm(odists_pred-odists)^2/env.O.mapN
        
        VG(1,k)
        oc
        
        pause;
        
    end
        
end