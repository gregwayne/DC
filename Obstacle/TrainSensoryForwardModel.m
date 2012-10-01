parfor k=1:nex

%     if mod(k,1000)==1
%         disp(sprintf('Predictor Trial %d',k));
%     end  
%    MakeParameters;    

    [env2,y]    = init_sim(env);
    y0          = y;

% %     % remove distant and undetected obstacles
% %     map                 = zeros(env.O.mapN,1);
% %     obs_delta           = bsxfun(@minus,env.E.obstacles,y(1:2));
% %     obs_dists           = sqrt(sum(obs_delta.^2,1));
% %     ids_in_range        = find(obs_dists<300);
% %     env.E.obstacles     = env.E.obstacles(:,ids_in_range);
% %     env.E.nobstacles    = length(ids_in_range);
% %     if env.E.nobstacles==0
% %         k = k-1;
% %         continue;
% %     end
% %     
% %     [map,env.E.obstacles_detected]  = ShootBeams(env,y,map,1);         
% %     obs_to_save         = unique(env.E.obstacles_detected);
% %     env.E.obstacles     = env.E.obstacles(:,obs_to_save);
% %     env.E.nobstacles    = length(obs_to_save);    
% %     if env.E.nobstacles==0
% %         k = k-1;
% %         continue;
% %     end
     
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
                
%         if show_on
%             map                             = zeros(env2.O.mapN,1);
%             [map,env.E.obstacles_detected]  = ShootBeams(env2,y,map,1);         
%             G       = SimAnimation(env2,G,y,u);
%             set(G.plan,'position',subgoal(1:2));
%             figure(2);
%             subplot(4,1,1);
%             bar(SenseObstacles(env2,y),'b');
%             axis([1 env2.O.mapN -2 2]);
%             drawnow;
%         end
    end            
           
    odists  = SenseObstacles(env2,y);    

    MP(:,k) = [p;psense0];
    %VP(:,k) = SenseProprioceptive(y)-psense0;
    VP(:,k) = SenseProprioceptive(y);
    
    MG(:,k) = [p;psense0;gsense0];
    %VG(:,k) = SenseGoal(y)-gsense0;
    VG(:,k) = SenseGoal(y);    
         
    oc      = ObstacleCost(env2,y);
    MC(:,k) = odists;    
    VC(k)   = oc;
  
    %dodists = odists-odists0; 
    
    MO(:,k) = [p;psense0;odists0];
    %VO(:,k) = dodists;
    VO(:,k) = odists;    
        
%     if show_on
%         
%         dodists_pred = FProp(oforward,[p;psense0;odists0]);
%         dp_pred      = FProp(pforward,[p;psense0]);
%         dg_pred      = FProp(gforward,[p;psense0;gsense0]);
%         c_pred       = FProp(ocdecoder,odists);
%         
%         figure(2);
%         subplot(5,1,2);
%         plot(1:env2.O.mapN,dodists,'b',1:env2.O.mapN,dodists_pred,'r');
%         axis([1 env2.O.mapN -2 2]);
%         
%         subplot(5,1,3);
%         plot(1:2,VP(:,k),1:2,dp_pred);
%         ylim([-1 1]);
%         
%         subplot(5,1,4);
%         plot(1:3,VG(:,k),1:3,dg_pred);
%         ylim([-1 1]);
%         
%         subplot(5,1,5);
%         bar([VC(:,k),c_pred],'b');
%         drawnow;
%         pause;
%         
%     end
    
end

%save('patterns_forward.mat','MP','VP','MG','VG','MO','VO','MC','VC');