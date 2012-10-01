results_hlc     = load('results+1.mat');
results_llc     = load('results+2.mat');

results_hlc     = results_hlc.results;
results_llc     = results_llc.results;

logging_hlc     = results_hlc.logging;
logging_llc     = results_llc.logging;

ntrials         = length(logging_hlc);

%% Compute cost histogram based on logging
costs_hlc       = zeros(3,ntrials);
costs_llc       = zeros(3,ntrials);

for trial=1:ntrials
    
    env                 = results_hlc.envs{trial};        
    [cost_hlc,collision_hlc]  = ComputeTrialCost(env,logging_hlc,trial);
    env                 = results_llc.envs{trial};        
    [cost_llc,collision_llc]  = ComputeTrialCost(env,logging_llc,trial);    
    
    costs_hlc(:,trial)      = cost_hlc/logging_hlc{trial}{4};
    costs_llc(:,trial)      = cost_llc/logging_llc{trial}{4};    
    
end

figure(1);
[NH,X]  = hist(costs_hlc(3,:),50);
b       = bar(X,NH/sum(NH));
axis([0, max(X)+1, 0 0.35]);
hold on;
set(b,'FaceColor',[1 0 0]);
text(median(costs_hlc(3,:)), 0.3 ,'.','FontSize',75,'Color','r');
NL      = hist(costs_llc(3,:),X);
c       = bar(X,NL/sum(NL),'BarWidth',0.8);
d       = get(c,'Children');
set(d,'FaceAlpha',0.7);
text(median(costs_llc(3,:)), 0.3 ,'.','FontSize',75,'Color','b');
set(gca,'XTick',linspace(0,20,5));
set(gca,'YTick',linspace(0,0.3,3));
box off;


