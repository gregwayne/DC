opts                = struct;
opts.rng            = [];
opts.max_trials     = 1000;
opts.max_steps      = 6000;
opts.single_arena   = 0;

addpath ../Testing/;

for memory_based=1:2

    opts.memory_based   = memory_based;
    [results, opts]     = RunTest(opts);
    
    save(strcat('~/Google Drive/DC_DATA/Data/results+',num2str(memory_based),'.mat'),'results');

end
    
    
    


