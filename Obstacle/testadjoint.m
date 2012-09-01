MakeParameters;
[env,y]     = init_sim(env);

load 'pforward.mat';
load 'gforward.mat';
load 'oforward.mat';
load 'ocdecoder.mat';
load '../Truck/lcontroller.mat';
load 'oerror.mat';

psense      = SenseProprioceptive(y);
gsense      = SenseGoal(y);        
osense      = SenseObstacles(env,y);

NS          = 4; % number of stages
m2s         = zeros(3,NS);

for t=1:NS
    d           = rand;
    rangle      = 2*pi*rand;    
    m2s(:,t)    = [d;cos(rangle);sin(rangle)];
end

yorig=y;
tic
[cst,grad]  = AdjointCost2(env,NS,3,m2s(:),psense,gsense,osense,pforward,gforward,oforward,ocdecoder,[1,1]);
toc
y=yorig;
AdjFn   = @(x,nil) AdjointCost2(env,NS,3,x,psense,gsense,osense,pforward,gforward,oforward,ocdecoder,[1,1]);
ngrad   = NumericalGradient(AdjFn,m2s,[]);

reshape(grad,[3 NS])
reshape(ngrad,[3,NS])






