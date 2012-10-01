close all;
clear all;
load env.mat;
load lcontroller.mat;
addpath('../');
[env,x]=init_sim(env);
x(1:2) = 0;
G = SimAnimation(env,[],x,0);
%axis([-200 200 -200 200]);
delete(G.goal);
drawnow;

c = 1;

while 1
    [x_new,y_new] = ginput(1);
    if isempty(x_new)
        break;
    end
    xs(c) = x_new;
    ys(c) = y_new;
    line(xs(1:c),ys(1:c),'Color','r');
    c = c+1;
end

ps = zeros(2,100);
len = 1;

for inpt=1:(c-1)
    
    while 1
       
        ps(:,len) = x(1:2);
        len = len + 1;
        y = [x(1)-xs(inpt);x(2)-ys(inpt);x(3:4)];
        if norm(y(1:2)/100) < 0.1
            break;
        end
        u = lcontroller.FProp(EgoCentric(y));
        x = SimDynamics(env,x,u,8);
        G = SimAnimation(env,G,x,u);
        line(ps(1,1:(len-1)),ps(2,1:(len-1)),'Color','g');
        %axis([-200 200 -200 200]);
        drawnow;
        
    end
    
end