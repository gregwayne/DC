run ../Obstacle/MakeParameters;

Gs = {};

for i=1:20
   
    nobstacles = 5*i;
    env.E.nobstacles = nobstacles;
    env.E.obstacles  = [];
    env             = init_sim(env);
    Gs{i} = SimAnimation(env,[],zeros(4,1),0);
    axis([-500 500 -500 500]);
    delete(Gs{i}.left_wheel);
    delete(Gs{i}.right_wheel);
    delete(Gs{i}.trailer);
    delete(Gs{i}.cab);
    delete(Gs{i}.link);
    delete(Gs{i}.plan);
    delete(Gs{i}.lineplan);
    for k=1:env.O.mapN
        delete(Gs{i}.beam(k));
    end
    drawnow;
    saveas(gcf,strcat('env+',num2str(nobstacles)),'png');
    close all;
    
end