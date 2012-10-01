function [env,G] = JiggleObstacles(env,G)

    displace        = env.E.jiggle_rate*(rand(2,env.E.nobstacles)-0.5);
    env.E.obstacles = env.E.obstacles + displace;

    for i=1:env.E.nobstacles

        obstacle_i = G.obstacles{i};
        verts = get(obstacle_i{2},'Vertices');
        verts = verts + repmat(displace(:,i)',size(verts,1),1);
        set(obstacle_i{2},'Vertices',verts);

    end
    
end