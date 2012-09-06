function DrawSubgoal(env,G,m2,x)

    subgoal_pos = ConvertBearingToAbsolutePosition(env.E,m2,x);
    structP     = struct;
    structP.w   = 6;
    structP.l   = 6;
    subgoal_pos(3) = 0;
    plan_pts    = GetBox(structP,subgoal_pos);
    set(G.plan,'Xdata',[plan_pts(1:4,1);plan_pts(1,1)]);
    set(G.plan,'Ydata',[plan_pts(1:4,2);plan_pts(1,2)]); 
                        
end