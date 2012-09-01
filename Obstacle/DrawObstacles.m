function G = DrawObstacles(E,G,radius,n_edges)
    
    if isempty(G.obstacles)
        G.obstacles = {};
    else
       
        for i=1:E.nobstacles

            delete(G.obstacles{i}{2});
            
        end
        
    end
    
    for i=1:E.nobstacles
       
       center           = E.obstacles(:,i);
       [xs,ys]          = CirclePoints(center,radius,n_edges);
       G.obstacles{i}   = {E.obstacles(:,i),patch(xs,ys,[1 1 0])};
       set(G.obstacles{i}{2},'EdgeColor','none');
       uistack(G.obstacles{i}{2},'bottom');
       
    end

end