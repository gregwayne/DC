function [map,obstacle_list,angles] = ShootBeams(env,x,map,record)

    angles          = mod(x(3) + linspace(0,2*pi,env.O.mapN),2*pi);
    obstacle_list   = [];

    maxd    = 500;
    r       = env.E.disk;
    dxs     = bsxfun(@minus,env.E.obstacles,x(1:2));
    
    for i=1:length(angles)
       
        ang     = angles(i);
        cs      = [cos(ang);sin(ang)];
        angdx   = bsxfun(@times,dxs,cs);
        
        sadx    = -2*sum(angdx,1);
        rt      = sqrt(sadx.^2 - 4*(sum(dxs.^2,1) - r^2));
        pdists  = (-sadx + rt)/2;
        mdists  = (-sadx - rt)/2;
                        
        pdi     = imag(pdists);
        mdi     = imag(mdists);
        
        pdists  = (pdi==0).*pdists + (pdi~=0).*maxd;
        mdists  = (mdi==0).*mdists + (mdi~=0).*maxd;
        pdists(pdists<0)          = maxd;
        mdists(mdists<0)          = maxd;
        
        dists   = min(pdists,mdists);
        
        [D,I]   = min(dists);
        if ~isempty(I)
           
            obstacle_list = [obstacle_list,I(1)]; 
            
        end
        
        map(i)  = D;
               
    end
                
end