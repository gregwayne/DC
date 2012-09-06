function G = SimAnimation(env,G,x,u)

    addpath([docroot '/techdoc/creating_plots/examples']);
    % adds data position function

    E               = env.E;
    structE         = struct;
    structE.w       = E.tw;
    structE.l       = E.tl;
    trailer_pts     = GetBox(structE,x(1:3));
    trailer_pts     = [trailer_pts;trailer_pts(1,1:2)];
    
    trailer_front   = (trailer_pts(2,:) + trailer_pts(3,:))'/2;     
    y               = [trailer_front + 2*[cos(x(3));sin(x(3))]; x(4)];
    
    structE.w       = E.cw;
    structE.l       = E.cl;
    cab_pts         = GetBox(structE,y);    
    cab_pts         = [cab_pts;cab_pts(1,1:2)];
    
    cab_back        = (cab_pts(1,:) + cab_pts(4,:))/2;
    trailer_front   = (trailer_pts(2,:) + trailer_pts(3,:))/2;
    wheel_angle     = u + x(4);
    
    CostSurfaceFn   = @(x1,x2) sqrt((x1/100).^2 + (x2/100).^2);
    relf            = env.O.rel_cost;
    
    [dmap,angles]   = DepthMap(env,x);
    [nmap,angles]   = SenseObstacles(env,x);
    bcol            = [1,1,0];

    if ~isstruct(G)
    
        G.fig = figure();
        figure(G.fig);
        clf(G.fig);
        
        set(G.fig,'NumberTitle','off','DoubleBuffer','on',...
            'BackingStore','on','Renderer','zbuffer',...
            'toolbar','none','menubar','none',...
            'Name','Hierarchical Neural Control','Position',[500, 200, 750, 750]);
        set(G.fig,'PaperPositionMode','auto');
        set(gca,'Position',[0 0 1 1]);        

        xd              = -500:5:500;
        [xs,ys]         = meshgrid(xd,xd);

        C   = bsxfun(CostSurfaceFn,xd,xd');
        bx  = bsxfun(@minus,xd,E.obstacles(1,:)').^2/(2*E.co^2);
        bbx = bsxfun(@minus,xd,E.obstacles(1,:)').^2/(2*E.disk^2);
        by  = bsxfun(@minus,xd,E.obstacles(2,:)').^2/(2*E.co^2);
        bby = bsxfun(@minus,xd,E.obstacles(2,:)').^2/(2*E.disk^2);
        for i=1:size(bx,2)
            for j=1:size(by,2)
                C(i,j) = C(i,j) + relf*exp(-min(bx(:,i)+by(:,j))) + relf*2*exp(-min(bbx(:,i)+bby(:,j)));
            end
        end
        
        [~,G.contour] = contourf(xs,ys,C'); 
        set(G.contour,'EdgeColor','none');
        axis square;
        G.del=250;
        axis([x(1)-G.del,x(1)+G.del,x(2)-G.del,x(2)+G.del]);
        axis off;
        
        gl              = E.disk;
        n_edges         = 100;
        [goal_xs,goal_ys] = CirclePoints([0,0],E.disk,n_edges);
        G.goal          = patch(goal_xs,goal_ys,[0 0 0]);
        uistack(G.goal,'bottom');
        
        G.obstacles     = [];
        G               = DrawObstacles(E,G,E.disk,100);
        G.left_wheel    = line([cab_pts(3,1)-cos(wheel_angle);cab_pts(3,1)+cos(wheel_angle)],...
                                [cab_pts(3,2)-sin(wheel_angle);cab_pts(3,2)+sin(wheel_angle)],...
                                'LineWidth',3,'Color','black');
        G.right_wheel   = line([cab_pts(2,1)-cos(wheel_angle);cab_pts(2,1)+cos(wheel_angle)],...
                                [cab_pts(2,2)-sin(wheel_angle);cab_pts(2,2)+sin(wheel_angle)],...
                                'LineWidth',3,'Color','black');
        G.trailer       = patch(trailer_pts(1:5,1),trailer_pts(1:5,2),[0 1 0]);
        G.link          = line([cab_back(1);trailer_front(1)],[cab_back(2);trailer_front(2)],'LineWidth',3,'Color','magenta');
        G.cab           = patch(cab_pts(1:5,1),cab_pts(1:5,2),[1 0 1]);
        G.plan          = patch(zeros(5,1),zeros(5,1),[1 0.9 0.9]);
                            
        G.lineplan      = line('LineWidth',2,'Color','m');
                                                        
        for i=1:env.O.mapN

            G.beam(i)   = line([x(1);x(1)+dmap(i)*cos(angles(i))],...
                                [x(2);x(2)+dmap(i)*sin(angles(i))],...
                                'LineWidth',1,'Color',bcol*nmap(i));

        end
        
        G.trucks        = [G.left_wheel;G.right_wheel;G.trailer;G.link;...
                            G.cab;G.plan];
                        
        uistack(G.beam,'up'); 
        uistack(G.trucks,'top'); 
        uistack(G.plan,'top');
        uistack(G.contour,'bottom');        
                
    else
        
        set(0,'CurrentFigure',G.fig);
        
        movep = 0;
        for i=1:E.nobstacles
            if norm(G.obstacles{i}{1}-E.obstacles(:,i)) > 1e-3
                movep = 1;
                break;
            end
        end
                    
        if movep % obstacle has moved      
            
            xd  = -500:5:500;
            xs  = get(G.contour,'Xdata');
            ys  = get(G.contour,'Ydata');       
            
            C   = bsxfun(CostSurfaceFn,xd,xd');
            bx  = bsxfun(@minus,xd,E.obstacles(1,:)').^2/(2*E.co^2);
            bbx = bsxfun(@minus,xd,E.obstacles(1,:)').^2/(2*E.disk^2);
            by  = bsxfun(@minus,xd,E.obstacles(2,:)').^2/(2*E.co^2);
            bby = bsxfun(@minus,xd,E.obstacles(2,:)').^2/(2*E.disk^2);
            for i=1:size(bx,2)
                for j=1:size(by,2)
                    C(i,j) = C(i,j) + relf*exp(-min(bx(:,i)+by(:,j))) + relf*2*exp(-min(bbx(:,i)+bby(:,j)));
                end
            end
            
            set(G.contour,'Zdata',C');
            G   = DrawObstacles(E,G,E.disk,100);            
            
        end
        
        uistack(G.contour,'bottom');
        
        set(gca,'XLim',[x(1)-G.del,x(1)+G.del],...
                'YLim',[x(2)-G.del,x(2)+G.del]);
        
        set(G.left_wheel,'Xdata',[cab_pts(3,1)-cos(wheel_angle);cab_pts(3,1)+cos(wheel_angle)],'Ydata',...
                            [cab_pts(3,2)-sin(wheel_angle);cab_pts(3,2)+sin(wheel_angle)]);
        set(G.right_wheel,'Xdata',[cab_pts(2,1)-cos(wheel_angle);cab_pts(2,1)+cos(wheel_angle)],'Ydata',...
                            [cab_pts(2,2)-sin(wheel_angle);cab_pts(2,2)+sin(wheel_angle)]);
        set(G.trailer,'Xdata',trailer_pts(1:5,1),'Ydata',trailer_pts(1:5,2));
        set(G.link,'Xdata',[cab_back(1);trailer_front(1)],'Ydata',[cab_back(2);trailer_front(2)]);
        set(G.cab,'Xdata',cab_pts(1:5,1),'Ydata',cab_pts(1:5,2));
        
        for i=1:env.O.mapN
            
            set(G.beam(i),'Xdata',[x(1);x(1)+dmap(i)*cos(angles(i))],...
                          'Ydata',[x(2);x(2)+dmap(i)*sin(angles(i))],...
                          'Color',bcol*nmap(i));
            
        end        
        
        obs             = zeros(E.nobstacles,1);
        for i=1:length(E.obstacles_detected)
            obs(E.obstacles_detected(i)) = 1;
        end
                    
        for i=1:E.nobstacles
            if obs(i)
                color = [1 1/3 1/3];
            else
                color = [1 1 0];
            end
            
            set(G.obstacles{i}{2},'FaceColor',color);  
        end
                
    end
    
end