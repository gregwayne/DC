function G = MultipleAnimation(E,G,zs,us,plans,max_t)

    f               = @(x1,x2) sqrt((x1/100)^2 + (x2/100)^2) + ...
                                + 2*sum(exp(-(x1-E.obstacles(1,:)).^2/(2*E.co^2) ...
                                -(x2-E.obstacles(2,:)).^2/(2*E.co^2)));
                             
    if ~isstruct(G)
    
        G.fig = figure();
        figure(G.fig);
        clf(G.fig);
        
        set(G.fig,'NumberTitle','off','DoubleBuffer','on',...
            'BackingStore','on','Renderer','OpenGL',...
            'Name','Truck Backer-Upper','Position',[1, 1, 650, 650]);

        [xs,ys]         = meshgrid(-500:10:500,-500:10:500);
        
        C = zeros(size(xs));
        for i=1:size(xs,1)
            for j=1:size(xs,2)
                C(j,i)  = f(xs(1,i),ys(j,1));
            end
        end
        [~,G.contour] = contour(xs,ys,C); 
        set(G.contour,'EdgeColor','none');
        axis square;
        axis([-600,600,-600,600]);
        axis off;

    end
    
    set(0,'CurrentFigure',G.fig);
    gl              = E.disk;
    n_edges         = 100;
    [goal_xs,goal_ys] = CirclePoints([0,0],E.disk,n_edges);
    G.goal          = patch(goal_xs,goal_ys,[0 0 0]);
    G.obstacles     = [];
    G               = DrawObstacles(E,G,E.co,100);

    hold on;
    for t=1:(max_t-1)

    x       = zs(:,t);
    u       = us(:,t);
    plan    = plans(:,t);

    structE         = {};
    structE.w       = E.tw;
    structE.l       = E.tl;
    trailer_pts     = GetBox(structE,x(1:3));
    trailer_pts     = [trailer_pts;trailer_pts(1,1:2)];

    trailer_front   = (trailer_pts(2,:) + trailer_pts(3,:))'/2;     
    y               = [trailer_front + [cos(x(3));sin(x(3))]; x(4)];

    structE.w       = E.cw;
    structE.l       = E.cl;
    cab_pts         = GetBox(structE,y);    
    cab_pts         = [cab_pts;cab_pts(1,1:2)];

    cab_back        = (cab_pts(1,:) + cab_pts(4,:))/2;
    trailer_front   = (trailer_pts(2,:) + trailer_pts(3,:))/2;
    wheel_angle     = u + x(4);

    % left wheel
%     line([cab_pts(3,1)-cos(wheel_angle);cab_pts(3,1)+cos(wheel_angle)],...
%                             [cab_pts(3,2)-sin(wheel_angle);cab_pts(3,2)+sin(wheel_angle)],...
%                             'LineWidth',3,'Color','black');
%     % right wheel
%     line([cab_pts(2,1)-cos(wheel_angle);cab_pts(2,1)+cos(wheel_angle)],...
%                             [cab_pts(2,2)-sin(wheel_angle);cab_pts(2,2)+sin(wheel_angle)],...
%                             'LineWidth',3,'Color','black');
%     % trailer
%     patch(trailer_pts(1:5,1),trailer_pts(1:5,2),[0 (t-1)/max_t 0]);
% 
%     % link
%     line([cab_back(1);trailer_front(1)],[cab_back(2);trailer_front(2)],'LineWidth',3,'Color','magenta');
% 
%     % cab
%     patch(cab_pts(1:5,1),cab_pts(1:5,2),[0 0 (t-1)/max_t]);

    a=line(zs(1,:),zs(2,:),'LineWidth',2,'Color','k');
    set(a,'LineStyle','-.');
    % plan
    %text(plan(1),plan(2),'.','FontSize',60,'Color',[(t-1)/max_t 0 0]);
    %plan
    end
    hold off;
            
end

function [xs,ys] = CirclePoints(center,radius,n_edges)
    
    xs = radius*cos(2*pi*(1:n_edges)/n_edges)+center(1);
    ys = radius*sin(2*pi*(1:n_edges)/n_edges)+center(2);
        
end

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
        
    end

end