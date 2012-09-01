function H = MultipleAnimation(E,H,xs)

    if isempty(H)
        H.fig = figure();
    end
    figure(H.fig);
    clf(H.fig);

    set(H.fig,'NumberTitle','off','DoubleBuffer','on',...
        'BackingStore','on',...%'Renderer','OpenGL',...
        'Name','Truck Backer-Upper','Position',[1, 1, 650, 650]);

    title(strcat('Truck Simulation: ',date));
    axis([-100,100,-100,100]);
    axis square;
    axis off;    

    structE         = {};
    structE.w       = E.tw;
    structE.l       = E.tl;
    
    gl              = E.cl;
    n_edges         = 100;
    goal_xs         = E.cl*cos(2*pi*(1:n_edges)/n_edges);
    goal_ys         = E.cl*sin(2*pi*(1:n_edges)/n_edges);        
    H.goal          = patch(goal_xs,goal_ys,[1 0 1]);
    
    for truck=1:size(xs,2)
        
        x               = xs(:,truck);
    
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

        patch(trailer_pts(1:5,1),trailer_pts(1:5,2),[0 1 0]);
        line([cab_back(1);trailer_front(1)],[cab_back(2);trailer_front(2)],'LineWidth',3,'Color','magenta');
        patch(cab_pts(1:5,1),cab_pts(1:5,2),[0 0 1]);
    
    end
    
end