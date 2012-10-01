function G = TruckAnimation(env,G,x,u)

    E               = env.E;
    structE         = {};
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

    if ~isstruct(G)
    
        G.fig = figure();
        figure(G.fig);
        clf(G.fig);
        
        set(G.fig,'NumberTitle','off','DoubleBuffer','on',...
            'BackingStore','on',...%'Renderer','OpenGL',...
            'Name','Truck Backer-Upper','Position',[1, 1, 650, 650]);
        title(strcat('Truck Simulation: ',date));
        axis([x(1)-150,x(1)+150,x(2)-150,x(2)+150]);
        axis square;
        axis off;
        %[im,map] = imread('Data/bigd.jpeg');
        %hi=imshow(im,map,'Parent',gca);
        
        gl              = E.cl;
        n_edges         = 100;
        [goal_xs,goal_ys] = CirclePoints([0,0],E.cl,n_edges);
        G.goal          = patch(goal_xs,goal_ys,[1 0 1]);
        G.left_wheel    = line([cab_pts(3,1)-cos(wheel_angle);cab_pts(3,1)+cos(wheel_angle)],...
                                [cab_pts(3,2)-sin(wheel_angle);cab_pts(3,2)+sin(wheel_angle)],...
                                'LineWidth',3,'Color','black');
        G.right_wheel   = line([cab_pts(2,1)-cos(wheel_angle);cab_pts(2,1)+cos(wheel_angle)],...
                                [cab_pts(2,2)-sin(wheel_angle);cab_pts(2,2)+sin(wheel_angle)],...
                                'LineWidth',3,'Color','black');
        G.trailer       = patch(trailer_pts(1:5,1),trailer_pts(1:5,2),[0 1 0]);
        G.link          = line([cab_back(1);trailer_front(1)],[cab_back(2);trailer_front(2)],'LineWidth',3,'Color','magenta');
        G.cab           = patch(cab_pts(1:5,1),cab_pts(1:5,2),[0 0 1]);
        
    else
        
        set(0,'CurrentFigure',G.fig);
        axis([x(1)-150,x(1)+150,x(2)-150,x(2)+150]);
        set(G.left_wheel,'Xdata',[cab_pts(3,1)-cos(wheel_angle);cab_pts(3,1)+cos(wheel_angle)],'Ydata',...
                            [cab_pts(3,2)-sin(wheel_angle);cab_pts(3,2)+sin(wheel_angle)]);
        set(G.right_wheel,'Xdata',[cab_pts(2,1)-cos(wheel_angle);cab_pts(2,1)+cos(wheel_angle)],'Ydata',...
                            [cab_pts(2,2)-sin(wheel_angle);cab_pts(2,2)+sin(wheel_angle)]);
        set(G.trailer,'Xdata',trailer_pts(1:5,1),'Ydata',trailer_pts(1:5,2));
        set(G.link,'Xdata',[cab_back(1);trailer_front(1)],'Ydata',[cab_back(2);trailer_front(2)]);
        set(G.cab,'Xdata',cab_pts(1:5,1),'Ydata',cab_pts(1:5,2));
        
    end
    
end

function [xs,ys] = CirclePoints(center,radius,n_edges)
    
    xs = radius*cos(2*pi*(1:n_edges)/n_edges);
    ys = radius*sin(2*pi*(1:n_edges)/n_edges);
        
end