uictrls = {};
uictrls.fig  = figure('Position',[10,800,400,85],'NumberTitle','off','DoubleBuffer','on',...
        'BackingStore','on','Renderer','OpenGL',...
        'Name','Simulation','MenuBar','none');
    
clf(uictrls.fig);    

simdisptoggle   = uicontrol(uictrls.fig,'Style','checkbox','Position',[20 10 150 20],...
            'String','Display Simulation','Value',1);
newbutton      = uicontrol(uictrls.fig,'Style','pushbutton','Position',[50 40 150 20],...
            'String','New Simulation','Value',0,'Callback',{'new_simulation'});
quitbutton      = uicontrol(uictrls.fig,'Style','pushbutton','Position',[220 40 150 20],...
            'String','Quit Simulation','Value',0,'Callback',{'quit_simulation'});
jiggletoggle   = uicontrol(uictrls.fig,'Style','checkbox','Position',[150 10 150 20],...
            'String','Jiggle Obstacles','Value',0);        
        
uictrls.simdisptoggle   = simdisptoggle;
uictrls.newbutton       = newbutton;
uictrls.quitbutton      = quitbutton;
uictrls.jiggletoggle    = jiggletoggle;