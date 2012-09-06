function h = nicefigs( operation, varargin)
% Create figures for papers
%
% fig = nicefigs( 'create', [ W H ] );
%    create fig of W x H centimeters
% fig = nicefigs( 'create', [ W H ], 0 );
%    Figure size is full paper instead of tight
%
%
%
%
% Written by Barak Blumenfeld
% Modified by Omri Barak

switch lower(operation)
    case 'create'
        h = figure;
        set(h,'units','centimeters','color',[1 1 1]);
        set(h,'position',[ 1 1 varargin{1}]);
        set(h,'paperunits',get(h,'units'));
        temp2=get(h,'position');
        temp3=temp2(3:4);
        tightFit = 1; % Adjust paper size to figure
        if nargin>2
            tightFit = varargin{2};
        end
        if tightFit
            set(h,'papersize',temp3*1.05);
        end
        temp=get(h,'paperSize');
        margs= (temp-temp3)/2;
        set(h,'paperposition',[margs temp2(3:4)])
end

end