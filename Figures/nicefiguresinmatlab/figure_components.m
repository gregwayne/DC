% Figure: components of tradeoff

%%
fig = figure;
set(fig,'units','centimeters','color',[1 1 1]);
set(fig,'position',[1 3 14 15]);
fig_set_print(fig);


sz = [4 3];
pan(1,1) = addaxes(sz,'left-top',fig,'left-top',[2 -0.7]);
pan(2,1) = addaxes(sz,'left-top',pan(1,1),'right-top',[2.5 0]);
for i=2:3
    for j=1:2
        pan(j,i) = addaxes(sz, 'left-top',pan(j,i-1),'left-bottom',[0 -2] );
    end
end

corder = [1 0 0; 0 0 1];
for i=1:6
    set(pan(i), 'ColorOrder', corder, 'NextPlot', 'replacechildren');
end

% pan_strings = 'ADBECF';
pan_strings = 'ABCDEF';
for i=1:6
    pan_text(i)=addaxes([1 1]*0.7,'right-top',pan(i),'left-top',[-0.7 0.7]);
    axes(pan_text(i));
    text(0,0.5,pan_strings(i),'fontsize',12,'FontWeight','bold');
    axis off;
end

styleP = { 'color'; 'linestyle' };
styleV = { 'k', '-' ; 'k', '--' };

%%
load result_numerics_vary_noise
[fldNames fldValues fldN fldM fldV] = parseFields( flds );

for j=1:size(res1,3)
    res1(:,7,j) = .5*erfc(res7(:,4,j)./sqrt(2*res7(:,5,j)));
%     res1(:,7,j) = .5*erfc(res7(:,2,j)./sqrt(2*res7(:,3,j))); % Gamma and Sigma
    res7(:,8,j) = res7(:,2,j).^2 * res6(j) / res5(j);
    res7(:,9,j) = res7(:,4,j).^2 * res6(j) / res5(j);
end

nCode = size(res1,1);
r1 = reshape(res1,[nCode size(res1,2) cellfun(@numel, flds(:,2))']);
r2 = squeeze( mean(r1,size(flds,1)+2) );
r5 = reshape(res5,[cellfun(@numel, flds(:,2))']);
r5 = squeeze( mean(r5,size(flds,1)) );
r7 = reshape(res7,[nCode size(res7,2) cellfun(@numel, flds(:,2))']);
r7 = squeeze( mean(r7,size(flds,1)+2) );


idx = [4 12];

for i=1:2
    a = idx(i);
    
    axes(pan(i,1));
    h = plot(params.codings,[ r1(:,[2 7],a)  ], 'linewidth', 2);
    set(h, styleP, styleV);
    if i<=2
        legend( 'Numerical', 'Formula', 'location', 'best' );
    end
    if i==1
        ylabel( 'Test error' );
    end
    axis tight;
    aa = axis;
    axis([0 0.5 0 min(0.5,aa(4))]);
    title( ['Noise ' num2str(fldValues( a,1 ) ) ] );
    
    axes(pan(i,2));
    h = plot(params.codings,r7(:,[8 9 ],a), 'linewidth', 2);
        set(h, styleP, styleV);

    
    if i<=2
        h=legend( '\Gamma', '\gamma',  'location', 'best' );
    end
    if i==1
        ylabel( 'Disc. factor' );
    end
    axis tight;
    aa = axis;
    axis([0 0.5 0 aa(4)]);
    
    axes(pan(i,3));
    h=plot(params.codings,1./r7(:,[3 5 ],a), 'linewidth', 2);
    set(h, styleP, styleV);
    axis tight;
    aa = axis;
    axis([0 0.5 0 aa(4)]);
    
    axis([0 0.5 0 20]);
    
    if i<=2
        legend( '1/\Sigma^2', '1/\sigma^2',  'location', 'best' );
    end
    if i==1
        ylabel( 'Gen. factor' );
    end
    
    xlabel( 'Coding level' );
    
    disp(r5(a));
%     disp(r6(a));
end

print -dpdf figure_components.pdf
