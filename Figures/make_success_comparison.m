a = load('../Obstacle/Data/win_counts.mat');
b = load('../Truck/Data/win_counts.mat');
win_o = a.win_counts;
win_t = b.win_counts;

ds = 10:10:500;

Mo = zeros(50,2);
MMo = zeros(50,1);
Mt = zeros(50,2);
MMt = zeros(50,1);
for i=1:size(win_o,1)
   
    [phat_o,pci_o] = binofit(sum(win_o(i,:)),100);
    [phat_t,pci_t] = binofit(sum(win_t(i,:)),100);
    
    MMo(i) = phat_o;
    MMt(i) = phat_t;
    
    Mo(i,:) = pci_o;
    Mt(i,:) = pci_t;
        
end

pts_o = zeros(2,100);
pts_t = zeros(2,100);


for i=1:size(win_o,1)
    
    pts_o(1,i) = ds(i);
    pts_o(2,i) = Mo(i,1);
    pts_t(1,i) = ds(i);
    pts_t(2,i) = Mt(i,1);
end

for i=1:size(win_o,1)
   
    pts_o(1,50+i) = ds(end-(i-1));
    pts_o(2,50+i) = Mo(end-(i-1),2);
    pts_t(1,50+i) = ds(end-(i-1));
    pts_t(2,50+i) = Mt(end-(i-1),2);
    
end

figure(1);
a=fill(pts_t(1,:),pts_t(2,:),[0 1 1]);
hold on;
b=fill(pts_o(1,:),pts_o(2,:),[1 0 1]);
legend([a,b],'Single-Level Controller','Hierarchical Controller');
line(ds,MMt,'color',0.6*[0 1 1]);
hold on;
line(ds,MMo,'color',0.6*[1 0 1]);
xlabel('Starting Distance');
ylabel('Fraction of Trials Successful');
fixfig(1);