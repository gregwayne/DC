function din = SensitivityAnalysis(net)

    N2  = net.N2;
    id  = eye(N2);
        
    din = zeros(net.N0,1);
    
    for i=1:N2
        
        din = din + BProp(net,id(:,i)).^2;        
        
    end

end