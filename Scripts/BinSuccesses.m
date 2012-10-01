function [p_bins,c_bins]    = BinSuccesses(first_dists,successes,N)

    p_bins      = zeros(N,1);
    c_bins      = zeros(N,2);  

    rs          = {first_dists,successes};
    [vals,idxs] = sort(first_dists);
    rs          = {first_dists(idxs),successes(idxs)};

    for i=1:N

        idxs    = find(rs{1} < (1000/N)*i & rs{1} > (1000/N)*(i-1));
        [p,c]   = binofit(sum(rs{2}(idxs)),length(idxs));
        
        p_bins(i)   = p;
        c_bins(i,:) = c;
 
    end

end
