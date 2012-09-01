VOpred = zeros(size(VO));
VOpm   = zeros(size(VO));
VOm    = zeros(size(VO));
max_sd = log10(1+150);
for i=1:size(MG,2)
    %VOpred(i)   = FProp(oforward,MO(:,i));
    VOpm(i)     = VOpred(i);
    VOm(i)      = VO(1,i);
    VOpm(i)     = 10^(VOpm(i));
    VOm(i)      = 10^(VOm(i));
end

