function strct = MakeStructFromObject(obj)

    strct = struct;
    
    plist = properties(obj);
    for i=1:length(plist)
        
        strct.(plist{i}) = obj.(plist{i});
        
    end
    
end