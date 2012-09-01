st=struct;
st.a=1;
st.b=2;

st2={};

parfor k=1:10
    st2{k}=st;
    st2{k}.c=rand;
end

st2
