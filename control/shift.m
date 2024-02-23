function[t0, x0, u0] = shift(T, t0, x0, u, f)
st = x0;
con = u(:,1);

f_val = f(st,con);

st = st + (T*f_val);

x0 = full(st);

t0 = t0 + T;

u0 = [u(:,2:size(u,2))';u(:,size(u,2))']';

end
