function u=mysat2(x,min,max)
% ensure the direction to be the same
if norm(x)>max
    u =max*x/norm(x);
elseif norm(x)<min
    u =min*x/norm(x);
else
    u =x ;
end
