function [xhat] = unitvect(x)
if norm(x) == 0
    xhat = 0*ones(size(x));
else
    xhat = x./norm(x);
end

