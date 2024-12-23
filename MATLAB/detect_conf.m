function b = detect_conf(r,r0,r_max,b0)
% b is the detection confidence measure
% r is the relative distance from origin
% r0 is the value of r till which slow linear decay exists 
% the value of b at r0 is 1-b0

if r <= r0
   b = 1 - (r/r0)*b0;     
elseif r > r0 && r <= r_max 
   b = (1-b0)*exp((r0-r)*3/(r_max-r0));     
else
   b = 0; 
end

end

