function [uvec] = unit_vec(vec)
%
if norm(vec) > 0
   uvec = vec/norm(vec); 
else
   uvec = zeros(size(vec)); 
end
end

