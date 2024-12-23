function dist = dist_vect(vec1,vec2)
disp = vec1 - vec2;
dist = sqrt(sum(disp.^2));
end

