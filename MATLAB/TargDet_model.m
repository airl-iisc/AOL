function [detect_Tgt] = TargDet_model(xr,phir,xTg,R_FOV,theta_FOV,p)
r0 = R_FOV*(2/3);
b0 = 0.1;

N = size(xr,2);
NTg = size(xTg,2);
detect_flg = zeros(NTg,N);
dist = zeros(NTg,N);
for j = 1:NTg
    dist(j,:) = dist_vect(xTg(:,j),xr);
for i = 1:N
   phi_vec = [cos(phir(:,i)) sin(phir(:,i))]'; 
   if norm(xTg(:,j) - xr(:,i)) <= R_FOV && dot(xTg(:,j)-xr(:,i),phi_vec)/norm(xTg(:,j) - xr(:,i)) >= cos(theta_FOV/2) 
       detect_flg(j,i) = detect_conf(dist(j,i),r0,R_FOV,b0);
   else
       detect_flg(j,i) = 0;
   end
end
end

A = double(rand(NTg,N) > p);
detect_Tgt = detect_flg.*A;
end

