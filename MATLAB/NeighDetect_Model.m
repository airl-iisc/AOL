function [G2] = NeighDetect_Model(xr,phir,Rfov_Lidar,thetaFOV_Lidar,p)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
r0 = Rfov_Lidar*(2/3);
b0 = 0.1;

N = size(xr,2);
dist = zeros(N,N);
diagn = zeros(N,N);
G1 = zeros(N,N);
G2 = zeros(N,N);
for i = 1:N
    dist(i,:) = dist_vect(xr,xr(:,i));
    diagn(i,i) = 1;
    G1(i,:) = double(dist(i,:) <= Rfov_Lidar) - diagn(i,:);
    id1 = find(G1(i,:));
    phi_vec = [cos(phir(:,i)) sin(phir(:,i))]'; 
    for j = id1
        if dot(xr(:,j)-xr(:,i),phi_vec)/norm(xr(:,j) - xr(:,i)) >= cos(thetaFOV_Lidar/2)
            G1(i,j) = detect_conf(dist(i,j),r0,Rfov_Lidar,b0);
        else
            G1(i,j) = 0;
        end
    end
    A = double(rand(1,N) > p);
    G2(i,:) = G1(i,:).*A;
end

end

