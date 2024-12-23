function G = comm_model(xr, R_comm, nl, p)
N = size(xr,2);
dist = zeros(N,N);
diagn = zeros(N,N);
G = zeros(N,N);
G1 = zeros(N,N);
G2 = zeros(N,N);
G3 = zeros(N,N);
for i = 1:N
    dist(i,:) = dist_vect(xr,xr(:,i));
    diagn(i,i) = 1;
    G1(i,:) = double(dist(i,:) < R_comm) - diagn(i,:);
    id1 = find(G1(i,:));
    nl_pseudo = min(length(id1),nl);
    id2 = randperm(length(id1),nl_pseudo);
    id3 = id1(id2);
    G2(i,id3) = ones(1,length(id2));
    A = double(rand(1,N) > p);
    G3(i,:) = G2(i,:).*A;
    G(i,:) = G3(i,:) + diagn(i,:);
end

end

