function [ID_srt,dist_srt] = NrstNeighComm(rbt_ID, xr_all)
% xr_all = permute(xr(:,t,:),[1 3 2])

xr_curr = xr_all(:,rbt_ID);
dist_all = sqrt(sum((xr_curr - xr_all).^2,1));

[dist_srt,ID_srt] = sort(dist_all);

end

