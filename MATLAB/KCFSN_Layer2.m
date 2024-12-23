function [xhat_Tgt_i, cov_M] = KCFSN_Layer2(xhatI_Tgt_i, xhatI_Tgt_j, CovMI_i, CovMI_j)
% KCF layer 2

xhatI_Tgt_ij = [xhatI_Tgt_j xhatI_Tgt_i];
Nbar = size(xhatI_Tgt_ij,2);

CovMI_ij = zeros(2,2,Nbar);

for k = 1:Nbar-1
    CovMI_ij(:,:,k) = CovMI_j(:,:,k);
end
CovMI_ij(:,:,Nbar) = CovMI_i;
   
   covM_sum = zeros(2,2);
   fsum = zeros(2,1);
   for k = 1:Nbar
      covM_sum = covM_sum + CovMI_ij(:,:,k)^-1; 
      
      fsum = CovMI_ij(:,:,k)\xhatI_Tgt_ij(:,k) + fsum;
   end
  
   xhat_Tgt_i = covM_sum\fsum;
   cov_M = Nbar*covM_sum^-1;

end