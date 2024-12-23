function [xhatI_Tgt_i, CovMI_i] = KCFSN_Layer1(xhatS_Tgt_i, xhat_Tgt_i_prev, det_Tgt_i, CovMdiagS_i, CovM_i_prev)
% KCF layer 1
      
   covM_S = diag([CovMdiagS_i CovMdiagS_i]); 
   covM_prev = CovM_i_prev;
   
   if det_Tgt_i > 0
   fsum = covM_S\xhatS_Tgt_i + covM_prev\xhat_Tgt_i_prev;
   CovMI_sum = (covM_S^-1 + covM_prev^-1);
   CovMI_i = ((1/2)*(covM_S^-1 + covM_prev^-1))^-1;
   else
   fsum = covM_prev\xhat_Tgt_i_prev;
   CovMI_i = covM_prev;   
   CovMI_sum = covM_prev^-1;
   end
   
   xhatI_Tgt_i = CovMI_sum\fsum; 
end