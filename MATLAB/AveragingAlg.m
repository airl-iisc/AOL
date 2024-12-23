function [xhat_Tgt_i] = AveragingAlg(k_A,det_Tgt_i,det_Tgt_j,xhatS_Tgt_i,xhatS_Tgt_j, xhat_Tgt_i_prev)
%

if isempty(det_Tgt_j) 
    det_Tgt_j_proxy = 0;
    xhatS_Tgt_j_proxy = zeros(length(xhat_Tgt_i_prev),1);
else
    det_Tgt_j_proxy = det_Tgt_j;
    xhatS_Tgt_j_proxy = xhatS_Tgt_j; 
end
    
xhat_Tgt_i = (det_Tgt_i.*xhatS_Tgt_i + sum(det_Tgt_j_proxy.*xhatS_Tgt_j_proxy,2) + k_A.*xhat_Tgt_i_prev)./(det_Tgt_i + sum(det_Tgt_j_proxy) + k_A);

end

