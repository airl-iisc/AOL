function [xhatI_Tgt_i] = AvgCnsnsInnov_FusionLayer1(xhatS_Tgt_i, xhat_Tgt_i_prev, det_Tgt_i, det_Tgt_neigh)
% ACF layer 1

det_i_lg = double(det_Tgt_i > 0);

if det_Tgt_i > 0 % tgt detected
    alpha_i = 0.5;
else % tgt undetected 
    alpha_i = 0;
end

xhatI_Tgt_i = alpha_i.*xhatS_Tgt_i + (1-alpha_i).*xhat_Tgt_i_prev;

end

