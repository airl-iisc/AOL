function [alpha_hat_i_nxt, alphapr_hat_i_nxt, what_i_nxt] = AOL_LearningPhase(t,Tp,eta_alp, eta_w, det_Tgt_i, xhatI_Tgt_j, xhatI_Tgt_i, xhatS_Tgt_i, xhat_Tgt_i_prev, xhat_Tgt_j_prev, what_j, what_i, alpha_hat_i, alphapr_hat_i, alpha_i, w_ii, del_alpha_i, del_w_ii, del_detTgt_i, p_mag, e_a1, e_a2, e_w1, e_w2)
% AOL-C

whatij = [what_j what_i];
xhatI_Tgt_ij = [xhatI_Tgt_j xhatI_Tgt_i];
xhat_Tgt_ij = [xhat_Tgt_j_prev xhat_Tgt_i_prev];
[wmax,IDstar] = max(whatij);

xhat_star = xhatI_Tgt_ij(:,IDstar);

if ceil(t/Tp) == floor(t/Tp)
    what_i = 1;
    alpha_hat_i = 1;
    alphapr_hat_i = 1;
end

alpha_hat_i_nxt = alpha_hat_i*exp(-eta_alp*min(norm(xhatS_Tgt_i - xhat_star)/15,1));
alphapr_hat_i_nxt = alphapr_hat_i*exp(-eta_alp*min(norm(xhat_Tgt_i_prev - xhat_star)/15,1));
det_Tgt_i_lgcl = det_Tgt_i;
what_i_nxt = what_i*exp(-eta_w*(1-det_Tgt_i_lgcl)); 
end

