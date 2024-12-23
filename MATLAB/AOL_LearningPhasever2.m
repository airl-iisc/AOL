function [alpha_hat_i_nxt, alphapr_hat_i_nxt, what_i_nxt] = AOL_LearningPhasever2(t,Tp,eta_alp, eta_w, det_Tgt_i, xhatI_Tgt_j, xhatI_Tgt_i, xhatS_Tgt_i, xhat_Tgt_i_prev, xhat_Tgt_j_prev, what_j, what_i, alpha_hat_i, alphapr_hat_i, alpha_i, w_ii, del_alpha_i, del_w_ii, del_detTgt_i, p_mag, e_a1, e_a2, e_w1, e_w2)

if t == 1
    e_pa1 = randi([0 1])*p_mag;
    e_pa2 = p_mag - e_pa1;
else
    e_pa1 = 0;
    e_pa2 = 0;
end

if ceil(t/Tp) == floor(t/Tp)
    what_i = 1;
    alpha_hat_i = 1;
    alphapr_hat_i = 1;
    e_pa1 = randi([0 1])*p_mag;
    e_pa2 = p_mag - e_pa1;
end

det_Tgt_i_lgcl = det_Tgt_i;
what_i_nxt = what_i*exp(-eta_w*(1-det_Tgt_i_lgcl)); 

alpha_hat_i_nxt = alpha_hat_i*exp(e_a1*del_alpha_i*del_detTgt_i + e_pa1*(1-det_Tgt_i) + e_a2*alpha_i*det_Tgt_i);
alphapr_hat_i_nxt = alphapr_hat_i*exp(-e_a1*del_alpha_i*del_detTgt_i + e_pa2*(1-det_Tgt_i) + e_a2*(1-alpha_i)*det_Tgt_i);

end

