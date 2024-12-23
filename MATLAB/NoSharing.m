function [xhat_Tgt_i] = NoSharing(t, det_Tgt_i, xhatS_Tgt_i, xhat_Tgt_i_prev)
%
        if det_Tgt_i > 0 % target detected
                xhat_Tgt_i = xhatS_Tgt_i;               
       else % target undetected 
                if t > 1
                    xhat_Tgt_i = xhat_Tgt_i_prev;
                end
       end
end

