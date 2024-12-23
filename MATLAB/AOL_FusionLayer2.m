function [xhat_Tgt_i, wii] = AOL_FusionLayer2(xhatI_Tgt_i, xhatI_Tgt_j, what_i, what_j)
%

wii = what_i./(what_i + sum(what_j));
wij = what_j./(what_i + sum(what_j));

xhat_Tgt_i = wii.*xhatI_Tgt_i + sum(wij.*xhatI_Tgt_j,2);

end

