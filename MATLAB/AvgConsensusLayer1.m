function [DxhatI_Tgt_i] = AvgConsensusLayer1(DxhatS_Tgt_i,Dxhat_Tgt_i_prev)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

alpha_i = 0.5;
DxhatI_Tgt_i = alpha_i*DxhatS_Tgt_i + (1-alpha_i)*Dxhat_Tgt_i_prev;

end

