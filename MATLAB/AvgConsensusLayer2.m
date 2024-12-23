function [Dxhat_Tgt_i] = AvgConsensusLayer2(i,det_Tgt_i, det_Tgt_j, DxhatI_Tgt_i, DxhatI_Tgt_j, neigh_ID_j, DxhatS_NCOM_j, DxhatS_NCOM_ID_j, Neigh_detect_conf_NCOM_j)
%
C_D_i = [];
Dx_ji = [];
detect_ji = [];

k = 1;
ID_place = zeros(1,length(neigh_ID_j));
for j = neigh_ID_j
  [Logi,mem_ID] = ismember(i,DxhatS_NCOM_ID_j{k}); 
    if Logi
        C_D_i = [C_D_i j];
        Dx_ji = [Dx_ji DxhatS_NCOM_j{k}(:,mem_ID)]; 
        detect_ji = [detect_ji Neigh_detect_conf_NCOM_j{k}(mem_ID)];
        ID_place(k) = 1;
    end
    k = k+1;
end
ID_plac_lg = logical(ID_place);

if isempty(det_Tgt_j) 
    det_Tgt_j_proxy = 0;
    DxhatI_Tgt_j_proxy = zeros(length(DxhatI_Tgt_i),1);
else
    det_Tgt_j_proxy = det_Tgt_j(ID_plac_lg);
    DxhatI_Tgt_j_proxy = DxhatI_Tgt_j(:,ID_plac_lg); 
end

if isempty(C_D_i)
   Dxhat_Tgt_i = DxhatI_Tgt_i;
else
%     det_Tgt_i 
%     detect_ji
%     det_Tgt_j_proxy
%     sum(detect_ji.*det_Tgt_j_proxy)
   if det_Tgt_i + sum(detect_ji.*det_Tgt_j_proxy) > 0 
        Dxhat_Tgt_i = (det_Tgt_i.*DxhatI_Tgt_i + sum(detect_ji.*det_Tgt_j_proxy.*(DxhatI_Tgt_j_proxy - Dx_ji),2))./(det_Tgt_i + sum(detect_ji.*det_Tgt_j_proxy));   
   else
        Dxhat_Tgt_i = DxhatI_Tgt_i;
   end
end

end

