function [v_ctrl,w_ctrl] = SwarmCLAW(dT_safe, dn_safe, kvTf_i, kvNs_i, kw_i, x_curr, phi_curr, RM_curr, x_nrst_curr, det_Tgt, det_Tgt_neigh, neigh_ID, Tgt_lst_seen_time, Tgt_lst_seen_pos, Tgt_lst_seen_time_neigh, Tgt_lst_seen_pos_neigh, xr_neigh)

%phi_tg_nxt_est = wrapToPi(phi_tg_nxt_est);
phi_curr = wrapToPi(phi_curr);

%delta_x_T = x_tg_curr - x_curr;
%delta_x_T_nxt = x_tg_nxt_est - x_curr;

%delta_x_T_nxt_onestep = x_tg_nxt_est_onestep - x_curr;
delta_x_N = x_nrst_curr - x_curr;

%% Velocity Reference Control Signal
if det_Tgt == 1 % target detected 
   x_tg_curr = Tgt_lst_seen_pos; 
   delta_x_T = x_tg_curr - x_curr; 
   vbar_i = kvTf_i*RM_curr'*(delta_x_T/norm(delta_x_T))*(norm(delta_x_T)-dT_safe);
else % target not detected 
   if isempty(neigh_ID) % if no neighbours around
     rnd_ang = rand*(2*pi);
     delta_x_T = [cos(rnd_ang); sin(rnd_ang)];  
     vbar_i = kvTf_i*RM_curr'*(delta_x_T/norm(delta_x_T))*5;      
   else % neighbours are around
     if isempty(find(det_Tgt_neigh, 1)) % none of the neighbours have detected the target
        [~,t_max_neigh_ord] = max([Tgt_lst_seen_time_neigh Tgt_lst_seen_time]);
        if isempty(t_max_neigh_ord) % last seen target history is empty for all neighbours and the robot itself
             delta_x_T = (mean(xr_neigh,2) - x_curr); 
             vbar_i = -kvTf_i*RM_curr'*(delta_x_T/norm(delta_x_T))*(norm(delta_x_T))^-1;
        elseif t_max_neigh_ord <= length(Tgt_lst_seen_time_neigh) % one of the neighbours have the latest target history
             tlspn = (Tgt_lst_seen_pos_neigh);
             x_tg_curr = tlspn(:,t_max_neigh_ord);
             delta_x_T = x_tg_curr - x_curr; 
             vbar_i = kvTf_i*RM_curr'*(delta_x_T/norm(delta_x_T));
        elseif t_max_neigh_ord > length(Tgt_lst_seen_time_neigh) % robot itself has the latest target history
             x_tg_curr = Tgt_lst_seen_pos;
             delta_x_T = x_tg_curr - x_curr; 
             vbar_i = kvTf_i*RM_curr'*(delta_x_T/norm(delta_x_T)); 
        end
    else % at least one neighbour has detected the target
        order_detect = logical(det_Tgt_neigh);
        ord_ID = det_Tgt_neigh(order_detect);
        tlspn = (Tgt_lst_seen_pos_neigh);
        x_tg_curr = mean(tlspn(:,ord_ID),2);
        delta_x_T = x_tg_curr - x_curr; 
        vbar_i = kvTf_i*RM_curr'*(delta_x_T/norm(delta_x_T))*(norm(delta_x_T)-dT_safe);
    end
   end
end

%% Velocity Correction Control -- Obstacle Avoidance
if norm(delta_x_N) <= dn_safe
    delta_v_N = -kvNs_i*RM_curr'*(delta_x_N/(norm(delta_x_N))^2)*(1 + (dn_safe - norm(delta_x_N))*5);
else
    delta_v_N = -kvNs_i*RM_curr'*(delta_x_N/(norm(delta_x_N))^2);
end

%% Yaw control 
headTar_onestep = atan2(delta_x_T(2),delta_x_T(1));

headTg_eff = 1.0*headTar_onestep;

delta_phi_target = wrapToPi(headTg_eff - phi_curr);

%% Final Control Signal
v_ctrl = min(max(vbar_i + delta_v_N,-10),10);
w_ctrl = min(max(kw_i*delta_phi_target,-30*pi/180),30*pi/180);

end

