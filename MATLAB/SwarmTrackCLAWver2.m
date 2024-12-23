function [v_ctrl,w_ctrl, search_flg_nxt, s_nxt, state_s_nxt] = SwarmTrackCLAWver2(dT_safe, dn_safe, kvTf_i, kvNs_i, kvRT_i, kw_i, x_curr, phi_curr, RM_curr, x_nrst_curr, det_Tgt_i, det_Tgt_j, neigh_ID, xhat_Tgt_i, search_flag, s, kvvTf_i, vhat_Tgt_i, dT_mon, state_s)
S_max = 50; 

phi_curr = wrapToPi(phi_curr);

delta_x_T = xhat_Tgt_i - x_curr;

delta_x_N = x_nrst_curr - x_curr;

sprl_rad_rate = 1.0;
%% Velocity Reference Control Signal
if isempty(neigh_ID) % neighbors = 0
    if det_Tgt_i == 0 % target undetected
        % Do a spiral search
        if search_flag == 0
           s = 0; 
        end
        
        delDT = sprl_rad_rate*s;
        delta_x_T = xhat_Tgt_i - x_curr + vhat_Tgt_i*0.1*s;
        vbar_i = kvTf_i*RM_curr'*(delta_x_T/norm(delta_x_T))*(norm(delta_x_T)-(delDT));
        
        vbar_rot3d = kvRT_i*cross([0 0 1]',[-delta_x_T' 0]');
        vbar_rot = RM_curr'*vbar_rot3d(1:2,1);
        
        search_flg_nxt = 1;
        
        if  state_s == 0 % growth
            if s >= S_max % switch from growth to decay
                s_del = -1;
                state_s_nxt = 1;
            else % stay at decay
                s_del = 1; 
                state_s_nxt = 0;
            end
        else % decay
            if s > 0 % stay at decay 
                s_del = -1;
                state_s_nxt = 1;
            else
                s_del = 1; % switch from decay to growth
                state_s_nxt = 0;
            end
        end
        s_nxt = s+s_del;
        
    else % target detected
        % Chase the target
        if norm(delta_x_T) < dT_mon && norm(delta_x_T) >= dT_safe
             vbar_i = kvvTf_i*RM_curr'*vhat_Tgt_i;
        else
             vbar_i = kvTf_i*RM_curr'*(delta_x_T/norm(delta_x_T))*(norm(delta_x_T)-dT_safe);   
        end
        vbar_rot = zeros(length(vbar_i),1);
        
        search_flg_nxt = 0;
        state_s_nxt = 0;
        s_nxt = s;
    end
else % neighbors > 0
    if det_Tgt_i == 0 % target undetected
        if sum(det_Tgt_j) == 0 % target undetected by neighbors
            % Do a spiral search
            if search_flag == 0
                s = 0; 
            end
            
            delDT = sprl_rad_rate*s;
            delta_x_T = xhat_Tgt_i - x_curr + vhat_Tgt_i*0.1*s;
            vbar_i = kvTf_i*RM_curr'*(delta_x_T/norm(delta_x_T))*(norm(delta_x_T)-(delDT));
            
            vbar_rot3d = kvRT_i*cross([0 0 1]',[-delta_x_T' 0]');
            vbar_rot = RM_curr'*vbar_rot3d(1:2,1);
        
            search_flg_nxt = 1;
            
            if  state_s == 0 % growth
                if s >= S_max % switch from growth to decay
                    s_del = -1;
                    state_s_nxt = 1;
                else % stay at decay
                    s_del = 1; 
                    state_s_nxt = 0;
                end
            else % decay
                if s > 0 % stay at decay 
                    s_del = -1;
                    state_s_nxt = 1;
                else
                    s_del = 1; % switch from decay to growth
                    state_s_nxt = 0;
                end
            end
            s_nxt = s+s_del;
        
        else % target detected by neighbors 
            % Chase the target
            if norm(delta_x_T) < dT_mon && norm(delta_x_T) >= dT_safe
                vbar_i = kvvTf_i*RM_curr'*vhat_Tgt_i;
            else
                vbar_i = kvTf_i*RM_curr'*(delta_x_T/norm(delta_x_T))*(norm(delta_x_T)-dT_safe);   
            end
            vbar_rot = zeros(length(vbar_i),1);
        
            search_flg_nxt = 0;
            state_s_nxt = 0;
            s_nxt = s;
        end
    else % target detected
        % Chase the target
        if norm(delta_x_T) < dT_mon && norm(delta_x_T) >= dT_safe
            vbar_i = kvvTf_i*RM_curr'*vhat_Tgt_i;
        else
             vbar_i = kvTf_i*RM_curr'*(delta_x_T/norm(delta_x_T))*(norm(delta_x_T)-dT_safe);   
        end
        vbar_rot = zeros(length(vbar_i),1);
        
        search_flg_nxt = 0;
        state_s_nxt = 0;
        s_nxt = s;
    end 
end

%% Velocity Correction Control -- Obstacle Avoidance
if norm(delta_x_N) <= dn_safe
    delta_v_N = -kvNs_i*RM_curr'*(delta_x_N/(norm(delta_x_N))^2)*(1 + (dn_safe - norm(delta_x_N))*5);
else
    delta_v_N = -kvNs_i*RM_curr'*(delta_x_N/(norm(delta_x_N))^2);
end
delta_v_N = min(max(delta_v_N,-5),5);

%% Yaw control 
headTar_onestep = atan2(delta_x_T(2),delta_x_T(1));
headTg_eff = 1.0*headTar_onestep;

if search_flg_nxt == 0
    delta_phi_target = wrapToPi(headTg_eff - phi_curr);
else
    delta_phi_target = 60*pi/180;
end

%% Final Control Signal
v_ctrl = min(max(vbar_i + delta_v_N + vbar_rot,-10),10);
w_ctrl = min(max(kw_i*delta_phi_target,-60*pi/180),60*pi/180);

end

