function [v_ctrl,w_ctrl, search_flg_nxt, s_nxt, state_s_nxt, dir_move, bound_hit] = SwarmTrackCLAW(t, dT_safe, dn_safe, kvTf_i, kvNs_i, kvRT_i, kw_i, x_curr, phi_curr, RM_curr, x_nrst_curr, det_Tgt_i, det_Tgt_j, neigh_ID, xhat_Tgt_i, search_flag, s, kvvTf_i, vhat_Tgt_i, dT_mon, state_s, x_curr_OD, phi_curr_OD, theta_FOV, dir_move_prev, box_eff, bound_hit_prev)
S_max = 80; 

phi_curr = wrapToPi(phi_curr);

delta_x_T = xhat_Tgt_i - x_curr_OD;

delta_x_N = x_nrst_curr - x_curr;

sprl_rad_rate = 2.0;

%% Velocity Reference Control Signal
if isempty(neigh_ID) % neighbors = 0
    if det_Tgt_i == 0 % target undetected
        % Do a spiral search
        if  state_s == 0 % growth in same direction
            if s >= S_max % switch direction
                s_del = 0;
                state_s_nxt = 1;
            else % stay in same direction
                s_del = 1; 
                state_s_nxt = 0;
            end
        else % set new direction
            s_del = 0;
            s = 0;
            state_s_nxt = 0;
        end
        
        if search_flag == 0
           s = 0; 
           headTar = atan2(delta_x_T(2),delta_x_T(1));
           if t > 0
            dir_move = [cos(headTar); sin(headTar)];
           else
            phi_rand0 = rand*2*pi;  
            dir_move = [cos(phi_rand0); sin(phi_rand0)];   
           end
        else
            if state_s_nxt == 1
                phi_rand = (rand-0.5)*2*pi; 
                dir_move =  [cos(phi_curr_OD + phi_rand); sin(phi_curr_OD + phi_rand)];
            else
                dir_move =  dir_move_prev;
            end
        end
        
        delDT = sprl_rad_rate*s;
        delta_x_T = dir_move;
        vbar_i = kvTf_i*RM_curr'*delta_x_T*10;
        vbar_i(2) = min(vbar_i(2) + s*sin(delDT*2*pi/(S_max)), 10);
        
        vbar_rot3d = kvRT_i*cross([0 0 1]',[-delta_x_T' 0]');
        vbar_rot = RM_curr'*vbar_rot3d(1:2,1);
        
        search_flg_nxt = 1;
        
        s_nxt = s+s_del;
        
    else % target detected
        % Chase the target
        if norm(delta_x_T) < dT_mon && norm(delta_x_T) >= dT_safe
             vbar_i = kvvTf_i*RM_curr'*vhat_Tgt_i;
        else
             vbar_i = kvTf_i*RM_curr'*(delta_x_T/norm(delta_x_T))*(norm(delta_x_T)-dT_safe);   
        end
        vbar_rot = zeros(length(vbar_i),1);
        dir_move =  dir_move_prev;
        
        search_flg_nxt = 0;
        state_s_nxt = 0;
        s_nxt = s;
    end
else % neighbors > 0
    if det_Tgt_i == 0 % target undetected
        if sum(det_Tgt_j) == 0 % target undetected by neighbors
            % Do a spiral search
         if  state_s == 0 % growth in same direction
            if s >= S_max % switch direction
                s_del = 0;
                state_s_nxt = 1;
            else % stay at growth
                s_del = 1; 
                state_s_nxt = 0;
            end
        else % set new direction
            s_del = 0;
            s = 0;
            state_s_nxt = 0;
        end
        
        if search_flag == 0
           s = 0; 
           headTar = atan2(delta_x_T(2),delta_x_T(1));
           if t > 0
            dir_move = [cos(headTar); sin(headTar)];
           else
            phi_rand0 = rand*2*pi;  
            dir_move = [cos(phi_rand0); sin(phi_rand0)];   
           end
        else
            if state_s_nxt == 1
                phi_rand = (rand-0.5)*2*pi; 
                dir_move =  [cos(phi_curr_OD + phi_rand); sin(phi_curr_OD + phi_rand)];
            else
                dir_move =  dir_move_prev;
            end
        end
        
        delDT = sprl_rad_rate*s;
        delta_x_T = dir_move;
        vbar_i = kvTf_i*RM_curr'*delta_x_T*10;
        vbar_i(2) = min(vbar_i(2) + s*sin(delDT*2*pi/(S_max)), 10);
        
        vbar_rot3d = kvRT_i*cross([0 0 1]',[-delta_x_T' 0]');
        vbar_rot = RM_curr'*vbar_rot3d(1:2,1);
        
        search_flg_nxt = 1;
        
        s_nxt = s+s_del;
            
       else % target detected by neighbors 
            % Chase the target
            if norm(delta_x_T) < dT_mon && norm(delta_x_T) >= dT_safe
                vbar_i = kvvTf_i*RM_curr'*vhat_Tgt_i;
            else
                vbar_i = kvTf_i*RM_curr'*(delta_x_T/norm(delta_x_T))*(norm(delta_x_T)-dT_safe);   
            end
            vbar_rot = zeros(length(vbar_i),1);
            dir_move =  dir_move_prev;
        
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
        dir_move =  dir_move_prev;
        
        search_flg_nxt = 0;
        state_s_nxt = 0;
        s_nxt = s;
    end 
end

%% Velocity Correction Control -- Obstacle Avoidance
if norm(delta_x_N) <= dn_safe
    delta_v_N = -kvNs_i*RM_curr'*(delta_x_N/(norm(delta_x_N))^2)*(1 + (dn_safe - norm(delta_x_N))*10);
else
    delta_v_N = -kvNs_i*RM_curr'*(delta_x_N/(norm(delta_x_N))^2);
end
delta_v_N = min(max(delta_v_N,-10),10);

%% Yaw control 

if search_flg_nxt == 0
    headTar_onestep = atan2(delta_x_T(2),delta_x_T(1));
    headTg_eff = 1.0*headTar_onestep;
    delta_phi_target = wrapToPi(headTg_eff - phi_curr_OD);
else
    headTar_onestep = atan2(dir_move(2),dir_move(1));
    headTg_eff = 1.0*headTar_onestep;
    delta_phi_target = wrapToPi(headTg_eff - phi_curr_OD);
end

%% Boundary Check

if norm(x_curr_OD,inf) >= box_eff
    vbar_i = 10*RM_curr'*[double(min(x_curr_OD(1)+box_eff,0)<0)-double(max(x_curr_OD(1)-box_eff,0)>0); double(min(x_curr_OD(2)+box_eff,0)<0)-double(max(x_curr_OD(2)-box_eff,0)>0)];
    bound_hit = 1;
else
    if bound_hit_prev > 0
        if norm(x_curr_OD,inf) >= box_eff-10
            vbar_i = 10*RM_curr'*[double(min(x_curr_OD(1)+box_eff-10,0)<0)-double(max(x_curr_OD(1)-box_eff+10,0)>0); double(min(x_curr_OD(2)+box_eff-10,0)<0)-double(max(x_curr_OD(2)-box_eff+10,0)>0)];
            bound_hit = 1;
        else
            bound_hit = 0;
        end
    else
        bound_hit = 0;
    end
end

%% Final Control Signal
vbar_i = min(max(vbar_i,-10),10);

v_ctrl = min(max(vbar_i + delta_v_N + vbar_rot.*0,-10),10);
w_ctrl = min(max(kw_i*delta_phi_target,-60*pi/180),60*pi/180);

end

