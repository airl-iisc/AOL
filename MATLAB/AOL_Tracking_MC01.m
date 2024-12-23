%% Master Code for comparison study between the three AOL variants, no comm. case, ACF, and KCF; 
%  by Shubhankar Gupta

clc
clear
%% Preliminary Parameters and variables

T = 600; % discrete-time steps
N = 30; % total no. of robots
NTg = 1; % no. of targets
deltaT = 0.1; % sampling rate
NRuns = 100; % number of simulation runs for each algorithm/method

casenum = [1 3 4 0 2 5]; % 1 = AOL-C, 3 = AOL-1P, 4 = AOL-2P, 0 = No Comm., 2 = ACF, 5 = KCF 
case_k = 1; 

cml_det_scr_avg = zeros(length(casenum),NRuns);
cml_clsns_scr_avg = zeros(length(casenum),NRuns);
for caseNo = casenum

cumul_det_score_i = zeros(NRuns,N);
cumul_det_score_tot_avg = zeros(1,NRuns);
cumul_closeness_score_i = zeros(NRuns,N);
cumul_closeness_score_tot_avg = zeros(1,NRuns);
for n = 1:NRuns

rng(n*9)  % random number generator   
    
xr = zeros(2,T,N); % position vectors of N robots over T time-steps
phir = zeros(1,T,N); % heading angle of N robots over T time-steps

RotM = @(phi) [cos(phi) -sin(phi); sin(phi) cos(phi)]; % robot body-axis to global coordinate axis

%% Target Dynamics

box_l = 100;
delBox = 10;
box_eff = box_l - delBox;
k_box = 10.0;

xTg = zeros(2,T); % position vector of the Target over T time-steps
phiTg = zeros(1,T);

wbar_Tg = zeros(1,T);
vbar_Tg = zeros(2,T); 
delta_v = zeros(2,T);
Tc = 50;

rand01_array = [0 1];
rand11_ID = randperm(2,1);
rand12_ID = randperm(2,1);
rand21_ID = randperm(2,1);
rand22_ID = randperm(2,1);
rand11 = rand01_array(rand11_ID);
rand12 = rand01_array(rand12_ID);
rand21 = rand01_array(rand21_ID);
rand22 = rand01_array(rand22_ID);

al = 30;
a2 = 35;
bl = a2-al;

xTg(:,1) = [rand11*(rand*20) + (1-rand11)*(-rand*20); 1*(al+rand*bl) + (0)*(-al-rand*bl)];
phiTg(:,1) = pi*(rand-0.5)*2;
bound_hit_Tgt_prev = 0;
for t = 1:T
    
    if t <= 1
        wbar_Tg(1,t) = randi([-1 1])*(6*pi/(T*deltaT) + rand*6*pi/(T*deltaT));
        vbar_Tg(:,t) = [2 + rand*8; 2*(rand-0.5)*10];
    end
    if ceil(t/Tc) == floor(t/Tc)
        wbar_Tg(1,t) = randi([-1 1])*(6*pi/(T*deltaT) + rand*6*pi/(T*deltaT));
        vbar_Tg(:,t) = [2 + rand*8; 2*(rand-0.5)*10];
    else
        if t > 1
            wbar_Tg(1,t) = wbar_Tg(1,t-1);
            vbar_Tg(:,t) = vbar_Tg(:,t-1);
        end
    end
    vbar_Tg_F = max(min(delta_v(:,t)*0 + vbar_Tg(:,t),10),-10);
    
    if norm(xTg(:,t),inf) >= box_eff
        delta_vdir = [double(min(xTg(1,t)+box_eff,0)<0)-double(max(xTg(1,t)-box_eff,0)>0); double(min(xTg(2,t)+box_eff,0)<0)-double(max(xTg(2,t)-box_eff,0)>0)];
        vbar_Tg_F = (RotM(phiTg(:,t)))'*unit_vec(delta_vdir)*k_box;
        bound_hit_Tgt = 1;
    else
        if bound_hit_Tgt_prev > 0
        if norm(xTg(:,t),inf) >= box_eff-10
            delta_vdir = [double(min(xTg(1,t)+box_eff-10,0)<0)-double(max(xTg(1,t)-box_eff+10,0)>0); double(min(xTg(2,t)+box_eff-10,0)<0)-double(max(xTg(2,t)-box_eff+10,0)>0)];
            vbar_Tg_F = (RotM(phiTg(:,t)))'*unit_vec(delta_vdir)*k_box;
            bound_hit_Tgt = 1;
        else
            bound_hit_Tgt = 0;
        end
    else
        bound_hit_Tgt = 0;
        end 
    end
    bound_hit_Tgt_prev = bound_hit_Tgt; 
   [xTg(:,t+1),phiTg(:,t+1)] = targetctrldyn(deltaT,xTg(:,t),phiTg(:,t),RotM(phiTg(:,t)),vbar_Tg_F,wbar_Tg(:,t));
end

%% initial conditions for the robots
box_rbt = 20;
box_rbt_del = 10;

nx = 1;
ny = 1;
for i = 1:N
   xr(:,1,i) = [-box_rbt - box_rbt_del + nx*box_rbt_del; box_rbt + box_rbt_del - ny*10];
   phir(:,1,i) = pi/2;
   if nx >= floor(2*box_rbt/box_rbt_del) + 1
       ny = ny + 1; 
       nx = 0;
   end
   nx = nx + 1;
end

%% Parameter set 

R_FOV = 15;
R_comm = 30;
nl = 3;
p_ld = 0.1;
theta_FOV = 160*(pi/180); 
p_vl = 0.1;

Tp = 15;
eta_alp = 0.01;
eta_w = 15;

dn_safe = 5*ones(1,N);
dTgt_safe = 8;
dT_safe = dTgt_safe*ones(1,N);
dT_mon = 11*ones(1,N);

e_a1 = 10; % 20
e_a2 = 0.1;  % 1, 5
% 
e_a1_3 = 20; % 20
e_a2_3 = 5;  % 1, 5
e_w1 = 1;  
e_w2 = 0.01;
% 
p_mag1 = 0.1; % 1
p_mag2 = 0.1;
p_mag3 = 0.1;

kvTf = 6*ones(1,N);
kvvTf = 1*ones(1,N);
kvNs = 12*ones(1,N);
kvRT = 2*ones(1,N);
kw = 15*ones(1,N);

%% Noise in the estimates of target's position - faults in exteroception 
fault_exts = 1; % 1 = faulty exteroception, 0 = no faults in exteroception

kf_ext = 0.5; % ratio of no. of robots that have faulty exterocpetion
Nf_ext = floor(N*kf_ext); % no. of robots with faulty exteroception

nsr_mean = zeros(T,N);

if fault_exts == 1
    nsr_val = [0.01*ones(1,N-Nf_ext) 10*ones(1,Nf_ext)];
else
    nsr_val = 0.01*ones(1,N);
end

nsr_mean(1:floor(T/6),:) = repmat(nsr_val(randperm(length(nsr_val),N)),length(1:floor(T/6)),1);
nsr_mean(ceil(T/6):floor(2*T/6),:) = repmat(nsr_val(randperm(length(nsr_val),N)),length(ceil(T/6):floor(2*T/6)),1);
nsr_mean(ceil(2*T/6):floor(3*T/6),:) = repmat(nsr_val(randperm(length(nsr_val),N)),length(ceil(2*T/6):floor(3*T/6)),1);
nsr_mean(ceil(3*T/6):floor(5*T/6),:) = repmat(nsr_val(randperm(length(nsr_val),N)),length(ceil(3*T/6):floor(5*T/6)),1);
nsr_mean(ceil(5*T/6):ceil(6*T/6),:) = repmat(nsr_val(randperm(length(nsr_val),N)),length(ceil(5*T/6):ceil(6*T/6)),1);

nsr_var_val = [0.01 5];
nsr_var_val02 = [0.01 2];
nsr_var = zeros(T,N);
for t = 1:T
    for i = 1:N
        if nsr_mean(t,i) < 2
            nsr_var(t,i) = nsr_mean(t,i);
        elseif nsr_mean(t,i) < 4 && nsr_mean(t,i) >= 2  
            nsr_var(t,i) = nsr_var_val(randperm(length(nsr_var_val02),1));
        else
            nsr_var(t,i) = nsr_var_val(randperm(length(nsr_var_val),1));
        end
    end
end

%% faults in the IMU/Proprioception
fault_IMU = 1; % 1 = faulty proprioception, 0 = no faults in proprioception

kf_IMU = 0.5; % ratio of no. of robots that have faulty proprioception
Nf_IMU = floor(N*kf_IMU); % no. of robots with faulty proprioception

nsr_mean_IMU = zeros(T,N);

if fault_IMU == 1
    nsr_val_IMU = [0.01*ones(1,N-Nf_IMU) 5*ones(1,Nf_IMU)];
else
    nsr_val_IMU = 0.01*ones(1,N);
end

nsr_mean_IMU(1:floor(T/6),:) = repmat(0.01*ones(1,N),length(1:floor(T/6)),1);
nsr_mean_IMU(ceil(T/6):ceil(6*T/6),:) = repmat(nsr_val_IMU(randperm(length(nsr_val_IMU),N)),length(ceil(T/6):ceil(6*T/6)),1);

nsr_var_val_IMU = [0.01 5];
nsr_var_val02_IMU = [0.01 2];
nsr_var_IMU = zeros(T,N);
for t = 1:T
    for i = 1:N
        if nsr_mean_IMU(t,i) < 2
            nsr_var_IMU(t,i) = nsr_mean_IMU(t,i);
        elseif nsr_mean_IMU(t,i) < 4 && nsr_mean_IMU(t,i) >= 2  
            nsr_var_IMU(t,i) = nsr_var_val_IMU(randperm(length(nsr_var_val02_IMU),1));
        else
            nsr_var_IMU(t,i) = nsr_var_val_IMU(randperm(length(nsr_var_val_IMU),1));
        end
    end
end

nsr_S_var = zeros(T,N);

%% 

Gdyn = zeros(N,N,T);
Ddyn = zeros(N,N,T);
detect_Tgt = zeros(NTg,N,T);
det_Tgt_neigh = cell(1,N);
neigh_ID = cell(1,N);
xr_neigh = cell(1,N);
xhatS_Tgt_j = cell(1,N);
xhatI_Tgt_j = cell(1,N);
xhat_Tgt_j_prev = cell(1,N);
xhat_Tgt = zeros(2,N,T);
xhatS_Tgt = zeros(2,N,T);
xhatI_Tgt = zeros(2,N,T);
xhatOD_i = zeros(2,N,T);
phihatOD_i = zeros(1,N,T);

DxhatS_Tgt = zeros(2,N,T);
Dxhat_Tgt = zeros(2,N,T);
DxhatI_Tgt = zeros(2,N,T);

DxhatS_Neigh = cell(1,N);
Neigh_detect_conf = cell(1,N);
DxhatS_Neigh_ID = cell(1,N);
DxhatS_NCOM = cell(1,N);
DxhatS_NCOM_ID = cell(1,N);
DxhatI_Tgt_j = cell(1,N);
Neigh_detect_conf_NCOM = cell(1,N);
what_j = cell(1,N);
cumul_det_Tgt_j = cell(1,N);
det_Tgt_neigh_prev = cell(1,N);

vhat_Tgt = zeros(2,N,T);
dir_move_src = zeros(2,N,T);
srch_flag = zeros(T,N);
s_cnt = zeros(T,N);
state_s = zeros(T,N);
det_Tgt_prev = zeros(1,N);
bound_hit_i = zeros(T,N);
cumul_det_Tgt = zeros(T,N);
alpha_hat = ones(T,N); 
alphapr_hat = ones(T,N);
what = ones(T,N);
alpha = zeros(T,N);
wii = zeros(T,N);
del_alpha = zeros(T,N);
del_wii = zeros(T,N);
del_detTgt = zeros(T,N);
det_score = zeros(T,N);
closeness_score = zeros(T,N);

CovMI = zeros(2,2,N,T);
for i = 1:N
    CovMI(:,:,i,1) = diag([0.02 0.02]);
end

CovM = zeros(2,2,N,T);
for i = 1:N
    CovM(:,:,i,1) = diag([0.02 0.02]);
end

CovMI_j = cell(1,N);

xhat_Tgt(:,:,1) = permute(xr(:,1,:),[1 3 2]); 
xhatS_Tgt(:,:,1) = permute(xr(:,1,:),[1 3 2]); 
xhatOD_i(:,:,1) = permute(xr(:,1,:),[1 3 2]);
phihatOD_i(:,:,1) = permute(phir(:,1,:),[1 3 2]);

phi_rand = (rand(1,N)-0.5)*2*pi;
dir_move_src(:,:,1) = [cos(permute(phir(:,1,:),[1 3 2])); sin(permute(phir(:,1,:),[1 3 2]))];
DxhatS_Tgt(:,:,1) = R_FOV*([cos(permute(phir(:,1,:),[1 3 2]) + phi_rand); sin(permute(phir(:,1,:),[1 3 2])+ phi_rand)]);
xhat_Tgt(:,:,1) = xhatOD_i(:,:,1) + DxhatS_Tgt(:,:,1);
for t = 1:T-1
    
    Gdyn(:,:,t) = comm_model(permute(xr(:,t,:),[1 3 2]),R_comm,nl,p_ld); %%%%%
    Gdyn_neigh = logical(Gdyn(:,:,t) - diag(ones(1,N)));
    
    detect_Tgt(:,:,t) = TargDet_model(permute(xr(:,t,:),[1 3 2]),permute(phir(:,t,:),[1 3 2]),xTg(:,t),R_FOV,theta_FOV,p_vl);
    det_Tgt = detect_Tgt(:,:,t);
    
    det_score(t,:) = detect_Tgt(:,:,t);
    closeness_score(t,:) = min(dTgt_safe*dist_vect(permute(xr(:,t,:),[1 3 2]),xTg(:,t)).^-1,1); 
    
    det_Tgt_lg = det_Tgt > 0;
    if t > 1
        cumul_det_Tgt(t,:) = cumul_det_Tgt(t-1,:) + det_Tgt;
    else
        cumul_det_Tgt(t,:) = det_Tgt;
    end
    
    %%%%%%%%%%%%%%%Local Sensor Estimates%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    for i = 1:N
       rnd_ang1 = rand*(2*pi);
       nsr_dir1 = [cos(rnd_ang1); sin(rnd_ang1)];
       sign_val = [-1 1];
       sign_ID = randperm(length(sign_val),1);
       rnd_sign = sign_val(sign_ID);
       
       xhatOD_i(:,i,t) = xr(:,t,i) + nsr_mean_IMU(t,i)*nsr_dir1; % add bias
       phihatOD_i(:,i,t) = wrapToPi(phir(:,t,i) + rnd_sign*nsr_mean_IMU(t,i)*(2*pi/180)); % add bias
       
       if det_Tgt(i) > 0 % target detected
                rnd_ang2 = rand*(2*pi);
                nsr_dir2 = [cos(rnd_ang2); sin(rnd_ang2)];
                DxhatS_Tgt(:,i,t) = xTg(:,t) - xr(:,t,i) + nsr_mean(t,i)*nsr_dir2; % add bias
       else % target undetected 
                if t > 1
                    DxhatS_Tgt(:,i,t) = DxhatS_Tgt(:,i,t-1);
                end
       end
       xhatS_Tgt(:,i,t) = xhatOD_i(:,i,t) + DxhatS_Tgt(:,i,t);
       nsr_S_var(t,i) = nsr_var(t,i) + nsr_var_IMU(t,i);
    end
    %%%%%%%%
    
    %%%%%%%%%%%% Estimate Fusion Layer 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%
    if caseNo == 1 || caseNo == 3 || caseNo == 4
        for i = 1:N
            if t > 1
                [xhatI_Tgt(:,i,t), alpha(t,i)] = AOL_FusionLayer1(xhatS_Tgt(:,i,t), xhat_Tgt(:,i,t-1), alpha_hat(t,i), alphapr_hat(t,i), det_Tgt(i), det_Tgt_neigh_prev{i});
            else
                [xhatI_Tgt(:,i,t), alpha(t,i)] = AOL_FusionLayer1(xhatS_Tgt(:,i,t), xhat_Tgt(:,i,t), alpha_hat(t,i), alphapr_hat(t,i), det_Tgt(i), det_Tgt_neigh_prev{i});
            end
        end
        
    elseif caseNo == 2
        for i = 1:N
            if t > 1
                [xhatI_Tgt(:,i,t)] = AvgCnsnsInnov_FusionLayer1(xhatS_Tgt(:,i,t), xhat_Tgt(:,i,t-1), det_Tgt(i), det_Tgt_neigh_prev{i});
            else
                [xhatI_Tgt(:,i,t)] = AvgCnsnsInnov_FusionLayer1(xhatS_Tgt(:,i,t), xhat_Tgt(:,i,t), det_Tgt(i), det_Tgt_neigh_prev{i});
            end
        end
        
    elseif caseNo == 5
        for i = 1:N
            if t > 1
               [xhatI_Tgt(:,i,t), CovMI(:,:,i,t)] = KCFSN_Layer1(xhatS_Tgt(:,i,t), xhat_Tgt(:,i,t-1), det_Tgt(i), nsr_S_var(t,i), CovM(:,:,i,t-1));
            else
               [xhatI_Tgt(:,i,t), CovMI(:,:,i,t)] = KCFSN_Layer1(xhatS_Tgt(:,i,t), xhat_Tgt(:,i,t), det_Tgt(i), nsr_S_var(t,i), CovM(:,:,i,t));
            end
        end
    end
    %%%%%%%%
    
    %%%%%%%% Information from communicating neighbors %%%%%%%%%
    for i = 1:N
       det_Tgt_neigh{i} = det_Tgt(:,Gdyn_neigh(i,:));
       rbt_ID = 1:N;
       neigh_ID{i} = rbt_ID(Gdyn_neigh(i,:)); % comm. neighbor ID's 
       
       xr_j = permute(xr(:,t,:),[1 3 2]);
       xr_neigh{i} = xr_j(:,Gdyn_neigh(i,:));
       
       xhatS_Tgt_j{i} = xhatS_Tgt(:,Gdyn_neigh(i,:),t);
       
       xhatI_Tgt_j{i} = xhatI_Tgt(:,Gdyn_neigh(i,:),t); % comm. neighbor Tgt estimates
       
%        xhat_Tgt_j_prev{i} = xhat_Tgt(:,Gdyn_neigh(i,:),t-1);
       
       cumul_det_Tgt_j{i} = cumul_det_Tgt(t,Gdyn_neigh(i,:));
       
       if t > 1
            xhat_Tgt_j_prev{i} = xhat_Tgt(:,Gdyn_neigh(i,:),t-1);
       else
            xhat_Tgt_j_prev{i} = xhat_Tgt(:,Gdyn_neigh(i,:),t);
       end
       
       if caseNo == 1 || caseNo == 3 || caseNo == 4
            what_j{i} = what(t,Gdyn_neigh(i,:)); 
       end
       
       if caseNo == 5
           CovMI_j{i} = CovMI(:,:,Gdyn_neigh(i,:),t);
       end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%Estimate Fusion Layer 2%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if caseNo == 1 || caseNo == 3 || caseNo == 4
         for i = 1:N
             [xhat_Tgt(:,i,t)] = AOL_FusionLayer2(xhatI_Tgt(:,i,t), xhatI_Tgt_j{i}, what(t,i), what_j{i});
         end    
    elseif caseNo == 2
        for i = 1:N
             [xhat_Tgt(:,i,t)] = AvgCnsnsInnov_FusionLayer2(xhatI_Tgt(:,i,t), xhatI_Tgt_j{i});
        end 
        
    elseif caseNo == 5
        for i = 1:N
             [xhat_Tgt(:,i,t), CovM(:,:,i,t)] = KCFSN_Layer2(xhatI_Tgt(:,i,t), xhatI_Tgt_j{i}, CovMI(:,:,i,t), CovMI_j{i});
        end     
    else
         for i = 1:N
            if t > 1 
               [xhat_Tgt(:,i,t)] = NoSharing(t, det_Tgt(i), xhatS_Tgt(:,i,t), xhat_Tgt(:,i,t-1)); 
            end
         end   
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if t > 1
       del_alpha(t,:) = alpha(t,:) - alpha(t-1,:); 
       del_wii(t,:) = wii(t,:) - wii(t-1,:);
       del_detTgt(t,:) = detect_Tgt(:,:,t) - detect_Tgt(:,:,t-1);
    else
       del_alpha(t,:) = zeros(1,N); 
       del_wii(t,:) = zeros(1,N);
       del_detTgt(t,:) = zeros(1,N);
    end
    %%%%%%%%%%%%%%%%%%%%Learning Phase%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if caseNo == 1
         for i = 1:N
             if t > 1
                [alpha_hat(t+1,i), alphapr_hat(t+1,i), what(t+1,i)] = AOL_LearningPhase(t, Tp, eta_alp, eta_w, det_Tgt(i), xhatI_Tgt_j{i}, xhatI_Tgt(:,i,t), xhatS_Tgt(:,i,t), xhat_Tgt(:,i,t-1), xhat_Tgt_j_prev{i}, what_j{i}, what(t,i), alpha_hat(t,i), alphapr_hat(t,i), alpha(t,i), wii(t,i), del_alpha(t,i), del_wii(t,i), del_detTgt(t,i), p_mag1, e_a1, e_a2, e_w1, e_w2);
             else
                [alpha_hat(t+1,i), alphapr_hat(t+1,i), what(t+1,i)] = AOL_LearningPhase(t, Tp, eta_alp, eta_w, det_Tgt(i), xhatI_Tgt_j{i}, xhatI_Tgt(:,i,t), xhatS_Tgt(:,i,t), xhat_Tgt(:,i,t), xhat_Tgt_j_prev{i}, what_j{i}, what(t,i), alpha_hat(t,i), alphapr_hat(t,i), alpha(t,i), wii(t,i), del_alpha(t,i), del_wii(t,i), del_detTgt(t,i), p_mag1, e_a1, e_a2, e_w1, e_w2);
             end
         end
    elseif caseNo == 3
        for i = 1:N
             if t > 1
                [alpha_hat(t+1,i), alphapr_hat(t+1,i), what(t+1,i)] = AOL_LearningPhasever2(t, Tp, eta_alp, eta_w, det_Tgt(i), xhatI_Tgt_j{i}, xhatI_Tgt(:,i,t), xhatS_Tgt(:,i,t), xhat_Tgt(:,i,t-1), xhat_Tgt_j_prev{i}, what_j{i}, what(t,i), alpha_hat(t,i), alphapr_hat(t,i), alpha(t,i), wii(t,i), del_alpha(t,i), del_wii(t,i), del_detTgt(t,i), p_mag2, e_a1, e_a2, e_w1, e_w2);
             else
                [alpha_hat(t+1,i), alphapr_hat(t+1,i), what(t+1,i)] = AOL_LearningPhasever2(t, Tp, eta_alp, eta_w, det_Tgt(i), xhatI_Tgt_j{i}, xhatI_Tgt(:,i,t), xhatS_Tgt(:,i,t), xhat_Tgt(:,i,t), xhat_Tgt_j_prev{i}, what_j{i}, what(t,i), alpha_hat(t,i), alphapr_hat(t,i), alpha(t,i), wii(t,i), del_alpha(t,i), del_wii(t,i), del_detTgt(t,i), p_mag2, e_a1, e_a2, e_w1, e_w2);
             end
         end
    elseif caseNo == 4
        for i = 1:N
             if t > 1
                [alpha_hat(t+1,i), alphapr_hat(t+1,i), what(t+1,i)] = AOL_LearningPhasever3(t, Tp, eta_alp, eta_w, det_Tgt(i), xhatI_Tgt_j{i}, xhatI_Tgt(:,i,t), xhatS_Tgt(:,i,t), xhat_Tgt(:,i,t-1), xhat_Tgt_j_prev{i}, what_j{i}, what(t,i), alpha_hat(t,i), alphapr_hat(t,i), alpha(t,i), wii(t,i), del_alpha(t,i), del_wii(t,i), del_detTgt(t,i), p_mag3, e_a1_3, e_a2_3, e_w1, e_w2);
             else
                [alpha_hat(t+1,i), alphapr_hat(t+1,i), what(t+1,i)] = AOL_LearningPhasever3(t, Tp, eta_alp, eta_w, det_Tgt(i), xhatI_Tgt_j{i}, xhatI_Tgt(:,i,t), xhatS_Tgt(:,i,t), xhat_Tgt(:,i,t), xhat_Tgt_j_prev{i}, what_j{i}, what(t,i), alpha_hat(t,i), alphapr_hat(t,i), alpha(t,i), wii(t,i), del_alpha(t,i), del_wii(t,i), del_detTgt(t,i), p_mag3, e_a1_3, e_a2_3, e_w1, e_w2);
             end
        end  
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%Velocity Estimate%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for i = 1:N
       if det_Tgt(i) > 0 % target detected
                if t > 1
                    vhat_Tgt(:,i,t) = det_Tgt_prev(i)*(Dxhat_Tgt(:,i,t) - Dxhat_Tgt(:,i,t-1))/deltaT;
                else
                    vhat_Tgt(:,i,t) = zeros(2,1);
                end                
       else % target undetected 
                if t > 1
                    vhat_Tgt(:,i,t) = vhat_Tgt(:,i,t-1);
                else
                    vhat_Tgt(:,i,t) = zeros(2,1);
                end
       end
    end
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%Control Phase%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for i = 1:N
        [ID_nrst_rtb,~] = NrstNeighDist(i,permute(xr(:,t,:),[1 3 2]));
        [v_ctrl,w_ctrl, srch_flag(t+1,i), s_cnt(t+1,i), state_s(t+1,i), dir_move_src(:,i,t+1), bound_hit_i(t+1,i)] = SwarmTrackCLAW(t, dT_safe(i), dn_safe(i), kvTf(i), kvNs(i), kvRT(i), kw(i), xr(:,t,i), phir(:,t,i), RotM(phir(:,t,i)), xr(:,t,ID_nrst_rtb), det_Tgt(i), det_Tgt_neigh{i}, neigh_ID{i}, xhat_Tgt(:,i,t), srch_flag(t,i), s_cnt(t,i), kvvTf(i), vhat_Tgt(:,i,t), dT_mon(i), state_s(t,i), xhatOD_i(:,i,t), phihatOD_i(:,i,t), theta_FOV, dir_move_src(:,i,t),box_eff, bound_hit_i(t,i));
        [xr(:,t+1,i),phir(:,t+1,i)] = robotkinematics(xr(:,t,i), phir(:,t,i), RotM(phir(:,t,i)), deltaT, v_ctrl, w_ctrl);
    end
    det_Tgt_prev = det_Tgt_lg;
    det_Tgt_neigh_prev{i} = det_Tgt(:,Gdyn_neigh(i,:));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
Gdyn(:,:,T) = comm_model(permute(xr(:,T,:),[1 3 2]),R_comm,nl,p_ld);


%%
cumul_det_score_i(n,:) = sum(det_score); 
cumul_det_score_tot_avg(n) = sum(cumul_det_score_i(n,:))./N;

cumul_closeness_score_i(n,:) = sum(closeness_score);
cumul_closeness_score_tot_avg(n) = sum(cumul_closeness_score_i(n,:))./N;
end

%%
cml_det_scr_avg(case_k,:) = cumul_det_score_tot_avg;
cml_clsns_scr_avg(case_k,:) = cumul_closeness_score_tot_avg;

case_k = case_k + 1;
end
%%
cml_det_score_mean = mean(cml_det_scr_avg,2);
cml_clsns_scr_mean = mean(cml_clsns_scr_avg,2);

%% Comparison study final plots
figure()

subplot(2,1,1)
plot(1*ones(1,NRuns),cml_det_scr_avg(1,:),2*ones(1,NRuns),cml_det_scr_avg(2,:),3*ones(1,NRuns),cml_det_scr_avg(3,:),4*ones(1,NRuns),cml_det_scr_avg(4,:),5*ones(1,NRuns),cml_det_scr_avg(5,:), 6*ones(1,NRuns),cml_det_scr_avg(6,:),'linestyle','none','marker','o','LineWidth',0.1)
set(gca,'FontSize',12)
xlabel('1: AOL-C, 2: AOL-1P, 3: AOL-2P, 4: No comm., 5: ACF, 6: KCF','FontSize',12,'Interpreter','Latex');
ylabel('Cumul. Detection Score','FontSize',12,'Interpreter','Latex');
title("Avg. Cumul. Detection Score per robot after 60 sec.",'FontSize',12,'Interpreter','Latex');
legend('AOL-C', 'AOL-1P', 'AOL-2P', 'No Comm.','ACF','KCF','Interpreter','Latex')
grid on
grid minor
xlim([0 6])
ylim([0 600])
hold on
boxplot(cml_det_scr_avg')
hold off

subplot(2,1,2)
plot(1*ones(1,NRuns),cml_clsns_scr_avg(1,:),2*ones(1,NRuns),cml_clsns_scr_avg(2,:),3*ones(1,NRuns),cml_clsns_scr_avg(3,:),4*ones(1,NRuns),cml_clsns_scr_avg(4,:),5*ones(1,NRuns),cml_clsns_scr_avg(5,:),6*ones(1,NRuns),cml_clsns_scr_avg(6,:),'linestyle','none','marker','o','LineWidth',0.1)
set(gca,'FontSize',12)
xlabel('1: AOL-C, 2: AOL-1P, 3: AOL-2P, 4: No comm., 5: ACF, 6: KCF','FontSize',12,'Interpreter','Latex');
ylabel('Cumul. Closeness Score','FontSize',12,'Interpreter','Latex');
title("Avg. Cumul. Closeness Score per robot after 60 sec.",'FontSize',12,'Interpreter','Latex');
legend('AOL-C', 'AOL-1P', 'AOL-2P', 'No Comm.','ACF','KCF','Interpreter','Latex')
grid on
grid minor
xlim([0 6])
ylim([0 600])
hold on
boxplot(cml_clsns_scr_avg')
hold off