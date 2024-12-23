clc
clear
%% Preliminary Parameters and variables

T = 400; % discrete-time steps
N = 6; % total no. of robots
NTg = 1; % no. of targets
deltaT = 0.1; % sampling rate
xr = zeros(2,T,N); % position vectors of N robots over T time-steps
phir = zeros(1,T,N); % heading angle of N robots over T time-steps

RotM = @(phi) [cos(phi) -sin(phi); sin(phi) cos(phi)]; % robot body-axis to global coordinate axis

%% Target Dynamics

box_l = 150;
delBox = 10;
box_eff = box_l - delBox;
k_box = 10.0;

xTg = zeros(2,T); % position vector of the Target over T time-steps
phiTg = zeros(1,T);

wbar_Tg = zeros(1,T);
vbar_Tg = zeros(2,T); 
delta_v = zeros(2,T);
Tc = 50;
xTg(:,1) = [rand*20; rand*20];
phiTg(:,1) = pi*(rand-0.5)*2;

for t = 1:T
    if norm(xTg(:,t),inf) >= box_eff
        delta_vdir = [-min(xTg(1,t)+box_eff,0)-max(xTg(1,t)-box_eff,0); -min(xTg(2,t)+box_eff,0)-max(xTg(2,t)-box_eff,0)];
        delta_v(:,t) = (RotM(phiTg(:,t)))'*unit_vec(delta_vdir)*k_box;
    else
        delta_v(:,t) = zeros(2,1); 
    end
    if t <= 1
        wbar_Tg(1,t) = randi([-1 1])*(6*pi/(T*deltaT) + rand*6*pi/(T*deltaT));
        vbar_Tg(:,t) = [8 + rand*2; 2 + rand*8];
    end
    if ceil(t/Tc) == floor(t/Tc)
        wbar_Tg(1,t) = randi([-1 1])*(6*pi/(T*deltaT) + rand*6*pi/(T*deltaT));
        vbar_Tg(:,t) = [8 + rand*2; 2 + rand*8];
    else
        if t > 1
            wbar_Tg(1,t) = wbar_Tg(1,t-1);
            vbar_Tg(:,t) = vbar_Tg(:,t-1);
        end
    end
    vbar_Tg_F = max(min(delta_v(:,t) + vbar_Tg(:,t),10),-10);
   [xTg(:,t+1),phiTg(:,t+1)] = targetctrldyn(deltaT,xTg(:,t),phiTg(:,t),RotM(phiTg(:,t)),vbar_Tg_F,wbar_Tg(:,t));
end

%% initial conditions for the robots

for i = 1:N
   xr(:,1,i) = [i*10; 0];
   phir(:,1,i) = pi/2;
end

%% Parameter set 1

R_FOV = 15;
R_comm = 20;
nl = 3;
p_ld = 0.1;
theta_FOV = 160*(pi/180); 
p_vl = 0.1;

dn_safe = 5*ones(1,N);
dT_safe = 10*ones(1,N);
dT_mon = 11*ones(1,N);
kvTf = 6*ones(1,N);
kvvTf = 1*ones(1,N);
kvNs = 10*ones(1,N);
kvRT = 2*ones(1,N);
kw = 10*ones(1,N);

%% Noise in the estimates of target's position 

nsr_mean = zeros(T,N);

nsr_val = [0.01*ones(1,floor(N/2)) 4*ones(1,floor(N/2)+1)];

nsr_mean(1:floor(T/6),:) = repmat(nsr_val(randperm(length(nsr_val),N)),length(1:floor(T/6)),1);
nsr_mean(ceil(T/6):floor(2*T/6),:) = repmat(nsr_val(randperm(length(nsr_val),N)),length(ceil(T/6):floor(2*T/6)),1);
nsr_mean(ceil(2*T/6):floor(3*T/6),:) = repmat(nsr_val(randperm(length(nsr_val),N)),length(ceil(2*T/6):floor(3*T/6)),1);
nsr_mean(ceil(3*T/6):floor(5*T/6),:) = repmat(nsr_val(randperm(length(nsr_val),N)),length(ceil(3*T/6):floor(5*T/6)),1);
nsr_mean(ceil(5*T/6):ceil(6*T/6),:) = repmat(nsr_val(randperm(length(nsr_val),N)),length(ceil(5*T/6):ceil(6*T/6)),1);

nsr_var_val = [0.01 4];
nsr_var_val02 = [0.01 2];
nsr_var = zeros(T,N);
for t = 1:T
    for i = 1:N
        if nsr_mean(t,i) < 2
            nsr_var(t,i) = nsr_mean(t,i);
        elseif nsr_mean(t,i) < 3 && nsr_mean(t,i) >= 2  
            nsr_var(t,i) = nsr_var_val(randperm(length(nsr_var_val02),1));
        else
            nsr_var(t,i) = nsr_var_val(randperm(length(nsr_var_val),1));
        end
    end
end

 
%% 
caseNo = 1;
k_A = 0.1;

Gdyn = zeros(N,N,T);
Ddyn = zeros(N,N,T);
detect_Tgt = zeros(NTg,N,T);
det_Tgt_neigh = cell(1,N);
neigh_ID = cell(1,N);
xr_neigh = cell(1,N);
xhatS_Tgt_j = cell(1,N);
xhatI_Tgt_j = cell(1,N);
xhat_Tgt = zeros(2,N,T);
xhatS_Tgt = zeros(2,N,T);

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

vhat_Tgt = zeros(2,N,T);
srch_flag = zeros(T,N);
s_cnt = zeros(T,N);
state_s = zeros(T,N);
det_Tgt_prev = zeros(1,N);

xhat_Tgt(:,:,1) = permute(xr(:,1,:),[1 3 2]); 
xhatS_Tgt(:,:,1) = permute(xr(:,1,:),[1 3 2]); 
DxhatS_Tgt(:,:,1) = R_FOV*([cos(permute(phir(:,1,:),[1 3 2])); sin(permute(phir(:,1,:),[1 3 2]))]);
for t = 1:T-1
    
    Gdyn(:,:,t) = comm_model(permute(xr(:,t,:),[1 3 2]),R_comm,nl,p_ld); %%%%%
    Gdyn_neigh = logical(Gdyn(:,:,t) - diag(ones(1,N)));
    
    detect_Tgt(:,:,t) = TargDet_model(permute(xr(:,t,:),[1 3 2]),permute(phir(:,t,:),[1 3 2]),xTg(:,t),R_FOV,theta_FOV,p_vl);
    det_Tgt = detect_Tgt(:,:,t);
    
    Ddyn(:,:,t) = NeighDetect_Model(permute(xr(:,t,:),[1 3 2]),permute(phir(:,t,:),[1 3 2]),R_FOV,theta_FOV,p_vl);
    Ddyn_log = Ddyn(:,:,t)>0;
    
    %%%%%%%%%%%%%%%Sensing Phase%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for i = 1:N
       if det_Tgt(i) > 0 % target detected
                xhatS_Tgt(:,i,t) = xTg(:,t); 
                DxhatS_Tgt(:,i,t) = xTg(:,t) - xr(:,t,i); % add bias
       else % target undetected 
                if t > 1
                    xhatS_Tgt(:,i,t) = xhatS_Tgt(:,i,t-1);
                    DxhatS_Tgt(:,i,t) = DxhatS_Tgt(:,i,t-1);
                end
       end
       if sum(Ddyn_log(i,:)) > 0 % at least one neighbor detected 
                xr_j = permute(xr(:,t,:),[1 3 2]);
                Dxr_j = xr_j(:,Ddyn_log(i,:)) - xr_j(:,i); % add bias
                DxhatS_Neigh{i} = Dxr_j;
                rbt_ID = 1:N;
                DxhatS_Neigh_ID{i} = rbt_ID(Ddyn_log(i,:));
                Neigh_detect_conf{i} = Ddyn(i,Ddyn_log(i,:),t);
       else
                DxhatS_Neigh{i} = [];
                DxhatS_Neigh_ID{i} = [];
                Neigh_detect_conf{i} = [];
       end   
    end
    %%%%%%%%
    %%%%%%%%%%%% Estimate Fusion Layer 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%
    if caseNo == 1
        for i = 1:N
            if t > 1
                DxhatI_Tgt(:,i,t) = AvgConsensusLayer1(DxhatS_Tgt(:,i,t),Dxhat_Tgt(:,i,t-1));
            else
                DxhatI_Tgt(:,i,t) = AvgConsensusLayer1(DxhatS_Tgt(:,i,t),DxhatS_Tgt(:,i,t));
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
       
       DxhatI_Tgt_j{i} = DxhatI_Tgt(:,Gdyn_neigh(i,:),t); % comm. neighbor Tgt estimates
       
       DxhatS_NCOM{i} = DxhatS_Neigh(Gdyn_neigh(i,:)); %cell % comm. neighbor FOV detected robots' relative positions
       DxhatS_NCOM_ID{i} = DxhatS_Neigh_ID(Gdyn_neigh(i,:)); %cell % 
       Neigh_detect_conf_NCOM{i} = Neigh_detect_conf(Gdyn_neigh(i,:)); % cell
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%Estimate Fusion Layer 2%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if caseNo == 1
         for i = 1:N
            if t > 1 
                %[xhat_Tgt(:,i,t)] = AveragingAlg(k_A,det_Tgt(i),det_Tgt_neigh{i},xhatS_Tgt(:,i,t),xhatS_Tgt_j{i}, xhat_Tgt(:,i,t-1));
                [Dxhat_Tgt(:,i,t)] = AvgConsensusLayer2(i, det_Tgt(i), det_Tgt_neigh{i}, DxhatI_Tgt(:,i,t), DxhatI_Tgt_j{i}, neigh_ID{i}, DxhatS_NCOM{i}, DxhatS_NCOM_ID{i}, Neigh_detect_conf_NCOM{i});
            end
         end  
    else
         for i = 1:N
            if t > 1 
               [xhat_Tgt(:,i,t)] = NoSharing(t, det_Tgt(i), xhatS_Tgt(:,i,t), xhat_Tgt(:,i,t-1)); 
            end
         end   
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%Velocity Estimate%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for i = 1:N
       if det_Tgt(i) == 1 % target detected
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
        [v_ctrl,w_ctrl, srch_flag(t+1,i), s_cnt(t+1,i), state_s(t+1,i)] = SwarmTrackCLAW(dT_safe(i), dn_safe(i), kvTf(i), kvNs(i), kvRT(i), kw(i), xr(:,t,i), phir(:,t,i), RotM(phir(:,t,i)), xr(:,t,ID_nrst_rtb), det_Tgt(i), det_Tgt_neigh{i}, neigh_ID{i}, Dxhat_Tgt(:,i,t), srch_flag(t,i), s_cnt(t,i), kvvTf(i), vhat_Tgt(:,i,t), dT_mon(i), state_s(t,i));
        [xr(:,t+1,i),phir(:,t+1,i)] = robotkinematics(xr(:,t,i), phir(:,t,i), RotM(phir(:,t,i)), deltaT, v_ctrl, w_ctrl);
    end
    det_Tgt_prev = det_Tgt;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
Gdyn(:,:,T) = comm_model(permute(xr(:,T,:),[1 3 2]),R_comm,nl,p_ld);

%%
n = 1;

myVideo = VideoWriter('myVideoFile'); %open video file
myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
open(myVideo)

%figure()
%figure('units','normalized','outerposition',[0 0 1 1])
figure('units','pixels','position',[0 0 1920 1080])

for t = 1:T
%     subplot(2,3,1)
%     plot(xTg(1,1*n:t),xTg(2,1*n:t),'r','LineWidth', 0.001)
%     hold on
%     plot(xr_gl(1,t,1),xr_gl(2,t,1),'x',xr_gl(1,t,2),xr_gl(2,t,2),'x',xr_gl(1,t,3),xr_gl(2,t,3),'x',xr_gl(1,t,4),xr_gl(2,t,4),'x',xr_gl(1,t,5),xr_gl(2,t,5),'x',xr_gl(1,t,6),xr_gl(2,t,6),'x',xTg(1,t),xTg(2,t),'s','LineWidth', 2, 'MarkerSize', 8)
%     hold on 
%     plot([xr_gl(1,t,1) xr_gl(1,t,1)+12*cos(phir_gl(:,t,1))],[xr_gl(2,t,1) xr_gl(2,t,1)+12*sin(phir_gl(:,t,1))],'--',[xr_gl(1,t,2) xr_gl(1,t,2)+12*cos(phir_gl(:,t,2))],[xr_gl(2,t,2) xr_gl(2,t,2)+12*sin(phir_gl(:,t,2))],'--',[xr_gl(1,t,3) xr_gl(1,t,3)+12*cos(phir_gl(:,t,3))],[xr_gl(2,t,3) xr_gl(2,t,3)+12*sin(phir_gl(:,t,3))],'--',[xr_gl(1,t,4) xr_gl(1,t,4)+12*cos(phir_gl(:,t,4))],[xr_gl(2,t,4) xr_gl(2,t,4)+12*sin(phir_gl(:,t,4))],'--',[xr_gl(1,t,5) xr_gl(1,t,5)+12*cos(phir_gl(:,t,5))],[xr_gl(2,t,5) xr_gl(2,t,5)+12*sin(phir_gl(:,t,5))],'--',[xr_gl(1,t,6) xr_gl(1,t,6)+12*cos(phir_gl(:,t,6))],[xr_gl(2,t,6) xr_gl(2,t,6)+12*sin(phir_gl(:,t,6))],'--','LineWidth', 0.1)
%     hold off
%     axis equal
%     xlim([xTg(1,t)-20 xTg(1,t)+20])
%     ylim([xTg(2,t)-20 xTg(2,t)+20])
%     xlabel('$x_1$ (m)','FontSize',12,'Interpreter','Latex')
%     ylabel('$x_2$ (m)','FontSize',12,'Interpreter','Latex')
%     title('Greedy-Local','FontSize',12,'Interpreter','Latex')
%     %pause(0.01)
% 
%     subplot(2,3,2)
%     plot(xTg(1,1*n:t),xTg(2,1*n:t),'r','LineWidth', 0.001)
%     hold on
%     plot(xr_CI(1,t,1),xr_CI(2,t,1),'x',xr_CI(1,t,2),xr_CI(2,t,2),'x',xr_CI(1,t,3),xr_CI(2,t,3),'x',xr_CI(1,t,4),xr_CI(2,t,4),'x',xr_CI(1,t,5),xr_CI(2,t,5),'x',xr_CI(1,t,6),xr_CI(2,t,6),'x',xTg(1,t),xTg(2,t),'s','LineWidth', 2, 'MarkerSize', 8)
%     hold on 
%     plot([xr_CI(1,t,1) xr_CI(1,t,1)+12*cos(phir_CI(:,t,1))],[xr_CI(2,t,1) xr_CI(2,t,1)+12*sin(phir_CI(:,t,1))],'--',[xr_CI(1,t,2) xr_CI(1,t,2)+12*cos(phir_CI(:,t,2))],[xr_CI(2,t,2) xr_CI(2,t,2)+12*sin(phir_CI(:,t,2))],'--',[xr_CI(1,t,3) xr_CI(1,t,3)+12*cos(phir_CI(:,t,3))],[xr_CI(2,t,3) xr_CI(2,t,3)+12*sin(phir_CI(:,t,3))],'--',[xr_CI(1,t,4) xr_CI(1,t,4)+12*cos(phir_CI(:,t,4))],[xr_CI(2,t,4) xr_CI(2,t,4)+12*sin(phir_CI(:,t,4))],'--',[xr_CI(1,t,5) xr_CI(1,t,5)+12*cos(phir_CI(:,t,5))],[xr_CI(2,t,5) xr_CI(2,t,5)+12*sin(phir_CI(:,t,5))],'--',[xr_CI(1,t,6) xr_CI(1,t,6)+12*cos(phir_CI(:,t,6))],[xr_CI(2,t,6) xr_CI(2,t,6)+12*sin(phir_CI(:,t,6))],'--','LineWidth', 0.1)
%     hold off
%     axis equal
%     xlim([xTg(1,t)-20 xTg(1,t)+20])
%     ylim([xTg(2,t)-20 xTg(2,t)+20])
%     xlabel('$x_1$ (m)','FontSize',12,'Interpreter','Latex')
%     ylabel('$x_2$ (m)','FontSize',12,'Interpreter','Latex')
%     title('Covariance Intersection','FontSize',12,'Interpreter','Latex')
%     %pause(0.01)
%   
%     subplot(2,3,3)
%     plot(deltaT*(1:t),Tot_cumuL_fhat(1:t)./N,deltaT*(1:t),Tot_cumuL_fhatgl(1:t)./N,deltaT*(1:t),Tot_cumuL_fhatCI(1:t)./N,deltaT*(1:t),Tot_cumuL_fhatBF(1:t)./N,'LineWidth',1.2)
%     set(gca,'FontSize',12)
%     xlabel('time (sec.)','FontSize',12,'Interpreter','Latex');
%     ylabel('Avg. Cumul. Loss','FontSize',12,'Interpreter','Latex');
%     title("Avg. cumul. loss per robot vs time",'FontSize',12,'Interpreter','Latex');
%     legend('D2EAL','Grd.Lcl.','CI','BayesF','Interpreter','Latex')
%     xlim([0 deltaT*T])
%     ylim([0 90])
%     grid on
%     grid minor
%     
%     subplot(2,3,4)
%     plot(xTg(1,1*n:t),xTg(2,1*n:t),'r','LineWidth', 0.001)
%     hold on
%     plot(xr_BF(1,t,1),xr_BF(2,t,1),'x',xr_BF(1,t,2),xr_BF(2,t,2),'x',xr_BF(1,t,3),xr_BF(2,t,3),'x',xr_BF(1,t,4),xr_BF(2,t,4),'x',xr_BF(1,t,5),xr_BF(2,t,5),'x',xr_BF(1,t,6),xr_BF(2,t,6),'x',xTg(1,t),xTg(2,t),'s','LineWidth', 2, 'MarkerSize', 8)
%     hold on 
%     plot([xr_BF(1,t,1) xr_BF(1,t,1)+12*cos(phir_BF(:,t,1))],[xr_BF(2,t,1) xr_BF(2,t,1)+12*sin(phir_BF(:,t,1))],'--',[xr_BF(1,t,2) xr_BF(1,t,2)+12*cos(phir_BF(:,t,2))],[xr_BF(2,t,2) xr_BF(2,t,2)+12*sin(phir_BF(:,t,2))],'--',[xr_BF(1,t,3) xr_BF(1,t,3)+12*cos(phir_BF(:,t,3))],[xr_BF(2,t,3) xr_BF(2,t,3)+12*sin(phir_BF(:,t,3))],'--',[xr_BF(1,t,4) xr_BF(1,t,4)+12*cos(phir_BF(:,t,4))],[xr_BF(2,t,4) xr_BF(2,t,4)+12*sin(phir_BF(:,t,4))],'--',[xr_BF(1,t,5) xr_BF(1,t,5)+12*cos(phir_BF(:,t,5))],[xr_BF(2,t,5) xr_BF(2,t,5)+12*sin(phir_BF(:,t,5))],'--',[xr_BF(1,t,6) xr_BF(1,t,6)+12*cos(phir_BF(:,t,6))],[xr_BF(2,t,6) xr_BF(2,t,6)+12*sin(phir_BF(:,t,6))],'--','LineWidth', 0.1)
%     hold off
%     axis equal
%     xlim([xTg(1,t)-20 xTg(1,t)+20])
%     ylim([xTg(2,t)-20 xTg(2,t)+20])
%     xlabel('$x_1$ (m)','FontSize',12,'Interpreter','Latex')
%     ylabel('$x_2$ (m)','FontSize',12,'Interpreter','Latex')
%     title('Bayes Fusion','FontSize',12,'Interpreter','Latex')
    %pause(0.01)
    
    subplot(1,2,1)
    plot(xTg(1,1*n:t),xTg(2,1*n:t),'r','LineWidth', 0.001)
    hold on
    plot(xr(1,t,1),xr(2,t,1),'x',xr(1,t,2),xr(2,t,2),'x',xr(1,t,3),xr(2,t,3),'x',xr(1,t,4),xr(2,t,4),'x',xr(1,t,5),xr(2,t,5),'x',xr(1,t,6),xr(2,t,6),'x',xTg(1,t),xTg(2,t),'s','LineWidth', 2, 'MarkerSize', 8)
    hold on 
    plot([xr(1,t,1) xr(1,t,1)+12*cos(phir(:,t,1))],[xr(2,t,1) xr(2,t,1)+12*sin(phir(:,t,1))],'--',[xr(1,t,2) xr(1,t,2)+12*cos(phir(:,t,2))],[xr(2,t,2) xr(2,t,2)+12*sin(phir(:,t,2))],'--',[xr(1,t,3) xr(1,t,3)+12*cos(phir(:,t,3))],[xr(2,t,3) xr(2,t,3)+12*sin(phir(:,t,3))],'--',[xr(1,t,4) xr(1,t,4)+12*cos(phir(:,t,4))],[xr(2,t,4) xr(2,t,4)+12*sin(phir(:,t,4))],'--',[xr(1,t,5) xr(1,t,5)+12*cos(phir(:,t,5))],[xr(2,t,5) xr(2,t,5)+12*sin(phir(:,t,5))],'--',[xr(1,t,6) xr(1,t,6)+12*cos(phir(:,t,6))],[xr(2,t,6) xr(2,t,6)+12*sin(phir(:,t,6))],'--','LineWidth', 0.1)
    hold off
    axis equal
    xlim([-box_l box_l])
    ylim([-box_l box_l])
    xlabel('$x_1$ (m)','FontSize',12,'Interpreter','Latex')
    ylabel('$x_2$ (m)','FontSize',12,'Interpreter','Latex')
    title('D2EAL','FontSize',12,'Interpreter','Latex')
    legend('Mothership Trajectory','Robot 1','Robot 2','Robot 3','Robot 4','Robot 5','Robot 6','Mothership')
    
    subplot(1,2,2)
    hg = plot(digraph(Gdyn(:,:,t)-diag(ones(1,N))));
    hg.XData = [1 1+sqrt(2) sqrt(2)+2 sqrt(2)+1 1 0];
    hg.YData = [0 0 1 1+sqrt(2) 1+sqrt(2) sqrt(2)];
    hg.LineWidth = 2;
    hg.NodeColor = [0 0.4470 0.7410;
                    0.8500 0.3250 0.0980;
                    0.9290 0.6940 0.1250;
                    0.4940 0.1840 0.5560;
                    0.4660 0.6740 0.1880;
                    0.3010 0.7450 0.9330];
    hg.MarkerSize = 6;
    title('Communication Network')
    xlabel('Nodes are robots')
    ylabel('Edges are communication links')
    pause(0.1)
    
    sgtitle(['Time = ',num2str(t*deltaT,'%4.1f'),' sec.']) 
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
end
close(myVideo)
%%
% n = 1;
% for t = 400
% figure()
%     plot(xTg(1,1*n:t),xTg(2,1*n:t),'r','LineWidth', 0.001)
%     hold on
%     plot(xr(1,t,1),xr(2,t,1),'x',xr(1,t,2),xr(2,t,2),'x',xr(1,t,3),xr(2,t,3),'x',xr(1,t,4),xr(2,t,4),'x',xr(1,t,5),xr(2,t,5),'x',xr(1,t,6),xr(2,t,6),'x',xTg(1,t),xTg(2,t),'s','LineWidth', 2, 'MarkerSize', 8)
%     hold on 
%     plot([xr(1,t,1) xr(1,t,1)+12*cos(phir(:,t,1))],[xr(2,t,1) xr(2,t,1)+12*sin(phir(:,t,1))],'--',[xr(1,t,2) xr(1,t,2)+12*cos(phir(:,t,2))],[xr(2,t,2) xr(2,t,2)+12*sin(phir(:,t,2))],'--',[xr(1,t,3) xr(1,t,3)+12*cos(phir(:,t,3))],[xr(2,t,3) xr(2,t,3)+12*sin(phir(:,t,3))],'--',[xr(1,t,4) xr(1,t,4)+12*cos(phir(:,t,4))],[xr(2,t,4) xr(2,t,4)+12*sin(phir(:,t,4))],'--',[xr(1,t,5) xr(1,t,5)+12*cos(phir(:,t,5))],[xr(2,t,5) xr(2,t,5)+12*sin(phir(:,t,5))],'--',[xr(1,t,6) xr(1,t,6)+12*cos(phir(:,t,6))],[xr(2,t,6) xr(2,t,6)+12*sin(phir(:,t,6))],'--','LineWidth', 0.1)
%     hold off
%     axis equal
%     xlim([-400 +400])
%     ylim([-400 +400])
%     xlabel('$x_1$ (m)','FontSize',12,'Interpreter','Latex')
%     ylabel('$x_2$ (m)','FontSize',12,'Interpreter','Latex')
%     title('D2EAL','FontSize',12,'Interpreter','Latex')
%     legend('Mothership Trajectory','Robot 1','Robot 2','Robot 3','Robot 4','Robot 5','Robot 6','Mothership')
% end