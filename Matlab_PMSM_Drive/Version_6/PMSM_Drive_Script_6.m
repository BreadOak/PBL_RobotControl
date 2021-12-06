clc
clear all;
close all;

%% Simulation setup
Tstep = 50e-9;             % Time step
Tfinal = 0.6;              % End time
t = [0 : Tstep : Tfinal];
Step_Num = length(t);

Tsw = 41e-6;               % Current control step
Tsw_vc = 41e-5;            % Velocity control step
Tsw_pc = 41e-4;            % Position control step

Sampling_Step = floor(Tsw/Tstep);
Sampling_Step_vc = floor(Tsw_vc/Tstep);
Sampling_Step_pc = floor(Tsw_pc/Tstep);

TWOPI = 2*pi;
PIover6 = pi/6;
SQRT2 = sqrt(2);
SQRT3 = sqrt(3);
INV_SQRT3 = 1/sqrt(3);
INV_SQRT2 = 1/sqrt(2);
INV3 = 1/3;
Rpm2rm = 2*pi/60;
Rm2rpm = 60/(2*pi);

Ref = zeros(1,Step_Num);   % ref
Control_mode = 4;          % [1:Current, 2:Velocity, 3: Position 4: H-infinity]

%% Model setting
%--------------------------%
%-- PMSM model variables --%
%--------------------------%
M_Ias = zeros(1,Step_Num);
M_Ibs = zeros(1,Step_Num);
M_Ics = zeros(1,Step_Num);
M_Idss = zeros(1,Step_Num);
M_Iqss = zeros(1,Step_Num);
M_Idse = zeros(1,Step_Num);
M_Iqse = zeros(1,Step_Num);
M_Vdse = zeros(1,Step_Num);
M_Vqse = zeros(1,Step_Num);
M_Vdss = zeros(1,Step_Num);
M_Vqss = zeros(1,Step_Num);
M_Te = zeros(1,Step_Num);     % Model torque
M_Vas = zeros(1,Step_Num);
M_Vbs = zeros(1,Step_Num);
M_Vcs = zeros(1,Step_Num);

M_Wrm = zeros(1,Step_Num);
M_Wr = zeros(1,Step_Num);
M_Thetarm = zeros(1,Step_Num);
M_Thetar = zeros(1,Step_Num);

%---------------------------%
%-- PMSM model parameters --%
%---------------------------%
M_Lds = 1.707e-3;
M_Lqs = 1.707e-3;
M_Rs = 1.471;
M_LAMpm = 0.014;
M_Pole = 8;
M_PolePair = M_Pole/2;

%--------------------------%
%-- Mechanical variables --%
%--------------------------%
M_TL = zeros(1,Step_Num);     % Load torque profile

%--------------------------%
%-- Mechanical parameter --%
%--------------------------%
M_Jm = 9.039e-6;
M_Bm = 0.001*(0.001/(2*pi));  % 0.001 [Nm/krpm]

%% Inverter setting
%------------------------%
%-- Inverter variables --%
%------------------------%
I_Carrier = zeros(1,Step_Num);
I_Carrier(1) = 1;
I_Del_Carrier = -1;

%-------------------------%
%-- Inverter parameters --%
%-------------------------%
I_Vdc = 48;

%% Controler setting
%--------------------------%
%-- Controller variables --%
%--------------------------%
Thetar = zeros(1,Step_Num);       % electrical angle
Thetarm = zeros(1,Step_Num);      % mechanical angle [rad]
Wr = zeros(1,Step_Num);           % electrical anglurar velocity 
Wrpm_Ref = zeros(1,Step_Num);     % anglurar velocity [rpm] reference
Wrm_Ref = zeros(1,Step_Num);      % anglurar velocity [rad/s] reference
Wrm = zeros(1,Step_Num);          % mechanical anglurar velocity
Wrm_Integ = zeros(1,Step_Num);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Thetar_Ref = zeros(1,Step_Num);
Thetarm_Ref = zeros(1,Step_Num); 
Wrm_Integ_Ref = zeros(1,Step_Num);
Wrm_Ref_Sat = zeros(1,Step_Num);
Wrm_Ref_Fb = zeros(1,Step_Num);
Wrm_Ref_Anti = zeros(1,Step_Num);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Te_Integ_Ref = zeros(1,Step_Num);
Te_Ref = zeros(1,Step_Num);
Te_Ref_Sat = zeros(1,Step_Num);
Te_Ref_Anti = zeros(1,Step_Num);

Iqse_Ref_Sat = zeros(1,Step_Num);
Iqse_Ref_Fb = zeros(1,Step_Num);  
Ias = zeros(1,Step_Num);
Ibs = zeros(1,Step_Num);
Ics = zeros(1,Step_Num);
Idss = zeros(1,Step_Num);
Iqss = zeros(1,Step_Num);
Idse = zeros(1,Step_Num);
Iqse = zeros(1,Step_Num);
Idse_Ref = zeros(1,Step_Num);
Iqse_Ref = zeros(1,Step_Num);

Vdse_Integ_Ref = zeros(1,Step_Num);
Vqse_Integ_Ref = zeros(1,Step_Num);
Vdse_Ref = zeros(1,Step_Num);
Vqse_Ref = zeros(1,Step_Num);
Vdss_Ref = zeros(1,Step_Num);
Vqss_Ref = zeros(1,Step_Num);
Vas_Ref = zeros(1,Step_Num);
Vbs_Ref = zeros(1,Step_Num);
Vcs_Ref = zeros(1,Step_Num);
Van_Ref = zeros(1,Step_Num);
Vbn_Ref = zeros(1,Step_Num);
Vcn_Ref = zeros(1,Step_Num);
Vdse_Ref_Anti = zeros(1,Step_Num);
Vqse_Ref_Anti = zeros(1,Step_Num);
Duty_Van_Ref = zeros(1,Step_Num);
Duty_Vbn_Ref = zeros(1,Step_Num);
Duty_Vcn_Ref = zeros(1,Step_Num);

% motor spec
Lds = 1.707e-3;
Lqs = 1.707e-3;
Rs = 1.471;
LAMpm = 0.014;
Pole = 8;
PolePair = M_Pole/2;
Jm = 9.039e-6;
Bm = 0.001*(0.001/(2*pi));  % 0.001 [Nm/krpm]
Is_Rated = 4.8*sqrt(2);
Wrm_Rated = 125*(pi/180);   % [rad/s] 
Vdc = 48;

% state variable filter setting
Fc_PLL = 2*pi*200;
Wc_PLL = 2*pi*Fc_PLL;
KiT_PLL = Tsw * Wc_PLL * Wc_PLL;
Kp_PLL = 2 * 1 * Wc_PLL;

% current controller setting
Fc_cc = 500;
Wc_cc = 2*pi*Fc_cc;
KidT_cc = Tsw * Rs * Wc_cc;  % d-axis
Kpd_cc = Lds * Wc_cc;
Kpd_Anti_cc = 1/Kpd_cc;
KiqT_cc = Tsw * Rs * Wc_cc;  % q-axis
Kpq_cc = Lqs * Wc_cc;
Kpq_Anti_cc = 1/Kpq_cc;

% velocity controller setting
Fc_vc = 50;
Wc_vc = 2*pi*Fc_vc;
KiT_vc = Tsw_vc * 0.2 * Jm * Wc_vc * Wc_vc;
Kp_vc = Jm * 2 * 0.5 * Wc_vc;
Kp_Anti_vc = 1/Kp_vc;

% position controller setting
Fc_pc = 5;
Wc_pc = Fc_pc*2*pi;
KiT_pc = 0;
Kp_pc = Wc_pc;
Kp_Anti_pc = 1/Kp_pc;

% H-infinity control setting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Err_Wrm_Integ = zeros(1,Step_Num);
Ialpha = zeros(1,Step_Num);
Ibeta = zeros(1,Step_Num);
Ialpha_Ref = zeros(1,Step_Num);
Ibeta_Ref = zeros(1,Step_Num);
Wrm_dot_Ref = zeros(1,Step_Num);
Valpha = zeros(1,Step_Num);
Vbeta = zeros(1,Step_Num);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
d_hat = 0;
Xa_hat = [0;0;0;0;0];
Ea_hat = [0;0;0;0;0];
Ls = 1.707e-3;

%% Input setting
%----------------------------%
%-- Input variables setup ---%
%----------------------------%
for i = 2: 1 : Step_Num 
    % profile
    if(t(i) > 0.01)
        %Ref(i) = 10*(pi/180);             % [rad]=[deg*(pi/180)]
        Ref(i) = 1000;                    % [rpm]
        %Ref(i) = 700*sin(i*2*pi/Step_Num);
        %Ref(i) = 0.1/(1.5*PolePair*LAMpm); % [A] = [Nm/(1.5*PolePair*LAMpm)]
        %Ref(i) = sin(i*2*pi/Step_Num)*0.1/(1.5*PolePair*LAMpm); % [A] = [Nm/(1.5*PolePair*LAMpm)]
    end
    % Load torque profile [Nm]
    if(t(i) > 0.03)
        %M_TL(i) = 0.1;                     
        %M_TL(i) = Ref(i)*(1.5*PolePair*LAMpm);
        M_TL(i) = 0.1;
    end  
end

tic
%% Simulation
%--------------------------%
%-- Simulation main loop --%
%--------------------------%
for i = 2: 1 : Step_Num

    %-- Controller --%  

    % control step update
    if(mod(i,Sampling_Step) == 0) 

        % Position calculation
        M_Err_Thetarm = M_Thetarm(i-1) - Thetarm(i-1);
        M_Err_Thetarm = BOUND_PI(M_Err_Thetarm);
        Wrm_Integ(i) = Wrm_Integ(i-1) + KiT_PLL * M_Err_Thetarm;
        Wrm(i) = Wrm_Integ(i) + Kp_PLL * M_Err_Thetarm;
   
        Thetarm(i) = Thetarm(i-1) + Tsw * Wrm(i);
        Thetarm(i) = BOUND_PI(Thetarm(i));

        Wr(i) = PolePair * Wrm(i);
        Thetar(i) = Thetar(i-1) + Tsw * Wr(i);
        Thetar(i) = BOUND_PI(Thetar(i));
        Thetar_Adv = Thetar(i) + Wr(i) * Tsw;
                
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Position Controller (input: Thetarm_Ref output: Wrpm_Ref)
        if(Control_mode==3 && mod(i,Sampling_Step_pc) == 0)
            Thetarm_Ref(i) = Ref(i);
            Err_Thetarm = Thetarm_Ref(i) - Thetarm(i);
            Err_Thetarm = BOUND_PI(Err_Thetarm);
            Wrm_Integ_Ref(i) = Wrm_Integ_Ref(i-1) + KiT_pc * (Err_Thetarm - (Kp_Anti_pc*Wrm_Ref_Anti(i))); % anti-wind up
            Wrm_Ref_Fb(i) = Wrm_Integ_Ref(i) + Kp_pc * Err_Thetarm;                                        % PI controller
    
            if(Wrm_Ref_Fb(i) >= Wrm_Rated)
                Wrm_Ref_Sat(i) = Wrm_Rated;
            elseif(Wrm_Ref_Fb(i) <= -Wrm_Rated)
                Wrm_Ref_Sat(i) = -Wrm_Rated;
            else
                Wrm_Ref_Sat(i) = Wrm_Ref_Fb(i);
            end
    
            Wrm_Ref_Anti(i) = Wrm_Ref(i) - Wrm_Ref_Sat(i);
            Wrm_Ref(i) = Wrm_Ref_Sat(i);
            
        else % mod(i,Sampling_Step_pc) != 0
           Wrm_Ref(i) = Wrm_Ref(i-1);
           Thetarm_Ref(i) = Thetarm_Ref(i-1); 
           Wrm_Integ_Ref(i) = Wrm_Integ_Ref(i-1);
           Wrm_Ref_Sat(i) = Wrm_Ref_Sat(i-1);
           Wrm_Ref_Fb(i) = Wrm_Ref_Fb(i-1);
           Wrm_Ref_Anti(i) = Wrm_Ref_Anti(i-1);
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Velocity Controller (input: Wrpm_Ref, output: Iqse_Ref)
        if((Control_mode==2 || Control_mode==3) && mod(i,Sampling_Step_vc) == 0)
            
            if(Control_mode==2)
                Wrm_Ref(i) = Ref(i)* Rpm2rm;
            end
            
            Err_Wrm = Wrm_Ref(i) - Wrm(i);
            Te_Integ_Ref(i) = Te_Integ_Ref(i-1) + KiT_vc * (Err_Wrm - (Kp_Anti_vc*Te_Ref_Anti(i))); % anti-wind up
            % Te_Ref(i) = Te_Integ_Ref(i) + Kp_vc * Err_Wrm;                                        % PI controller
            Te_Ref(i) = Te_Integ_Ref(i) - Kp_vc * Wrm(i);                                           % IP controller
            Iqse_Ref_Fb(i) = Te_Ref(i) / (1.5*PolePair*LAMpm);
            
            if(Iqse_Ref_Fb(i) >= Is_Rated) 
                Iqse_Ref_Sat(i) = Is_Rated;
            elseif(Iqse_Ref_Fb(i) <= -Is_Rated)
                Iqse_Ref_Sat(i) = -Is_Rated;
            else
                Iqse_Ref_Sat(i) = Iqse_Ref_Fb(i);
            end
    
            Te_Ref_Sat(i) = 1.5*PolePair*LAMpm*Iqse_Ref_Sat(i);
            Te_Ref_Anti(i) = Te_Ref(i) - Te_Ref_Sat(i);
            Iqse_Ref(i) = Iqse_Ref_Sat(i);

        else % mod(i,Sampling_Step_vc) != 0
            Wrm_Ref(i) = Wrm_Ref(i-1);
            Te_Integ_Ref(i) = Te_Integ_Ref(i-1);
            Te_Ref(i) = Te_Ref(i-1);
            Te_Ref_Sat(i) = Te_Ref_Sat(i-1);
            Te_Ref_Anti(i) = Te_Ref_Anti(i-1);
            Iqse_Ref_Sat(i) = Iqse_Ref_Sat(i-1);
            Iqse_Ref_Fb(i) = Iqse_Ref_Fb(i-1);
            Iqse_Ref(i) = Iqse_Ref(i-1);
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Current Controller (input: Iqse_Ref, output: Duty_Van_Ref, Duty_Vbn_Ref, Duty_Vcn_Ref)   
        if(Control_mode==1)
            Iqse_Ref(i) = Ref(i);
        elseif(Control_mode==4)
            Wrm_Ref(i) = Ref(i)* Rpm2rm;
        end

        Ias(i) = M_Ias(i-1);
        Ibs(i) = M_Ibs(i-1);
        Ics(i) = M_Ics(i-1);

        Cos_Thetar = cos(Thetar(i));
        Sin_Thetar = sin(Thetar(i));

        % Clarke Transfromation
%         Ialpha(i) = Ias(i) - Ibs(i)/2 - Ics(i)/2;
%         Ibeta(i) = SQRT3*Ibs(i)/2 - SQRT3*Ics(i)/2;
        Ialpha(i) = 2*Ias(i)/3 - Ibs(i)/3 - Ics(i)/3;
        Ibeta(i) = Ibs(i)/SQRT3 - Ics(i)/SQRT3;
        
        % Nonlinear Torque Modulation
        Torque_Ref = Jm*Wrm_dot_Ref(i) + Bm*Wrm_Ref(i) + M_TL(i);
        %Torque_Ref = Jm*Wrm_dot_Ref(i) + Bm*Wrm_Ref(i);
        Ialpha_Ref(i) = -2*Torque_Ref*Sin_Thetar/3*PolePair*LAMpm;
        Ibeta_Ref(i)  =  2*Torque_Ref*Cos_Thetar/3*PolePair*LAMpm;

        Ialpha_dot_Ref = -2*Torque_Ref*Cos_Thetar*Wrm(i)/3*LAMpm;
        Ibeta_dot_Ref =  -2*Torque_Ref*Sin_Thetar*Wrm(i)/3*LAMpm;

        % Error state
        Err_Ialpha = Ialpha_Ref(i) - Ialpha(i);
        Err_Ibeta = Ibeta_Ref(i) - Ibeta(i);
        Err_Wrm = Wrm_Ref(i) - Wrm(i);
        Err_Wrm_Integ(i) = Err_Wrm_Integ(i-1) + Err_Wrm * Tsw;
        Err_z = Err_Wrm_Integ(i);
        E = [Err_z, Err_Wrm, Err_Ialpha, Err_Ibeta]';
        
        % LPV Sysytem Modeling 
        Eta1 = PolePair*LAMpm*Sin_Thetar;
        Eta2 = PolePair*LAMpm*Cos_Thetar;
        Eta1_min = -PolePair*LAMpm;
        Eta2_min = -PolePair*LAMpm;
        Eta1_max = PolePair*LAMpm;
        Eta2_max = PolePair*LAMpm;
        Eta = [Eta1, Eta2]';
        Ae_0 = [0 1 0 0; 0 -Bm/Jm 0 0; 0 0 -Rs/Ls 0; 0 0 0 -Rs/Ls];
        Ae_hat1 = [0 0 0 0; 0 0 -3/2*Jm 0; 0 1/Ls 0 0; 0 0 0 0];
        Ae_hat2 = [0 0 0 0; 0 0 0 3/2*Jm; 0 0 0 0; 0 -1/Ls 0 0];
        Ae_1 = Ae_0 + Eta1_min*Ae_hat1;
        Ae_2 = Ae_0 + Eta1_max*Ae_hat1;
        Ae_3 = Ae_0 + Eta2_min*Ae_hat2;
        Ae_4 = Ae_0 + Eta2_max*Ae_hat2;
        
        % Vertex Expansion Technique
        V_hat = [SQRT2*Eta1_min SQRT2*Eta1_max 0 0; 0 0 SQRT2*Eta2_min SQRT2*Eta2_max; 1 1 0 0; 0 0 1 1];
        Eta_hat = [Eta1, Eta2, 0.5, 0.5]';
        Xi = V_hat\Eta_hat;

        Ae_Eta1 = Xi(1)*Ae_1;
        Ae_Eta2 = Xi(2)*Ae_2;
        Ae_Eta3 = Xi(3)*Ae_3;
        Ae_Eta4 = Xi(4)*Ae_4;

        Ae_Eta = Ae_Eta1 + Ae_Eta2 + Ae_Eta3 + Ae_Eta4;
        Be = [0 0; 0 0; 1/Ls 0; 0 1/Ls];
        
        Aa_Eta = [1 zeros(1,4); zeros(4,1) Ae_Eta];
        Ba = [zeros(1,2); Be];
        Ca = [0 1 0 0 0];
        
        gamma = 1e-1;
        gamma_d = 1;
        gamma_th = 1;
        gamma_w = 3;
        gamma_ialpha = 0.001;
        gamma_ibeta = 0.001;

        gamma_ualpha = 1e4;
        gamma_ubeta = 1e4;

        Da = [1 1 1 1 1];
        Bw = diag([1 1 1 1 1]);
        %Bw = diag([0 0 -1/Jm 0 0]);
        C1 = diag([gamma_d gamma_th gamma_w gamma_ialpha gamma_ibeta]);
        Dzu = diag([gamma_ualpha gamma_ubeta]);

        G1 = -((1/gamma^2)*(Bw*Bw') - Ba*Ba');
        G2 = -((1/gamma^2)*(C1'*C1) - Ca'*Ca);
        Q1 = C1'*C1;
        Q2 = Bw*Bw';
        
        [X_inf, K1, L1] = icare(Aa_Eta,[],Q1,[],[],[],G1);
        [Y_inf, K2, L2] = icare(Aa_Eta',[],Q2,[],[],[],G2);
        F_inf = -inv(Dzu'*Dzu)*Ba'*X_inf;
        L_inf = -Y_inf*Ca'*inv(Da*Da');
        Z = inv(eye(length(Y_inf*X_inf)) - (1/gamma^2)*Y_inf*X_inf);
        A_hat_inf = Aa_Eta + (1/gamma^2)*(Bw*Bw')*X_inf + Ba*F_inf + Z*L_inf*Ca;
        I = eye(length(A_hat_inf));

        Xa_hat_dot = (I + Tsw*(Aa_Eta - L_inf*Ca))*Xa_hat + Tsw*L_inf*Thetarm(i);
        Ea_hat_dot = (I + Tsw*A_hat_inf)*Ea_hat - Tsw*Z*L_inf*Thetarm(i); 
        u = F_inf*Ea_hat;
        %u = F_inf*[0;E];
        Xa_hat = Xa_hat_dot;
        Ea_hat = Ea_hat_dot;
        d_hat = Xa_hat(1);
        
%         % H-infinity State feedback control
%         [X1,K1,L1] = icare(Ae_Eta1,Be,R1,[],[],[],G);
%         [X2,K2,L2] = icare(Ae_Eta2,Be,R1,[],[],[],G);
%         [X3,K3,L3] = icare(Ae_Eta3,Be,R1,[],[],[],G);
%         [X4,K4,L4] = icare(Ae_Eta4,Be,R1,[],[],[],G);
% 
%         if isempty(X1)
%             X1 = 0; 
%         elseif isempty(X2)
%             X2 = 0;
%         elseif isempty(X3)
%             X3 = 0;
%         elseif isempty(X4)
%             X4 = 0;
%         end
% 
%         K_1 = -inv(R2)*Be'*X1;
%         K_2 = -inv(R2)*Be'*X2;
%         K_3 = -inv(R2)*Be'*X3;
%         K_4 = -inv(R2)*Be'*X4;
%         u = -(K_1*Xi(1) + K_2*Xi(2) + K_3*Xi(3) + K_4*Xi(4))*E;

        % alpha-beta transformation
        Valpha(i) = Ls*Ialpha_dot_Ref + Rs*Ialpha_Ref(i) - PolePair*LAMpm*Wrm_Ref(i)*Sin_Thetar - u(1);
        Vbeta(i)  = Ls*Ibeta_dot_Ref  + Rs*Ibeta_Ref(i)  + PolePair*LAMpm*Wrm_Ref(i)*Cos_Thetar - u(2);
        Vas_Ref(i) = Valpha(i);
        Vbs_Ref(i) = -0.5*(Valpha(i) - SQRT3*Vbeta(i));
        Vcs_Ref(i) = -0.5*(Valpha(i) + SQRT3*Vbeta(i));
        
%         Cos_Thetar = cos(Thetar(i));
%         Sin_Thetar = sin(Thetar(i));
% 
%         Cos_Thetar_Adv = cos(Thetar_Adv);
%         Sin_Thetar_Adv = sin(Thetar_Adv);
% 
%         Idss(i) = 2. * INV3 * (Ias(i) -0.5* (Ibs(i) + Ics(i)));
%         Iqss(i) = INV_SQRT3 * (Ibs(i) - Ics(i));
% 
%         Idse(i) =  Cos_Thetar * Idss(i) + Sin_Thetar * Iqss(i);
%         Iqse(i) = -Sin_Thetar * Idss(i) + Cos_Thetar * Iqss(i);
% 
%         Err_Idse = Idse_Ref(i) - Idse(i);
%         Vdse_Integ_Ref(i) = Vdse_Integ_Ref(i-1) + KidT_cc * (Err_Idse - (Kpd_Anti_cc*Vdse_Ref_Anti(i)));
%         Vdse_Ref_Fb = Vdse_Integ_Ref(i) + Kpd_cc * Err_Idse;
%         Vdse_Ref_Ff = Wr(i)*Lds*Iqse(i);
%         Vdse_Ref(i) = Vdse_Ref_Fb + Vdse_Ref_Ff;
% 
%         Err_Iqse = Iqse_Ref(i) - Iqse(i);
%         Vqse_Integ_Ref(i) = Vqse_Integ_Ref(i-1) + KiqT_cc * (Err_Iqse - (Kpq_Anti_cc*Vqse_Ref_Anti(i)));
%         Vqse_Ref_Fb = Vqse_Integ_Ref(i) + Kpq_cc * Err_Iqse;
%         Vqse_Ref_Ff = Wr(i)*Lds*Idse(i) + Wr(i) * LAMpm;
%         Vqse_Ref(i) = Vqse_Ref_Fb + Vqse_Ref_Ff;      
% 
%         Vdss_Ref(i) =  Cos_Thetar_Adv * Vdse_Ref(i) - Sin_Thetar_Adv * Vqse_Ref(i);
%         Vqss_Ref(i) =  Sin_Thetar_Adv * Vdse_Ref(i) + Cos_Thetar_Adv * Vqse_Ref(i);
% 
%         Vas_Ref(i) = Vdss_Ref(i);
%         Vbs_Ref(i) = -0.5*(Vdss_Ref(i) - SQRT3*Vqss_Ref(i));
%         Vcs_Ref(i) = -0.5*(Vdss_Ref(i) + SQRT3*Vqss_Ref(i));

        if(Vas_Ref(i) > Vbs_Ref(i))
		    Vmax = Vas_Ref(i);
		    Vmin = Vbs_Ref(i);
        else
		    Vmax = Vbs_Ref(i);
		    Vmin = Vas_Ref(i);
        end
        if(Vcs_Ref(i) > Vmax)
        	Vmax = Vcs_Ref(i);
        end
        if(Vcs_Ref(i) < Vmin)	
            Vmin = Vcs_Ref(i);
        end

        Vsn = -0.5*(Vmax + Vmin);
        
        Van_Ref(i) = Vas_Ref(i) + Vsn;
        Vbn_Ref(i) = Vbs_Ref(i) + Vsn;
        Vcn_Ref(i) = Vcs_Ref(i) + Vsn;    

        Duty_Van_Ref(i) = Van_Ref(i) / (0.5*Vdc);
        Duty_Vbn_Ref(i) = Vbn_Ref(i) / (0.5*Vdc);
        Duty_Vcn_Ref(i) = Vcn_Ref(i) / (0.5*Vdc);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Time step update
    else  
        Thetar(i) = Thetar(i-1);
        Thetarm(i) = Thetarm(i-1);
        Wr(i) = Wr(i-1);
        Wrpm_Ref(i) = Wrpm_Ref(i-1);
        Wrm_Ref(i) = Wrm_Ref(i-1);
        Wrm(i) = Wrm(i-1);
        Wrm_Integ(i) = Wrm_Integ(i-1);

        Thetar_Ref(i) = Thetar_Ref(i-1);
        Thetarm_Ref(i) = Thetarm_Ref(i-1); 
        Wrm_Integ_Ref(i) = Wrm_Integ_Ref(i-1);
        Wrm_Ref_Sat(i) = Wrm_Ref_Sat(i-1);
        Wrm_Ref_Fb(i) = Wrm_Ref_Fb(i-1);
        Wrm_Ref_Anti(i) = Wrm_Ref_Anti(i-1);

        Te_Integ_Ref(i) = Te_Integ_Ref(i-1);
        Te_Ref(i) = Te_Ref(i-1);
        Te_Ref_Sat(i) = Te_Ref_Sat(i-1);
        Te_Ref_Anti(i) = Te_Ref_Anti(i-1);
        Iqse_Ref_Sat(i) = Iqse_Ref_Sat(i-1);
        Iqse_Ref_Fb(i) = Iqse_Ref_Fb(i-1);                        
        Ias(i) = Ias(i-1);
        Ibs(i) = Ibs(i-1);
        Ics(i) = Ics(i-1);
        Idss(i) = Idss(i-1);
        Iqss(i) = Iqss(i-1);
        Idse(i) = Idse(i-1);
        Iqse(i) = Iqse(i-1);
        Idse_Ref(i) = Idse_Ref(i-1);
        Iqse_Ref(i) = Iqse_Ref(i-1);
        Vdse_Integ_Ref(i) = Vdse_Integ_Ref(i-1);
        Vqse_Integ_Ref(i) = Vqse_Integ_Ref(i-1);
        Vdse_Ref(i) =Vdse_Ref(i-1);
        Vqse_Ref(i) = Vqse_Ref(i-1);
        Vdss_Ref(i) = Vdss_Ref(i-1);
        Vqss_Ref(i) = Vqss_Ref(i-1);
        Vas_Ref(i) = Vas_Ref(i-1);
        Vbs_Ref(i) = Vbs_Ref(i-1);
        Vcs_Ref(i) = Vcs_Ref(i-1);
        Van_Ref(i) = Van_Ref(i-1);
        Vbn_Ref(i) = Vbn_Ref(i-1);
        Vcn_Ref(i) = Vcn_Ref(i-1);
        Vdse_Ref_Anti(i) = Vdse_Ref_Anti(i-1);
        Vqse_Ref_Anti(i) = Vqse_Ref_Anti(i-1);
        Duty_Van_Ref(i) = Duty_Van_Ref(i-1);
        Duty_Vbn_Ref(i) = Duty_Vbn_Ref(i-1);
        Duty_Vcn_Ref(i) = Duty_Vcn_Ref(i-1);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        Err_Wrm_Integ(i) = Err_Wrm_Integ(i-1);
        Ialpha(i) = Ialpha(i-1);
        Ibeta(i) = Ibeta(i-1); 
        Ialpha_Ref(i) = Ialpha_Ref(i-1); 
        Ibeta_Ref(i) = Ibeta_Ref(i-1); 
        Wrm_dot_Ref(i) = Wrm_dot_Ref(i-1);
        Valpha(i) = Valpha(i-1);
        Vbeta(i) = Vbeta(i-1);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    
    %-- Inverter Update --%
    if(I_Carrier(i-1) >= 1)
        I_Carrier(i-1) = 1;
        I_Del_Carrier = -4/Sampling_Step;
    elseif(I_Carrier(i-1) <= -1)
        I_Carrier(i-1) = -1;
        I_Del_Carrier = 4/Sampling_Step;
    end
    I_Carrier(i) = I_Carrier(i-1) + I_Del_Carrier;
    if(Duty_Van_Ref(i) >= I_Carrier(i))
        M_Vas(i) = 0.5*I_Vdc;
    else
        M_Vas(i) = -0.5*I_Vdc;
    end
    if(Duty_Vbn_Ref(i) >= I_Carrier(i))
        M_Vbs(i) = 0.5*I_Vdc;
    else
        M_Vbs(i) = -0.5*I_Vdc;
    end
    if(Duty_Vcn_Ref(i) >= I_Carrier(i))
        M_Vcs(i) = 0.5*I_Vdc;
    else
        M_Vcs(i) = -0.5*I_Vdc;
    end

    %-- PMSM Update --%
    M_Cos_Thetar = cos(M_Thetar(i-1));
    M_Sin_Thetar = sin(M_Thetar(i-1));

    M_Vdss(i) = 2. * INV3 * (M_Vas(i) -0.5* (M_Vbs(i) + M_Vcs(i)));
    M_Vqss(i) = INV_SQRT3 * (M_Vbs(i) - M_Vcs(i));


    M_Vdse(i) =  M_Cos_Thetar * M_Vdss(i) + M_Sin_Thetar * M_Vqss(i);
    M_Vqse(i) = -M_Sin_Thetar * M_Vdss(i) + M_Cos_Thetar * M_Vqss(i);

    M_Delta_Idse = Tstep * (1/M_Lds) * (0.5*(M_Vdse(i-1)+M_Vdse(i)) - M_Rs*M_Idse(i-1) + M_Wr(i-1)*M_Lqs*M_Iqse(i-1));
    M_Idse(i) = M_Idse(i-1) + M_Delta_Idse;

    M_Delta_Iqse = Tstep * (1/M_Lqs) * (0.5*(M_Vqse(i-1)+M_Vqse(i)) - M_Rs*M_Iqse(i-1) - M_Wr(i-1)*(M_Lds*M_Idse(i-1) + M_LAMpm));
    M_Iqse(i) = M_Iqse(i-1) + M_Delta_Iqse;

    M_Te(i) = (3/2)*M_PolePair*(M_LAMpm*M_Iqse(i) + (M_Lds-M_Lqs)*M_Idse(i)*M_Iqse(i));

    M_Idss(i) = M_Cos_Thetar * M_Idse(i) - M_Sin_Thetar * M_Iqse(i);
    M_Iqss(i) = M_Sin_Thetar * M_Idse(i) + M_Cos_Thetar * M_Iqse(i);

    M_Ias(i) = M_Idss(i);
    M_Ibs(i) = -0.5 * (M_Idss(i) - SQRT3 * M_Iqss(i)); 
    M_Ics(i) = -0.5 * (M_Idss(i) + SQRT3 * M_Iqss(i));

    M_Delta_Wrm = Tstep * (1/M_Jm) * (0.5*(M_Te(i-1)+M_Te(i)) - M_Bm*M_Wrm(i-1) - 0.5*(M_TL(i-1)+M_TL(i)));
    M_Wrm(i) = M_Wrm(i-1) + M_Delta_Wrm;
    M_Wr(i) = M_PolePair * M_Wrm(i);
    M_Thetarm(i) = M_Thetarm(i-1) + Tstep * M_Wrm(i);    
    M_Thetarm(i) = BOUND_PI(M_Thetarm(i));
    M_Thetar(i) = M_PolePair * M_Thetarm(i);
    M_Thetar(i) = BOUND_PI(M_Thetar(i));
end
toc
%% Figures
close all;
x1 = 0;
x2 = Tfinal ;

Error_Hinf_c = Iqse_Ref*1.5*PolePair*LAMpm - M_Te;
Error_Hinf_v = Wrm_Ref*Rm2rpm - M_Wrm*Rm2rpm;
rms(Error_Hinf_v)

if(Control_mode==1)
    f = figure('Name','Motor Waveforms','NumberTitle','off');

    subplot(2,3,1);
    plot(t, Idse_Ref, t, Idse, t,M_Idse);
    xlim([x1 x2]);
    ylim([-0.1 0.1]);
    title('d-axis current [A]')
    xlabel('Time[sec]');
    ylabel('[A]')
    legend('Reference','Measured', 'Real')
    grid on;

    subplot(2,3,4);
    plot(t, Iqse_Ref, t, Iqse, t,M_Iqse);
    xlim([x1 x2]);
    ylim([0 2]);
    title('q-axis current [A]')
    xlabel('Time[sec]');
    ylabel('[A]')
    legend('Reference','Measured', 'Real')
    grid on;

    subplot(2,3,2);
    plot(t,Iqse_Ref*1.5*PolePair*LAMpm, t,M_Te, t, M_TL);
    xlim([x1 x2]);
    ylim([0 0.2]);
    title('Motor torque [Nm]')
    xlabel('Time[sec]');
    ylabel('[Nm]');
    legend('Reference torque','Real torque','Load torque')
    grid on;

    subplot(2,3,5);
    plot(t,M_Wrm*Rm2rpm);
    xlim([x1 x2]);
    ylim([0 2500]);
    title('Rotating speed [r/min]')
    xlabel('Time[sec]');
    ylabel('[r/min]')
    legend('Measured')
    grid on;

    subplot(2,3,3);
    plot(t,M_Ias, t,M_Ibs, t,M_Ics);
    xlim([x1 x2]);
    title('Phase current(Real) [A]')
    xlabel('Time[sec]');
    ylabel('[A]')
    legend('A-phase','B-phase', 'C-phase')
    grid on;

    subplot(2,3,6);
    plot(t,Van_Ref, t,Vbn_Ref,t,Vcn_Ref);
    xlim([x1 x2]);
    title('Pole voltage reference [V]')
    xlabel('Time[sec]');
    ylabel('[V]')
    legend('A-phase','B-phase', 'C-phase')
    grid on;

    f1 = figure('Name','Error(Nm)','NumberTitle','off');
    plot(t,Iqse_Ref*1.5*PolePair*LAMpm - M_Te);
    %ylim([-0.5 0.5]);
    title('Error(PI)')
    grid on;

elseif(Control_mode==2)
    f = figure('Name','Motor Waveforms','NumberTitle','off');

    subplot(2,3,1);
    plot(t, Idse_Ref, t, Idse, t,M_Idse);
    xlim([x1 x2]);
%     ylim([-1 1]);
    title('d-axis current [A]')
    xlabel('Time[sec]');
    ylabel('[A]')
    legend('Reference','Measured', 'Real')
    grid on;

    subplot(2,3,4);
    plot(t, Iqse_Ref, t, Iqse, t,M_Iqse);
    xlim([x1 x2]);
%     ylim([-5 5]);
    title('q-axis current [A]')
    xlabel('Time[sec]');
    ylabel('[A]')
    legend('Reference','Measured', 'Real')
    grid on;

    subplot(2,3,2);
    plot(t,Te_Ref_Sat, t,M_Te, t, M_TL);
    xlim([x1 x2]);
%     ylim([-1 1]);
    title('Motor torque [Nm]')
    xlabel('Time[sec]');
    ylabel('[Nm]');
    legend('Reference torque','Real torque','Load torque')
    grid on;

    subplot(2,3,5);
    plot(t,Wrm_Ref*Rm2rpm, t,M_Wrm*Rm2rpm);
    xlim([x1 x2]);
%     ylim([-1500 1500]);
    title('Rotating speed [r/min]')
    xlabel('Time[sec]');
    ylabel('[r/min]')
    legend('Reference','Measured')
    grid on;

    subplot(2,3,3);
    plot(t,M_Ias, t,M_Ibs, t,M_Ics);
    xlim([x1 x2]);
%     ylim([-5 5]);
    title('Phase current(Real) [A]')
    xlabel('Time[sec]');
    ylabel('[A]')
    legend('A-phase','B-phase', 'C-phase')
    grid on;

    subplot(2,3,6);
    plot(t,Van_Ref, t,Vbn_Ref,t,Vcn_Ref);
    xlim([x1 x2]);
%     ylim([-20 20]);
    title('Pole voltage reference [V]')
    xlabel('Time[sec]');
    ylabel('[V]')
    legend('A-phase','B-phase', 'C-phase')
    grid on;

    f1 = figure('Name','Error(rpm)','NumberTitle','off');
    plot(t,Wrm_Ref*Rm2rpm - M_Wrm*Rm2rpm);
%     ylim([-200 1000]);
    title('Error(PI_vel)')
    grid on;

elseif(Control_mode==3)
    f = figure('Name','Motor Waveforms','NumberTitle','off');

    subplot(2,3,1);
    plot(t, Idse_Ref, t, Idse, t,M_Idse);
    xlim([x1 x2]);
    title('d-axis current [A]')
    xlabel('Time[sec]');
    ylabel('[A]')
    legend('Reference','Measured', 'Real')
    grid on;

    subplot(2,3,4);
    plot(t, Iqse_Ref, t, Iqse, t,M_Iqse);
    xlim([x1 x2]);
    title('q-axis current [A]')
    xlabel('Time[sec]');
    ylabel('[A]')
    legend('Reference','Measured', 'Real')
    grid on;

%     subplot(2,3,4);
%     plot(t, Iqse_Ref);
%     xlim([x1 x2]);
%     title('q-axis current [A]')
%     xlabel('Time[sec]');
%     ylabel('[A]')
%     legend('Reference')
%     grid on;

%     subplot(2,3,2);
%     plot(t,Te_Ref_Sat, t,M_Te, t, M_TL);
%     xlim([x1 x2]);
%     title('Motor torque [Nm]')
%     xlabel('Time[sec]');
%     ylabel('[Nm]');
%     legend('Reference torque','Real torque','Load torque')
%     grid on;

    subplot(2,3,2);
    plot(t,Wrm_Ref*Rm2rpm, t,M_Wrm*Rm2rpm);
    xlim([x1 x2]);
    title('Rotating speed [r/min]')
    xlabel('Time[sec]');
    ylabel('[r/min]')
    legend('Reference','Measured')
    grid on;

    subplot(2,3,5);
    plot(t, Ref, t,M_Thetarm, t,Thetarm);
    xlim([x1 x2]);
    title('Motor angle [rad]')
    xlabel('Time[sec]');
    ylabel('[rad]')
    legend('Reference','Real','Measured')
    grid on;

    subplot(2,3,3);
    plot(t,M_Ias, t,M_Ibs, t,M_Ics);
    xlim([x1 x2]);
    title('Phase current(Real) [A]')
    xlabel('Time[sec]');
    ylabel('[A]')
    legend('A-phase','B-phase', 'C-phase')
    grid on;

    subplot(2,3,6);
    plot(t,Van_Ref, t,Vbn_Ref,t,Vcn_Ref);
    xlim([x1 x2]);
    title('Pole voltage reference [V]')
    xlabel('Time[sec]');
    ylabel('[V]')
    legend('A-phase','B-phase', 'C-phase')
    grid on;
    
    elseif(Control_mode==4)
    f = figure('Name','Motor Waveforms','NumberTitle','off');

    subplot(2,3,1);
    plot(t, Idse_Ref, t, Idse, t,M_Idse);
    xlim([x1 x2]);
    ylim([-0.5 0.5]);
    title('d-axis current [A]')
    xlabel('Time[sec]');
    ylabel('[A]')
    legend('Reference','Measured', 'Real')
    grid on;

    subplot(2,3,4);
    plot(t, Iqse_Ref, t, Iqse, t,M_Iqse);
    xlim([x1 x2]);
    ylim([-3 3]);
    title('q-axis current [A]')
    xlabel('Time[sec]');
    ylabel('[A]')
    legend('Reference','Measured', 'Real')
    grid on;

    subplot(2,3,2);
    plot(t,M_Te, t, M_TL);
    xlim([x1 x2]);
    ylim([-0.5 0.5]);
    title('Motor torque [Nm]')
    xlabel('Time[sec]');
    ylabel('[Nm]');
    legend('Real torque','Load torque')
    grid on;

    subplot(2,3,5);
    plot(t,Wrm_Ref*Rm2rpm, t,M_Wrm*Rm2rpm);
    xlim([x1 x2]);
    %ylim([-1500 1500]);
    title('Rotating speed [r/min]')
    xlabel('Time[sec]');
    ylabel('[r/min]')
    legend('Reference','Measured')
    grid on;

    subplot(2,3,3);
    plot(t,M_Ias, t,M_Ibs, t,M_Ics);
    xlim([x1 x2]);
    ylim([-3 3]);
    title('Phase current(Real) [A]')
    xlabel('Time[sec]');
    ylabel('[A]')
    legend('A-phase','B-phase', 'C-phase')
    grid on;

    subplot(2,3,6);
    plot(t,Van_Ref, t,Vbn_Ref,t,Vcn_Ref);
    xlim([x1 x2]);
    ylim([-10 10]);
    title('Pole voltage reference [V]')
    xlabel('Time[sec]');
    ylabel('[V]')
    legend('A-phase','B-phase', 'C-phase')
    grid on;

    f1 = figure('Name','Error(rpm)','NumberTitle','off');
    plot(t,Wrm_Ref*Rm2rpm - M_Wrm*Rm2rpm);
    %ylim([-150 150]);
    title('Error(Hinf_vel)')
    grid on;
end