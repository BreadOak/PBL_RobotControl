clc
clear all;
close all;

%% Matlab to ROS2 setup
PMSM_Node = ros2node("/matlab_node", 0);   % node initialization
pause(1);

% % Subscirber
CurrentMode_sub = ros2subscriber(PMSM_Node, "/CurrentMode", "std_msgs/Int32", @callback_CurrentMode);
VelocityMode_sub = ros2subscriber(PMSM_Node, "/VelocityMode", "std_msgs/Int32", @callback_VelocityMode);
PositionMode_sub = ros2subscriber(PMSM_Node, "/PositionMode", "std_msgs/Int32", @callback_PositionMode);
DOBoption_sub = ros2subscriber(PMSM_Node, "/DOBoption", "std_msgs/Int32", @callback_DOBoption);
ModeSelection_sub = ros2subscriber(PMSM_Node, "/mode_selection", "std_msgs/Int32", @callback_ModeSelection);
Reference_sub = ros2subscriber(PMSM_Node, "/Reference", "std_msgs/Float64", @callback_Reference);
global CurrentMode;
global VelocityMode;
global PositionMode;
global DOBoption;
global mode_selection;
global Reference;

field1 = 'label';  value1 = 'PMSM_data';
field2 = 'size';  value2 = uint32(0);
field3 = 'stride';  value3 = uint32(0);
s = struct(field1, value1, field2, value2, field3, value3);

% Publisher
Current_pub = ros2publisher(PMSM_Node,"/Current","std_msgs/Float64MultiArray");
Current_pubMsg = ros2message("std_msgs/Float64MultiArray");
Current_pubMsg.layout.dim = s;
Current_pubMsg.layout.data_offset = uint32(0);

Voltage_pub = ros2publisher(PMSM_Node,"/Voltage","std_msgs/Float64MultiArray");
Voltage_pubMsg = ros2message("std_msgs/Float64MultiArray");
Voltage_pubMsg.layout.dim = s;
Voltage_pubMsg.layout.data_offset = uint32(0);

Torque_pub = ros2publisher(PMSM_Node,"/Torque","std_msgs/Float64MultiArray");
Torque_pubMsg = ros2message("std_msgs/Float64MultiArray");
Torque_pubMsg.layout.dim = s;
Torque_pubMsg.layout.data_offset = uint32(0);

Velocity_pub = ros2publisher(PMSM_Node,"/Velocity","std_msgs/Float64MultiArray");
Velocity_pubMsg = ros2message("std_msgs/Float64MultiArray");
Velocity_pubMsg.layout.dim = s;
Velocity_pubMsg.layout.data_offset = uint32(0);

Angle_pub = ros2publisher(PMSM_Node,"/Angle","std_msgs/Float64MultiArray");
Angle_pubMsg = ros2message("std_msgs/Float64MultiArray");
Angel_pubMsg.layout.dim = s;
Angel_pubMsg.layout.data_offset = uint32(0);

%% Simulation setup
Tstep = 50e-9;             % Time step
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

% Reference
Ref = 0;  
% Loop Count
Count = 0;
mode_selection = 0;

%---------------------------Control mode select---------------------------%
% 0: Not use      1: Current(PI)   2: Current(MPC)      
% 0: Not use      1: Velocity(PI)  2: Velocity(MPC) 3: Velocity(H-inf)    %
% 0: Not use      1: Position(PI)      
% 0: Not use      1: Current(PI+DOB)
%-------------------------------------------------------------------------%
Current_Control_mode = 0;    
Velocity_Control_mode = 0;
Position_Control_mode = 0;
DOB_mode = 0;

% % Subscirber Input

%% Model setting
%--------------------------%
%-- PMSM model variables --%
%--------------------------%
M_Ias = 0;
M_Ibs = 0;
M_Ics = 0;
M_Idss = 0;
M_Iqss = 0;
M_Idse = 0;
M_Iqse = 0;
M_Vdse = 0;
M_Vqse = 0;
M_Vdss = 0;
M_Vqss = 0;
M_Te = 0;  % Model Torque
M_Vas = 0;
M_Vbs = 0;
M_Vcs = 0;

M_Wrm = 0;
M_Wr = 0;
M_Thetarm = 0;
M_Thetar = 0;

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
M_TL = 0;     % Load torque profile

%--------------------------%
%-- Mechanical parameter --%
%--------------------------%
M_Jm = 9.039e-6;
M_Bm = 0.001*(0.001/(2*pi));  % 0.001 [Nm/krpm]

%-----------------------------%
%-- Previous value -----------%
%-----------------------------%
M_Te_prev = 0;
M_Vdse_prev = 0;
M_Vqse_prev = 0;
M_TL_prev = 0;

Idse_prev = 0;
Iqse_prev = 0;

%% Inverter setting
%------------------------%
%-- Inverter variables --%
%------------------------%
I_Carrier = 0;
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
Thetar = 0;       % electrical angle
Thetarm = 0;      % mechanical angle [rad]
Wr = 0;           % electrical anglurar velocity 
Wrpm_Ref = 0;     % anglurar velocity [rpm] reference
Wrm_Ref = 0;      % anglurar velocity [rad/s] reference
Wrm = 0;          % mechanical anglurar velocity
Wrm_Integ = 0;

Thetar_Ref = 0;
Thetarm_Ref = 0; 
Wrm_Integ_Ref = 0;
Wrm_Ref_Sat = 0;
Wrm_Ref_Fb = 0;
Wrm_Ref_Anti = 0;

Te_Integ_Ref = 0;
Te_Ref = 0;
Te_Ref_Sat = 0;
Te_Ref_Anti = 0;

Iqse_Ref_Sat = 0;
Iqse_Ref_Fb = 0; 
Ias = 0;
Ibs = 0;
Ics = 0;
Idss = 0;
Iqss = 0;
Idse = 0;
Iqse = 0;
Idse_Ref = 0;
Iqse_Ref = 0;

Vdse_Integ_Ref = 0;
Vqse_Integ_Ref = 0;
Vdse_Ref = 0;
Vqse_Ref = 0;
Vdss_Ref = 0;
Vqss_Ref = 0;
Vas_Ref = 0;
Vbs_Ref = 0;
Vcs_Ref = 0;
Van_Ref = 0;
Vbn_Ref = 0;
Vcn_Ref = 0;
Vdse_Ref_Anti = 0;
Vqse_Ref_Anti = 0;
Duty_Van_Ref = 0;
Duty_Vbn_Ref = 0;
Duty_Vcn_Ref = 0;

% Motor spec
Lds = 1.707e-3;
Lqs = 1.707e-3;
Rs = 1.471;
LAMpm = 0.014;
Pole = 8;
PolePair = M_Pole/2;
Jm = 9.039e-6;
Bm = 0.001*(0.001/(2*pi));             % 0.001 [Nm/krpm]
Gear_ratio = 121;
Is_Rated = 4.8*sqrt(2);
Wrm_Rated = Gear_ratio*125*(pi/180);   % [rad/s] 
Vdc = 48;

% State variable filter setting
Fc_PLL = 2*pi*200;
Wc_PLL = 2*pi*Fc_PLL;
KiT_PLL = Tsw * Wc_PLL * Wc_PLL;
Kp_PLL = 2 * 1 * Wc_PLL;

%% Current(PI) controller setting
Fc_cc = 500;
Wc_cc = 2*pi*Fc_cc;
KidT_cc = Tsw * Rs * Wc_cc;  % d-axis
Kpd_cc = Lds * Wc_cc;
Kpd_Anti_cc = 1/Kpd_cc;
KiqT_cc = Tsw * Rs * Wc_cc;  % q-axis
Kpq_cc = Lqs * Wc_cc;
Kpq_Anti_cc = 1/Kpq_cc;

%% Current(MPC) controller setting
u = [0;0]; % initial input
Np = 5;
Nc = 1;
rw = 1;
%Delta_t = Tsw;
Bar_R = rw*eye(length(u)*Nc);

%% Current(DOB) controller setting
beta = 10;
disturbance_d = 0;
disturbance_q = 0;

%% Velocity(PI) controller setting
Fc_vc = 50;
Wc_vc = 2*pi*Fc_vc;
KiT_vc = Tsw_vc * 0.2 * Jm * Wc_vc * Wc_vc;
Kp_vc = Jm * 2 * 0.5 * Wc_vc;
Kp_Anti_vc = 1/Kp_vc;

%% Velocity(MPC) controller setting
u = [0;0]; % initial input
N = 5;
KI = 10;
gam_id = 100;
gam_iq = 1;
gam_wme = 10;
gam_u = 0.01;

Gam_i = [gam_id, 0; 0, gam_iq];
Gam_wme = [gam_wme, -gam_wme; -gam_wme, gam_wme];

Delta_t = Tsw;

% Continuous Model
Ac = [-Rs/Lds, 0, Lqs/Lds, 0; 0, -Rs/Lqs, 0, -LAMpm/Lqs; 0, 0, 0, 0; 0, 1.5*PolePair*LAMpm/Jm, 0, -Bm/Jm];
Bc = [1/Lds, 0; 0, 1/Lqs; 0, 0; 0, 0];

% Discrete Model
Ad = [1-Delta_t*Rs/Lds, 0, Delta_t*Lqs/Lds, 0; 0, 1-Delta_t*Rs/Lqs, 0, -Delta_t*LAMpm/Lqs; 0, 0, 1, 0; 0, Delta_t*1.5*PolePair*LAMpm/Jm, 0, 1-Delta_t*Bm/Jm];
Bd = [Delta_t/Lds, 0; 0, Delta_t/Lqs; 0, 0; 0, 0];

% Augmented Model
Aa = [Ad, zeros(4,1), Bd; zeros(1,4), 1, zeros(1,2); zeros(2,4), zeros(2,1), eye(2,2)];
Ba = [zeros(5,2); eye(2,2)];

Q = [Gam_i, zeros(2,1), zeros(2,4); zeros(1,2), zeros(1,1), zeros(1,4); zeros(2,3), Gam_wme, zeros(2,2); zeros(2,2), zeros(2,1), zeros(2,4)];
R = [gam_u, 0; 0, gam_u];

model = LTISystem('A',Aa,'B',Ba);
model.x.min = [-0.5; -5; -7500*0.42; -1500*0.42; -1000*0.42; -6; -6];
model.x.max = [ 0.5;  5;  7500*0.42;  1500*0.42;  1000*0.42;  6;  6];
model.u.min = [-12; -12];
model.u.max = [ 12;  12];
model.x.penalty = QuadFunction(Q);
model.u.penalty = QuadFunction(R);
mpc = MPCController(model, N);

%% Velocity(H-inf) controller setting
Err_Wrm_Integ = 0;
Ialpha = 0;
Ibeta = 0;
Ialpha_Ref = 0;
Ibeta_Ref = 0;
Wrm_dot_Ref = 0;
Valpha = 0;
Vbeta = 0;
d_hat = 0;
Xa_hat = [0;0;0;0;0];
Ea_hat = [0;0;0;0;0];
Ls = 1.707e-3;

Eta1_min = -PolePair*LAMpm;
Eta2_min = -PolePair*LAMpm;
Eta1_max = PolePair*LAMpm;
Eta2_max = PolePair*LAMpm;

Ae_0 = [0 1 0 0; 0 -Bm/Jm 0 0; 0 0 -Rs/Ls 0; 0 0 0 -Rs/Ls];
Ae_hat1 = [0 0 0 0; 0 0 -3/2*Jm 0; 0 1/Ls 0 0; 0 0 0 0];
Ae_hat2 = [0 0 0 0; 0 0 0 3/2*Jm; 0 0 0 0; 0 -1/Ls 0 0];
Ae_1 = Ae_0 + Eta1_min*Ae_hat1;
Ae_2 = Ae_0 + Eta1_max*Ae_hat1;
Ae_3 = Ae_0 + Eta2_min*Ae_hat2;
Ae_4 = Ae_0 + Eta2_max*Ae_hat2;
Be = [0 0; 0 0; 1/Ls 0; 0 1/Ls];

Aa_Eta_1 = [0 zeros(1,4); zeros(4,1) Ae_1];
Aa_Eta_2 = [0 zeros(1,4); zeros(4,1) Ae_2];
Aa_Eta_3 = [0 zeros(1,4); zeros(4,1) Ae_3];
Aa_Eta_4 = [0 zeros(1,4); zeros(4,1) Ae_4];
Ba = [zeros(1,2); Be];
Ca = [0 1 0 0 0];
Da = [1 1 1 1 1];

gamma = 1e5;
gamma_d = 1;
gamma_th = 1;
gamma_w = 3;
gamma_ialpha = 0.001;
gamma_ibeta = 0.001;
gamma_ualpha = 1e4;
gamma_ubeta = 1e4;

Bw = diag([1 1 1 1 1]);
C1 = [gamma_d gamma_th gamma_w gamma_ialpha gamma_ibeta];
Dzu = [gamma_ualpha gamma_ubeta];

setlmis([]);
X_inf = lmivar(1,[5,1]); % Symmetric Matrix 5 by 5
Z_inf = lmivar(2,[2,5]); % Matrix 2 by 5

lmiterm([-1 1 1 X_inf],1,1); % X_inf > 0

% LMI
lmiterm([2 1 1 X_inf],Aa_Eta_1,1,'s');
lmiterm([2 1 1 Z_inf],Ba,1,'s');
lmiterm([2 1 2 0],Bw);
lmiterm([2 1 3 X_inf],1,C1');
lmiterm([2 1 3 -Z_inf],1,Dzu');
lmiterm([2 2 2 0],-gamma);
lmiterm([2 2 3 0],0);
lmiterm([2 3 3 0],-gamma);

lmiterm([3 1 1 X_inf],Aa_Eta_2,1,'s');
lmiterm([3 1 1 Z_inf],Ba,1,'s');
lmiterm([3 1 2 0],Bw);
lmiterm([3 1 3 X_inf],1,C1');
lmiterm([3 1 3 -Z_inf],1,Dzu');
lmiterm([3 2 2 0],-gamma);
lmiterm([3 2 3 0],0);
lmiterm([3 3 3 0],-gamma);

lmiterm([4 1 1 X_inf],Aa_Eta_3,1,'s');
lmiterm([4 1 1 Z_inf],Ba,1,'s');
lmiterm([4 1 2 0],Bw);
lmiterm([4 1 3 X_inf],1,C1');
lmiterm([4 1 3 -Z_inf],1,Dzu');
lmiterm([4 2 2 0],-gamma);
lmiterm([4 2 3 0],0);
lmiterm([4 3 3 0],-gamma);

lmiterm([5 1 1 X_inf],Aa_Eta_4,1,'s');
lmiterm([5 1 1 Z_inf],Ba,1,'s');
lmiterm([5 1 2 0],Bw);
lmiterm([5 1 3 X_inf],1,C1');
lmiterm([5 1 3 -Z_inf],1,Dzu');
lmiterm([5 2 2 0],-gamma);
lmiterm([5 2 3 0],0);
lmiterm([5 3 3 0],-gamma);

LMIs = getlmis;
options = [1e-5,0,0,0,0];
[TMIN,XFEAS] = feasp(LMIs,options,0);

X_inf = dec2mat(LMIs,XFEAS,X_inf);
Z_inf = dec2mat(LMIs,XFEAS,Z_inf);
F_inf = Z_inf*inv(X_inf); % state feedback gain

%% Position(PI) controller setting
Fc_pc = 5;
Wc_pc = Fc_pc*2*pi;
KiT_pc = 0;
Kp_pc = Wc_pc;
Kp_Anti_pc = 1/Kp_pc;

%% Simulation 
while(1)
    
    Current_Control_mode =  CurrentMode;
    Velocity_Control_mode =  VelocityMode;
    Position_Control_mode =  PositionMode;
    DOB_mode =  DOBoption;

    if (mode_selection > 0)
        
        SimulationTime = Count*Tstep;
        
        % % Input setting
        % if(SimulationTime > 0.01)          
        %     % Subscirber Input    
        %     Ref = Reference;
        % end
        
        % % Load torque profile [Nm]
        % if(SimulationTime > 0.03)     
        %     M_TL = 0;
        % end
    
        % Input setting
        if(SimulationTime > 0.01)  
            % Subscirber Input  
            Ref = Reference; %[rpm]
            if (Position_Control_mode==1)
                Ref = Reference*(pi/180); %[rad]
            elseif(Position_Control_mode==0 && Velocity_Control_mode==0 && (Current_Control_mode==1||Current_Control_mode==2))
                Ref = Reference/(1.5*PolePair*LAMpm); %[A] = [Nm/(1.5*PolePair*LAMpm)]
            end      
        end
        
        % Load torque profile [Nm]
        if(SimulationTime > 0.03)     
            M_TL = 0;
            if (Position_Control_mode==0 && Velocity_Control_mode==0 && (Current_Control_mode==1||Current_Control_mode==2))
                M_TL = Ref*(1.5*PolePair*LAMpm);
            end
        end 

        %-- Control Step Update --%%
        if(mod(Count,Sampling_Step) == 0) 
    
            % Position calculation
            M_Err_Thetarm = M_Thetarm - Thetarm;
            M_Err_Thetarm = BOUND_PI(M_Err_Thetarm);
            Wrm_Integ = Wrm_Integ + KiT_PLL * M_Err_Thetarm;
            Wrm = Wrm_Integ + Kp_PLL * M_Err_Thetarm;
       
            Thetarm = Thetarm + Tsw * Wrm;
            Thetarm = BOUND_PI(Thetarm);
    
            Wr = PolePair * Wrm;
            Thetar = Thetar + Tsw * Wr;
            Thetar = BOUND_PI(Thetar);
            Thetar_Adv = Thetar + Wr * Tsw;
    
            %%%%%%%%%% Position Controller %%%%%%%%%% 
    
            if(Position_Control_mode==1 && mod(Count,Sampling_Step_pc) == 0)
                %%%%%%%%%%%%%%%%%%%
                %%% Position PI %%%
                %%%%%%%%%%%%%%%%%%%
                Thetarm_Ref = Ref;
                Err_Thetarm = Thetarm_Ref - Thetarm;
                Err_Thetarm = BOUND_PI(Err_Thetarm);
                Wrm_Integ_Ref = Wrm_Integ_Ref + KiT_pc * (Err_Thetarm - (Kp_Anti_pc*Wrm_Ref_Anti)); % anti-wind up
                Wrm_Ref_Fb = Wrm_Integ_Ref + Kp_pc * Err_Thetarm;                                   % PI controller
        
                % Satutration
                if(Wrm_Ref_Fb >= Wrm_Rated)
                    Wrm_Ref_Sat = Wrm_Rated;
                elseif(Wrm_Ref_Fb <= -Wrm_Rated)
                    Wrm_Ref_Sat = -Wrm_Rated;
                else
                    Wrm_Ref_Sat = Wrm_Ref_Fb;
                end
        
                Wrm_Ref_Anti = Wrm_Ref - Wrm_Ref_Sat;
                Wrm_Ref = Wrm_Ref_Sat;
            end
    
            %%%%%%%%%% Velocity Controller %%%%%%%%%% 
    
            if((Velocity_Control_mode~=0 || Position_Control_mode==1) && mod(Count,Sampling_Step_vc) == 0)   
                %%%%%%%%%%%%%%%%%%%
                %%% Velocity PI %%%
                %%%%%%%%%%%%%%%%%%%
                if(Velocity_Control_mode==1 && Position_Control_mode==0)
                    Wrm_Ref = Ref* Rpm2rm;
                end
                Err_Wrm = Wrm_Ref - Wrm;
                Te_Integ_Ref = Te_Integ_Ref + KiT_vc * (Err_Wrm - (Kp_Anti_vc*Te_Ref_Anti)); % anti-wind up
                % Te_Ref(i) = Te_Integ_Ref(i) + Kp_vc * Err_Wrm;                             % PI controller
                Te_Ref = Te_Integ_Ref - Kp_vc * Wrm;                                         % IP controller
                Iqse_Ref_Fb = Te_Ref / (1.5*PolePair*LAMpm);
    
                % Satutration
                if(Iqse_Ref_Fb >= Is_Rated) 
                    Iqse_Ref_Sat = Is_Rated;
                elseif(Iqse_Ref_Fb <= -Is_Rated)
                    Iqse_Ref_Sat = -Is_Rated;
                else
                    Iqse_Ref_Sat = Iqse_Ref_Fb;
                end
    
                Te_Ref_Sat = 1.5*PolePair*LAMpm*Iqse_Ref_Sat;
                Te_Ref_Anti = Te_Ref - Te_Ref_Sat;
                Iqse_Ref = Iqse_Ref_Sat;
            end
    
            %%%%%%%%%% Current Controller %%%%%%%%%%
    
            Ias = M_Ias;
            Ibs = M_Ibs;
            Ics = M_Ics;
    
            Cos_Thetar = cos(Thetar);
            Sin_Thetar = sin(Thetar);
    
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%% Velocity & Current(MPC) %%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if(Velocity_Control_mode==2 && Current_Control_mode==0)
                Cos_Thetar_Adv = cos(Thetar_Adv);
                Sin_Thetar_Adv = sin(Thetar_Adv);
    
                if(Position_Control_mode~=1)
                    Wrm_Ref = Ref* Rpm2rm;
                end
    
                Wr_Ref = PolePair*Wrm_Ref;
                Wr_Ref = Wr_Ref + KI*(Wr_Ref-Wr)*Delta_t;
                xm = [Idse; Iqse];
                Xtr = [xm; Wr*Iqse; Wr; Wr_Ref; u];
                delta_u = mpc.evaluate(Xtr);
                u = u + delta_u;
        
                Vdse_Ref = u(1);
                Vqse_Ref = u(2);
        
                Vdss_Ref =  Cos_Thetar_Adv * Vdse_Ref - Sin_Thetar_Adv * Vqse_Ref;
                Vqss_Ref =  Sin_Thetar_Adv * Vdse_Ref + Cos_Thetar_Adv * Vqse_Ref;
        
                Vas_Ref = Vdss_Ref;
                Vbs_Ref = -0.5*(Vdss_Ref - SQRT3*Vqss_Ref);
                Vcs_Ref = -0.5*(Vdss_Ref + SQRT3*Vqss_Ref);
    
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%% Velocity & Current(H-inf) %%% - It's incomplete.
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            elseif(Velocity_Control_mode==3 && Current_Control_mode==0)
    
                if(Position_Control_mode~=1)
                    Wrm_Ref = Ref* Rpm2rm;
                end
    
                % Clarke Transfromation
                %Ialpha = Ias - Ibs/2 - Ics/2;
                %Ibeta = SQRT3*Ibs/2 - SQRT3*Ics/2;
                Ialpha = 2*Ias/3 - Ibs/3 - Ics/3;
                Ibeta = Ibs/SQRT3 - Ics/SQRT3;
    
                % Nonlinear Torque Modulation
                %Torque_Ref = Jm*Wrm_dot_Ref(i) + Bm*Wrm_Ref(i) + M_TL(i);
                Torque_Ref = Jm*Wrm_dot_Ref + Bm*Wrm_Ref;
                Ialpha_Ref = -2*Torque_Ref*Sin_Thetar/3*PolePair*LAMpm;
                Ibeta_Ref  =  2*Torque_Ref*Cos_Thetar/3*PolePair*LAMpm; 
                Ialpha_dot_Ref = -2*Torque_Ref*Cos_Thetar*Wrm/3*LAMpm;
                Ibeta_dot_Ref =  -2*Torque_Ref*Sin_Thetar*Wrm/3*LAMpm;
    
                % Error State
                Err_Ialpha = Ialpha_Ref - Ialpha;
                Err_Ibeta = Ibeta_Ref - Ibeta;
                Err_Wrm = Wrm_Ref - Wrm;
                Err_Wrm_Integ = Err_Wrm_Integ + Err_Wrm * Tsw;
                Err_z = Err_Wrm_Integ;
                E = [Err_z, Err_Wrm, Err_Ialpha, Err_Ibeta]';
                
                % State Feedback Control Input
                u = F_inf*[0;E];
               
                % alpha-beta transformation
                Valpha = Ls*Ialpha_dot_Ref + Rs*Ialpha_Ref - PolePair*LAMpm*Wrm_Ref*Sin_Thetar - u(1);
                Vbeta  = Ls*Ibeta_dot_Ref  + Rs*Ibeta_Ref  + PolePair*LAMpm*Wrm_Ref*Cos_Thetar - u(2);
                Vas_Ref = Valpha;
                Vbs_Ref = -0.5*(Valpha - SQRT3*Vbeta);
                Vcs_Ref = -0.5*(Valpha + SQRT3*Vbeta);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
            %%% Current PI (Option : DOB) %%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            elseif(Current_Control_mode==1)
    
                if(Velocity_Control_mode~=1)
                    Iqse_Ref = Ref;
                end
                
                Cos_Thetar_Adv = cos(Thetar_Adv);
                Sin_Thetar_Adv = sin(Thetar_Adv);
    
                Idss = 2. * INV3 * (Ias -0.5* (Ibs + Ics));
                Iqss = INV_SQRT3 * (Ibs - Ics);
    
                Idse =  Cos_Thetar * Idss + Sin_Thetar * Iqss;
                Iqse = -Sin_Thetar * Idss + Cos_Thetar * Iqss;
                
                if(DOB_mode==1)
                    Err_Idse = Idse_Ref - Idse;
                    Vdse_Integ_Ref = Vdse_Integ_Ref + KidT_cc * (Err_Idse - (Kpd_Anti_cc*Vdse_Ref_Anti));
                    Vdse_Ref_Fb = Vdse_Integ_Ref + Kpd_cc * Err_Idse;
                    dd_hat = (beta + 1)*(Lds*Wc_cc*Err_Idse + Rs*Idse - Vdse_Ref_Fb);
                    Vdse_Ref = Vdse_Ref_Fb - dd_hat + disturbance_d;
    
                    Err_Iqse = Iqse_Ref - Iqse;
                    Vqse_Integ_Ref = Vqse_Integ_Ref + KiqT_cc * (Err_Iqse - (Kpq_Anti_cc*Vqse_Ref_Anti));
                    Vqse_Ref_Fb = Vqse_Integ_Ref + Kpq_cc * Err_Iqse;
                    dq_hat = (beta + 1)*(Lqs*Wc_cc*Err_Iqse + Rs*Iqse - Vqse_Ref_Fb);
                    Vqse_Ref = Vqse_Ref_Fb - dq_hat + disturbance_q;
                else
                    Err_Idse = Idse_Ref - Idse;
                    Vdse_Integ_Ref = Vdse_Integ_Ref + KidT_cc * (Err_Idse - (Kpd_Anti_cc*Vdse_Ref_Anti));
                    Vdse_Ref_Fb = Vdse_Integ_Ref + Kpd_cc * Err_Idse;
                    Vdse_Ref_Ff = Wr*Lds*Iqse;
                    Vdse_Ref = Vdse_Ref_Fb + Vdse_Ref_Ff;
            
                    Err_Iqse = Iqse_Ref - Iqse;
                    Vqse_Integ_Ref = Vqse_Integ_Ref + KiqT_cc * (Err_Iqse - (Kpq_Anti_cc*Vqse_Ref_Anti));
                    Vqse_Ref_Fb = Vqse_Integ_Ref + Kpq_cc * Err_Iqse;
                    Vqse_Ref_Ff = Wr*Lds*Idse + Wr * LAMpm;
                    Vqse_Ref = Vqse_Ref_Fb + Vqse_Ref_Ff;  
                end
    
                Vdss_Ref =  Cos_Thetar_Adv * Vdse_Ref - Sin_Thetar_Adv * Vqse_Ref;
                Vqss_Ref =  Sin_Thetar_Adv * Vdse_Ref + Cos_Thetar_Adv * Vqse_Ref;
        
                Vas_Ref = Vdss_Ref;
                Vbs_Ref = -0.5*(Vdss_Ref - SQRT3*Vqss_Ref);
                Vcs_Ref = -0.5*(Vdss_Ref + SQRT3*Vqss_Ref);
    
            %%%%%%%%%%%%%%%%%%%
            %%% Current MPC %%%
            %%%%%%%%%%%%%%%%%%%
            elseif(Current_Control_mode==2)
    
                if(Velocity_Control_mode~=1)
                    Iqse_Ref = Ref;
                end
    
                Cos_Thetar_Adv = cos(Thetar_Adv);
                Sin_Thetar_Adv = sin(Thetar_Adv);
    
                Idss = 2. * INV3 * (Ias -0.5* (Ibs + Ics));
                Iqss = INV_SQRT3 * (Ibs - Ics);
        
                Idse =  Cos_Thetar * Idss + Sin_Thetar * Iqss;
                Iqse = -Sin_Thetar * Idss + Cos_Thetar * Iqss;
    
                r = [Idse_Ref; Iqse_Ref];
                Am = [-Rs/Lds, Wr*(Lqs/Lds); -Wr*(Lds/Lqs), -Rs/Lqs];
                Bm = [1/Lds, 0; 0, 1/Lqs];
                Cm = [1, 0; 0, 1];
                Dm = [0, 0; 0, 0];
    
                d = -Wr*LAMpm;
    
                [Ad,Bd,Cd,Dd] = c2dm(Am,Bm,Cm,Dm,Delta_t);
                [Phi_Phi, Phi_F, Phi_R, A_e, B_e, C_e] = mpcgain(Ad,Bd,Cd,Nc,Np);
                
                xm = [Idse; Iqse];
                delta_xm = xm - [Idse_prev; Iqse_prev]; 
                Y = Cd*xm;
                Xf = [delta_xm; Y];
    
                Delta_U = pinv(Phi_Phi+Bar_R)*(Phi_R*r-Phi_F*Xf);
                delta_u = Delta_U(1:2 , :);
                u = u + delta_u;
    
                Vdse_Ref = u(1);
                Vqse_Ref = u(2)-d;
    
                Vdss_Ref =  Cos_Thetar_Adv * Vdse_Ref - Sin_Thetar_Adv * Vqse_Ref;
                Vqss_Ref =  Sin_Thetar_Adv * Vdse_Ref + Cos_Thetar_Adv * Vqse_Ref;
        
                Vas_Ref = Vdss_Ref;
                Vbs_Ref = -0.5*(Vdss_Ref - SQRT3*Vqss_Ref);
                Vcs_Ref = -0.5*(Vdss_Ref + SQRT3*Vqss_Ref);
            end
    
            if(Vas_Ref > Vbs_Ref)
                Vmax = Vas_Ref;
                Vmin = Vbs_Ref;
            else
                Vmax = Vbs_Ref;
                Vmin = Vas_Ref;
            end
            if(Vcs_Ref > Vmax)
                Vmax = Vcs_Ref;
            end
            if(Vcs_Ref < Vmin)	
                Vmin = Vcs_Ref;
            end
    
            Vsn = -0.5*(Vmax + Vmin);
            
            Van_Ref = Vas_Ref + Vsn;
            Vbn_Ref = Vbs_Ref + Vsn;
            Vcn_Ref = Vcs_Ref + Vsn;    
    
            Duty_Van_Ref = Van_Ref / (0.5*Vdc);
            Duty_Vbn_Ref = Vbn_Ref / (0.5*Vdc);
            Duty_Vcn_Ref = Vcn_Ref / (0.5*Vdc);
        end
        
        %-- Inverter Update --%
        if(I_Carrier >= 1)
            I_Carrier = 1;
            I_Del_Carrier = -4/Sampling_Step;
        elseif(I_Carrier <= -1)
            I_Carrier = -1;
            I_Del_Carrier = 4/Sampling_Step;
        end
        I_Carrier = I_Carrier + I_Del_Carrier;
        if(Duty_Van_Ref >= I_Carrier)
            M_Vas = 0.5*I_Vdc;
        else
            M_Vas = -0.5*I_Vdc;
        end
        if(Duty_Vbn_Ref >= I_Carrier)
            M_Vbs = 0.5*I_Vdc;
        else
            M_Vbs = -0.5*I_Vdc;
        end
        if(Duty_Vcn_Ref >= I_Carrier)
            M_Vcs = 0.5*I_Vdc;
        else
            M_Vcs = -0.5*I_Vdc;
        end
    
        %-- PMSM Update --%
        M_Cos_Thetar = cos(M_Thetar);
        M_Sin_Thetar = sin(M_Thetar);
    
        M_Vdss = 2. * INV3 * (M_Vas -0.5* (M_Vbs + M_Vcs));
        M_Vqss = INV_SQRT3 * (M_Vbs - M_Vcs);
    
        M_Vdse =  M_Cos_Thetar * M_Vdss + M_Sin_Thetar * M_Vqss;
        M_Vqse = -M_Sin_Thetar * M_Vdss + M_Cos_Thetar * M_Vqss;
    
        M_Delta_Idse = Tstep * (1/M_Lds) * (0.5*(M_Vdse_prev+M_Vdse) - M_Rs*M_Idse + M_Wr*M_Lqs*M_Iqse);
        M_Idse = M_Idse + M_Delta_Idse;
    
        M_Delta_Iqse = Tstep * (1/M_Lqs) * (0.5*(M_Vqse_prev+M_Vqse) - M_Rs*M_Iqse - M_Wr*(M_Lds*M_Idse + M_LAMpm));
        M_Iqse = M_Iqse + M_Delta_Iqse;
    
        M_Te = 1.5*M_PolePair*(M_LAMpm*M_Iqse + (M_Lds-M_Lqs)*M_Idse*M_Iqse);
    
        M_Idss = M_Cos_Thetar * M_Idse - M_Sin_Thetar * M_Iqse;
        M_Iqss = M_Sin_Thetar * M_Idse + M_Cos_Thetar * M_Iqse;
    
        M_Ias = M_Idss;
        M_Ibs = -0.5 * (M_Idss - SQRT3 * M_Iqss); 
        M_Ics = -0.5 * (M_Idss + SQRT3 * M_Iqss);
    
        M_Delta_Wrm = Tstep * (1/M_Jm) * (0.5*(M_Te_prev+M_Te) - M_Bm*M_Wrm - 0.5*(M_TL_prev+M_TL));
        M_Wrm = M_Wrm + M_Delta_Wrm;
        M_Wr = M_PolePair * M_Wrm;
        M_Thetarm = M_Thetarm + Tstep * M_Wrm;    
        M_Thetarm = BOUND_PI(M_Thetarm);
        M_Thetar = M_PolePair * M_Thetarm;
        M_Thetar = BOUND_PI(M_Thetar);
    
        %-- Previous Value Upadate --%%
        M_Te_prev = M_Te;
        M_Vdse_prev = M_Vdse;
        M_Vqse_prev = M_Vqse;
        M_TL_prev = M_TL;
        Idse_prev = Idse; 
        Iqse_prev = Iqse;
    
        if (mod(Count,1000) == 0)
            tic
            % --Data Send--%
            % 1:Id(Measured), 2:Iq(Measured), 3:Id(Real),  4:Iq(Real),   
            % 5:I_A-phase,    6:I_B-phase,    7:I_C-phase, 8:Simulation Time.
            Current_pubMsg.data = [Idse, Iqse, M_Idse, M_Iqse, M_Ias, M_Ibs, M_Ics, SimulationTime]; 
            send(Current_pub,Current_pubMsg)
        
            % 1:V_A-phase, 2:V_B-phase, 3:V_C-phase, 4:Simulation Time.
            Voltage_pubMsg.data = [Van_Ref, Vbn_Ref, Vcn_Ref, SimulationTime]; 
            send(Voltage_pub,Voltage_pubMsg)
        
            % 1:Reference Torque, 2:Real Torque, 3:Load Torque, 4:Simulation Time.
            Torque_pubMsg.data = [Iqse_Ref*1.5*PolePair*LAMpm, M_Te, M_TL, SimulationTime];
            send(Torque_pub,Torque_pubMsg)
        
            % 1:Reference Speed, 2:Real Speed, 3:Simulation Time.
            Velocity_pubMsg.data = [Wrm_Ref*Rm2rpm, M_Wrm*Rm2rpm, SimulationTime]; 
            send(Velocity_pub,Velocity_pubMsg)
        
            % 1:Reference Angle, 2:Real Angle, 3:Simulation Time.
            Angle_pubMsg.data = [Thetarm_Ref/(pi/180), M_Thetarm/(pi/180), SimulationTime];
            send(Angle_pub,Angle_pubMsg)
            toc
        end
    
        % Count Update
        Count = Count +1;
    end
    pause(0.0000000001)
end
