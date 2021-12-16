clear all
clc

testNode = ros2node("/matlab_node", 0);   % node initialization
pause(1);

field1 = 'label';  value1 = 'PMSM_data';
field2 = 'size';  value2 = uint32(0);
field3 = 'stride';  value3 = uint32(0);
s = struct(field1, value1, field2, value2, field3, value3);

% Publisher
Current_pub = ros2publisher(testNode,"/Current","std_msgs/Float64MultiArray");
Current_pubMsg = ros2message("std_msgs/Float64MultiArray");
Current_pubMsg.layout.dim = s;
Current_pubMsg.layout.data_offset = uint32(0);

Voltage_pub = ros2publisher(testNode,"/Voltage","std_msgs/Float64MultiArray");
Voltage_pubMsg = ros2message("std_msgs/Float64MultiArray");
Voltage_pubMsg.layout.dim = s;
Voltage_pubMsg.layout.data_offset = uint32(0);

Torque_pub = ros2publisher(testNode,"/Torque","std_msgs/Float64MultiArray");
Torque_pubMsg = ros2message("std_msgs/Float64MultiArray");
Torque_pubMsg.layout.dim = s;
Torque_pubMsg.layout.data_offset = uint32(0);

Velocity_pub = ros2publisher(testNode,"/Velocity","std_msgs/Float64MultiArray");
Velocity_pubMsg = ros2message("std_msgs/Float64MultiArray");
Velocity_pubMsg.layout.dim = s;
Velocity_pubMsg.layout.data_offset = uint32(0);

Angle_pub = ros2publisher(testNode,"/Angle","std_msgs/Float64MultiArray");
Angle_pubMsg = ros2message("std_msgs/Float64MultiArray");
Angel_pubMsg.layout.dim = s;
Angel_pubMsg.layout.data_offset = uint32(0);

%----------------------------%
%-- Simulation setup --------%
%----------------------------%

%Tstep = 50e-9; % Original Tstep
Tstep = 10e-6; 

%Tsw = 41e-6;    % Original Current control step
%Tsw_vc = 41e-5; % Original Velocity control step
%Tsw_pc = 41e-4; % Original Position control step
Tsw = 0.1e-3;
Tsw_vc = 1e-3;            
Tsw_pc = 10e-3;            

% Sampling_Step = floor(Tsw/Tstep);
% Sampling_Step_vc = floor(Tsw_vc/Tstep);
% Sampling_Step_pc = floor(Tsw_pc/Tstep);

Sampling_Step = 10;
Sampling_Step_vc = 100;
Sampling_Step_pc = 1000;

TWOPI = 2*pi;
PIover6 = pi/6;
SQRT2 = sqrt(2);
SQRT3 = sqrt(3);
INV_SQRT3 = 1/sqrt(3);
INV_SQRT2 = 1/sqrt(2);
INV3 = 1/3;
Rpm2rm = 2*pi/60;
Rm2rpm = 60/(2*pi);

Control_mode = 3;          % [1:Current, 2:Velocity, 3: Position]

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
M_Te = 0; % torque
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

%-----------------------------%
%-- Inverter variables -------%
%-----------------------------%
I_Carrier = 1;
I_Del_Carrier = -1;

%-------------------------%
%-- Inverter parameters --%
%-------------------------%
I_Vdc = 48;

%% Controler setting
%--------------------------%
%-- Controller variables --%
%--------------------------%
Thetar = 0;
Thetarm = 0;
Wr = 0;
Wrpm_Ref = 0;
Wrm_Ref = 0;
Wrm = 0;
Wrm_Integ = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Thetar_Ref = 0;
Thetarm_Ref = 0; 
Wrm_Integ_Ref = 0;
Wrm_Ref_Sat = 0;
Wrm_Ref_Fb = 0;
Wrm_Ref_Anti = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

Ref = 0;
Count = 0;

%-- Simulation main loop --%
while(1)
    tic
    SimulationTime = Count*Tstep;
    
    %-- Input variables setup --%
    % Rotating speed profile
    if(SimulationTime > 0.01)
        %Ref = 0.1/(1.5*PolePair*LAMpm); % [A] = [Nm/(1.5*PolePair*LAMpm)]
        Ref = 5*(pi/180);               % [rad]=[deg*(pi/180)]
        %Ref = 1000;                      % [rpm]
    end
    % Load torque profile
    if(SimulationTime > 0.03)
        %M_TL = Ref*(1.5*PolePair*LAMpm);
        M_TL = 0;
    end  

    %-- Controller --%  
    if(mod(Count,Sampling_Step) == 0) 

        %-- Position calculation--%
        Err_Thetarm = M_Thetarm - Thetarm;
        Err_Thetarm = BOUND_PI(Err_Thetarm);
        Wrm_Integ = Wrm_Integ + KiT_PLL * Err_Thetarm;
        Wrm = Wrm_Integ + Kp_PLL * Err_Thetarm;
   
        Thetarm = Thetarm + Tsw * Wrm;
        Thetarm = BOUND_PI(Thetarm);

        Wr = PolePair * Wrm;
        Thetar = Thetar + Tsw * Wr;
        Thetar = BOUND_PI(Thetar);
        Thetar_Adv = Thetar + Wr * Tsw;

        %-- Position Controller --% 
        if(Control_mode==3 && mod(Count,Sampling_Step_pc) == 0)
            Thetarm_Ref = Ref;
            disp(Thetarm_Ref)
            Err_Thetarm = Thetarm_Ref - Thetarm;
            Err_Thetarm = BOUND_PI(Err_Thetarm);
            Wrm_Integ_Ref = Wrm_Integ_Ref + KiT_pc * (Err_Thetarm - (Kp_Anti_pc*Wrm_Ref_Anti)); % anti-wind up
            Wrm_Ref_Fb = Wrm_Integ_Ref + Kp_pc * Err_Thetarm;                                        % PI controller
    
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

        %-- Velocity Controller --%
        if((Control_mode==2 || Control_mode==3) && mod(Count,Sampling_Step_vc) == 0)
            
            if(Control_mode==2)
                Wrm_Ref = Ref* Rpm2rm;
            end

            Err_Wrm = Wrm_Ref - Wrm;
            Te_Integ_Ref = Te_Integ_Ref + KiT_vc * (Err_Wrm - (Kp_Anti_vc*Te_Ref_Anti)); % anti-wind up
            % Te_Ref(i) = Te_Integ_Ref(i) + Kp_vc * Err_Wrm;                                        % PI controller
            Te_Ref = Te_Integ_Ref - Kp_vc * Wrm;                                           % IP controller
            Iqse_Ref_Fb = Te_Ref / (1.5*PolePair*LAMpm);
            
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

        %-- Current Controller --%
        if(Control_mode==1)
            Iqse_Ref = Ref;
        end

        Ias = M_Ias;
        Ibs = M_Ibs;
        Ics = M_Ics;

        Cos_Thetar = cos(Thetar);
        Sin_Thetar = sin(Thetar);

        Cos_Thetar_Adv = cos(Thetar_Adv);
        Sin_Thetar_Adv = sin(Thetar_Adv);

        Idss = 2. * INV3 * (Ias -0.5* (Ibs + Ics));
        Iqss = INV_SQRT3 * (Ibs - Ics);

        Idse =  Cos_Thetar * Idss + Sin_Thetar * Iqss;
        Iqse = -Sin_Thetar * Idss + Cos_Thetar * Iqss;

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

        Vdss_Ref =  Cos_Thetar_Adv * Vdse_Ref - Sin_Thetar_Adv * Vqse_Ref;
        Vqss_Ref =  Sin_Thetar_Adv * Vdse_Ref + Cos_Thetar_Adv * Vqse_Ref;

        Vas_Ref = Vdss_Ref;
        Vbs_Ref = -0.5*(Vdss_Ref - SQRT3*Vqss_Ref);
        Vcs_Ref = -0.5*(Vdss_Ref + SQRT3*Vqss_Ref);

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

    % Previous Value Upadate
    M_Te_prev = M_Te;
    M_Vdse_prev = M_Vdse;
    M_Vqse_prev = M_Vqse;
    M_TL_prev = M_TL;

    % --Data Send--%
    
    % 1:Id(Measured), 2:Iq(Measured), 3:Id(Real), 4:Iq(Real), 5:I_A-phase, 6:I_B-phase, 7:I_C-phase 
    % 8:Simulation Time
    Current_pubMsg.data = [Idse, Iqse, M_Idse, M_Iqse, M_Ias, M_Ibs, M_Ics, SimulationTime]; 
    send(Current_pub,Current_pubMsg)

    % 1:V_A-phase, 2:V_B-phase, 3:V_C-phase, 4:Simulation Time
    Voltage_pubMsg.data = [Van_Ref, Vbn_Ref, Vcn_Ref, SimulationTime]; 
    send(Voltage_pub,Voltage_pubMsg)

    % 1:Reference Torque, 2:Real Torque, 3:Load Torque, 4:Simulation Time
    Torque_pubMsg.data = [Iqse_Ref*1.5*PolePair*LAMpm, M_Te, M_TL, SimulationTime];
    send(Torque_pub,Torque_pubMsg)

    % 1:Reference Speed, 2:Real Speed, 3:Simulation Time
    Velocity_pubMsg.data = [Wrm_Ref*Rm2rpm, M_Wrm*Rm2rpm, SimulationTime]; 
    send(Velocity_pub,Velocity_pubMsg)

    % 1:Reference Angle, 2:Real Angle, 3:Simulation Time
    Angle_pubMsg.data = [Thetarm_Ref/(pi/180), M_Thetarm/(pi/180), SimulationTime];
    send(Angle_pub,Angle_pubMsg)

    % Count Update
    Count = Count +1;
    toc
end