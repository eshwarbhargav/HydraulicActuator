%% Coded for the fulfilment of Master's Degree at Politecnico Di Milano
% Author:: Eshwar Bhargav Bhupanam
% Course:: Modeling and Simulation of Aerospace Systems
% Topic:: Electro Hydraulic Actuator System
% Year:: 2020-2021


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   MAIN SCRIPT                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% This is the Main script for the project of MSAS simulating an
% Electro-hydraulic actuator. Please execute it by sections.
% 
% The first section "Settings" allows to modify the simulation as the user
% wants between the "No load" and the "loaded" cases, the "step" and
% "ramps" responses, the controller and the final simulation time.
%
% The bulk of the integration is performed in the "Integration.m" script.
% If wanting to change the systems physical parameters or initial
% conditions, it has to be changed there.
%
% It is done this way because the script "Integration.m" is also used in
% other ocasions, such as in the "Cost_Function.m"


%% Initializing....
clear vars; clc; close all

%% Simulation settings
mode = 2; % Set to 1 for step reference response
          % Set to 2 for ramps reference response

Loaded = 1; % Set to 0 for the "No load" case
            % Set to 1 for the "loaded" case

% Electronic Control Unit - Controller
ECU.Kp = 1611.123741598567;	% Proportional constant of the controller
ECU.Ki = 1590.695266165130; % Integral constant of the controller

% Simulation final time
tend = 40;

%% Electro Hydraulic Actuator System - Physical model inputs

acc_g = 9.81;           % Accleration due to gravity in [m/s^2]

% Motor
MOTOR.R = 1.5;          % Armature resistance in [Ohm]<--- if increased lower: eig(V)
MOTOR.L = 7.5*1e-3 ;    % Armature inductance in [H]
MOTOR.Ke = 0.2;         % Back EMF in [V/(rad/s)]<--- if increased lower: eig(V)

% Pump
PUMP.J = 1.2e-3;           % Total moment of inertia in [Kg-m^2]<--- if increased lower: eig(V) 
PUMP.B = 0.083;            % Viscous damping friction coefficient in [N-m/(rad/s)]   <--- if increased lower: eig(V)
PUMP.Kt = MOTOR.Ke;        % Torque constant in [N-m/A]

PUMP.V_g = 10e-6;          % Flow displacement in [m^3/rev]
PUMP.eta_hm = 0.85;        % Motor-Pump transmission efficiency in [-]

PUMP.Pmax = 21e6;          % Pump maximum pressure in [N/m^2]
PUMP.mu_fric_coeff = 0.1;  % Coulomb friction coefficient in [-] 
PUMP.m = 4;             % Pump mass in [Kg];
PUMP.l = 0.05;        % Pump shaft length in [m]
PUMP.M_fric = PUMP.mu_fric_coeff*(PUMP.m*acc_g)*PUMP.l; % Columb-type dry friction Moment/Torque in [N-m]

% Actuator
ACTUATOR.rho = 7850; % Stainless steel material density in [Kg/m^3]
ACTUATOR.E_mod = 210e9; % Elastic modulus of steel in [N/m^2]
ACTUATOR.mu_surface = 0.3; % Surface friction coefficient (Cylinder-ActuatorWalls) in [-]
ACTUATOR.B = 0; % Viscosity coefficient (HydraulicFluid-ActuatorWalls) in [-]

ACTUATOR.F_stall;   % Stall force on the piston in [N]
ACTUATOR.A_pistonRing = ACTUATOR.F_stall/PUMP.Pmax; % Area of the piston ring in [m^2]

ACTUATOR.x_stroke = 0.3; % Stroke distance (i.e., x_extended - x_retracted) in [m] 
ACTUATOR.stroke_tol = 0.02; % Relative tolerance to consider the piston is stroked in [-]
ACTUATOR.w_pistonCylinder = 0.02; % Width of the piston cylinder in [m]
ACTUATOR.r_pistonRod = 1e-2; % Radius of the piston rod in [m]
ACTUATOR.A_pistonRod = pi*ACTUATOR.r_pistonRod^2; % Area of the piston rod in [m^2]

ACTUATOR.V_piston = (ACTUATOR.A_pistonRing*ACTUATOR.w_pistonCylinder)+(ACTUATOR.A_pistonRod*(2*ACTUATOR.x_stroke)); % Volume of the moving part (Piston) in [m^3]
ACTUATOR.m_piston = ACTUATOR.rho*ACTUATOR.V_piston; % Mass of the moving part (Piston) in [Kg]

ACTUATOR.t = 1e-2; % Thickness of the actuator outer walls in [m]
ACTUATOR.gap_actuatorCylinder = 0.1e-3; % Annular gap between actuator wall and piston cylinder in [m]
ACTUATOR.D = 2*sqrt((ACTUATOR.area_pistonRing+A_pistonRod)/(pi)) + (2*ACTUATOR.gap_actuatorCylinder); % Internal diameter in [m]
ACTUATOR.beta        = (ACTUATOR.E_mod*ACTUATOR.t)/ACTUATOR.D; % Bulk modulus of the actuator's container [N/m^2]

% Hydraulic
HYDRAULIC.beta0         = 1200e6; % Bulk modulus coefficient [N/m^2]
HYDRAULIC.T_beta0       = 40; % Reference temperature [ºC]
HYDRAULIC.P_beta0       = 0.1e6; % Reference pressure [N/m^2]
HYDRAULIC.PI_beta0      = 260e6; % Pressure sensitivity coefficient [Pa]
HYDRAULIC.TI_beta0      = 192; % Temperature sensitivity coefficient [ºC]
HYDRAULIC.GasPercent    = 0.1; % Volume percentage of gas in hydraulic fluid [%]
HYDRAULIC.mu_skydrol    = 890*20e-6; % Dynamic/Absolute viscosity of Skydrol at aprox 15ºC [N-s/m^2]

% External solicitation
if Loaded == 0
    D.k_Fext    = 0; % Proportionality constant of the external load in [N/m] 
elseif Loaded == 1
    D.k_Fext    = 0.8*ACTUATOR.F_stall/ACTUATOR.x_stroke;
end