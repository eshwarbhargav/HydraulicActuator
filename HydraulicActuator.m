%% Coded for the fulfilment of Master's Degree at Politecnico Di Milano
% Author:: Eshwar Bhargav Bhupanam
% Course:: Modeling and Simulation of Aerospace Systems
% Topic:: Hydraulic Actuator System
% Year:: 2020-2021

%% Initializing....
clear vars; clc; close all

%% Hydraulic Actuator System - Physical model inputs
data.fluid.rho = 890;    % Fluid - Skydrol's density in [Kg/m^3] 

data.accumulator.V_N2 = 0.01;  % Volume of nitrogen @ t = -inf in [m^3]
data.accumulator.P_N2 = 2.5*1e+6;    % Pressure of nitrogen @ t = -inf in [Pa]
data.accumulator.P0 = 21*1e+6;   % Initial pressure @ t = 0 in [Pa]
data.accumulator.gamma = 1.2;    % Adiabatic exponent 
data.accumulator.V0 = data.accumulator.V_N2*(data.accumulator.P_N2/data.accumulator.P0);
data.accumulator.V_acc0 = data.accumulator.V_N2 - data.accumulator.V0;

data.delivery.Ka = 1.12;  % coeff of pressure drop at accumulator outlet
data.delivery.Kcv = 2;    % coeff of pressure drop across the check valve
data.delivery.D23 = 0.018;   % Diameter of delivery line in [m]
data.delivery.L23 = 2;   % Length of delivery line in [m]
data.delivery.f23 = 0.032;   % Friction factor
data.delivery.A = (pi/4)*data.delivery.D23^2; % Area of delivery in [m^2]

data.distributor.Kd = 12;    % coeff of pressure drop across the distributor
data.distributor.D0 = 0.005; % Circular cross section diameter of distributor 
data.distributor.A0 = (pi/4)*data.distributor.D0^2;

data.actuator.Dc = 0.050;    % Diameter of cylinder in [m]
data.actuator.Ac = (pi/4)*data.actuator.Dc^2;
data.actuator.Dr = 0.022;    % Diameter of rod in [m]
data.actuator.Ae = (pi/4)*(data.actuator.Dc^2-data.actuator.Dr^2);
data.actuator.m = 2;         % Mass of the piston in [Kg]
data.actuator.stroke_max = 0.2;   % Maximum stroke length of the piston in [m]
data.actuator.F0 = 1000;     % Force/load in [N]
data.actuator.K = 120000;    % Force/load acting per unit length in [N/m]

data.reverse.Dr = 0.018;     % Diameter of return line in [m]
data.reverse.L67 = 15;       % Length of return line in [m]
data.reverse.f67 = 0.035;    % Friction factor
data.reverse.A = (pi/4)*data.reverse.Dr^2;

data.tank.P_tank0 = 0.1*1e+6;    % Initial pressure @ t = 0 in [Pa]
data.tank.V_tank0 = 0.001;       % Initial volume @ t = 0 in [m^3]
data.tank.Kt = 1.12;

data.time.ti = 0; data.par.x0 = 0; data.par.v0 = 0; % Initial conditions
data.time.t1 = 1; data.time.t2 = 1.5; data.time.tf = 3;    % Time intervals w.r.t 'z'
tspan = [data.time.ti data.time.tf];
y0 = [data.accumulator.V_acc0 data.tank.V_tank0 data.par.x0 data.par.v0];

%% Events - Setup
options=odeset('event',@stroke_max);
[tevent,~]=ode15s(@hydraulicsystem,tspan,y0,options,data); % Computes the event
t_event = tevent(end);
[t_1,output_1]=ode15s(@hydraulicsystem,[data.time.ti t_event],y0,odeset,data); % from t -> tevent
y0 = [output_1(end,1) output_1(end,2) 0.2 0];
[t_2,output_2]=ode15s(@hydraulicsystem,[t_event data.time.tf],y0,odeset,data); % tevent -> tfinal

%% System response in terms of Pressure and Flow-rate
params=[output_1;output_2]; t=[t_1;t_2];
pressflow=zeros(length(t),18);
for i=1:length(t)
    [~,pressflow(i,:)]=hydraulicsystem(t(i),params(i,:),data); %find pressures and flows given the state
end

%% Plotting
figure()
subplot(2,1,1)
plot(t, params(:,1)); % Volume of accumulator
grid on; grid minor;
legend({'Volume of acumulator'}, 'Interpreter', 'latex');
ylabel('Volume $[Kg/m^3]$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex')

subplot(2,1,2)
plot(t,params(:,2)); % Volume of tank
grid on; grid minor;
legend({'Volume of tank'}, 'Interpreter', 'latex');
ylabel('Volume $[Kg/m^3]$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex')

figure()
plot(t,params(:,3)); hold on; grid on; % Piston position
plot(t,params(:,4)); % piston velocity
legend({'Piston position', 'Piston velocity'}, 'Interpreter', 'latex');
ylabel('Position [m], Velocity [m/s]', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex')

figure()
subplot(3,1,1)
plot(t,pressflow(:,3), t,pressflow(:,5), t,pressflow(:,7), t,pressflow(:,9)) % Delivery line pressures
grid on; grid minor;
title('Pressure history along the delivery line','Interpreter','latex');
legend({'PA','P1','P2','P3'}, 'Interpreter', 'latex');
ylabel('Pressure [Pa]', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex')

subplot(3,1,2)
plot(t,pressflow(:,11), t,pressflow(:,13)) % Actuator pressures
grid on; grid minor;
title('Pressure history along the actuator','Interpreter','latex');
legend({'P4','P5'}, 'Interpreter', 'latex');
ylabel('Pressure [Pa]', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex')

subplot(3,1,3)
plot(t,pressflow(:,15), t,pressflow(:,17)) % Return line pressures
grid on; grid minor;
title('Pressure history along the return line','Interpreter','latex');
legend({'P6','P7'}, 'Interpreter', 'latex');
ylabel('Pressure [Pa]', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex')

figure()
plot(t,pressflow(:,8), t,pressflow(:,16)) % Flow rates
grid on;
legend({'Delivery flow rate', 'Return flow rate'}, 'Interpreter', 'latex');
ylabel('$Q [m^3/s]$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex')

%% Functions

function [value, isterminal, direction]=stroke_max(~,y0,data)
xmax=data.actuator.stroke_max;

value=xmax-y0(3); % Distance of the pistion to reach the maximum stroke
isterminal = 1 ;
direction = -1 ;
end

function [f, output] = hydraulicsystem(t,y,data)
accumulator.V_acc=y(1);
tank.V_tank = y(2);
par.x=y(3); 
par.v=y(4); 
data.accumulator.V0=data.accumulator.P_N2*data.accumulator.V_N2/data.accumulator.P0;

flow.Q4 = data.actuator.Ac*par.v;
flow.Q5 = data.actuator.Ae*par.v;

tol = 1e-8;
if t<=data.time.t1
    m=0;
    theta=2*acos(1-2*abs(m));
    av=0.5*(data.distributor.D0/2)^2*(theta-sin(theta));

    flow.Q3=0;
    flow.Q6=0;
    pressure.P5=data.tank.P_tank0;
    pressure.P4=(pressure.P5*data.actuator.Ae+data.actuator.F0)/data.actuator.Ac;
elseif data.time.t1<t && t<data.time.t2
    m=(t-data.time.t1)/(data.time.t2-data.time.t1);
    theta=2*acos(1-2*abs(m));
    av=0.5*(data.distributor.D0/2)^2*(theta-sin(theta));

    flow.Q3=flow.Q4;
    flow.Q6=flow.Q5;
else
    m=1;
    theta=2*acos(1-2*abs(m));
    av=0.5*(data.distributor.D0/2)^2*(theta-sin(theta));

    flow.Q3=flow.Q4;
    flow.Q6=flow.Q5;
end
flow.Q_acc=flow.Q3;
flow.QA=flow.Q3; 
flow.Q1=flow.Q3;
flow.Q2=flow.Q3; 
flow.Q7=flow.Q6; 
flow.Q_T=flow.Q7;

pressure.P_acc=data.accumulator.P0*(data.accumulator.V0/(data.accumulator.V_N2-accumulator.V_acc))^(data.accumulator.gamma);
pressure.PA=pressure.P_acc-0.5*data.delivery.Ka*data.fluid.rho*(flow.QA/data.delivery.A)*abs(flow.QA/data.delivery.A);
pressure.P1=pressure.PA;
pressure.P2=pressure.P1-0.5*data.delivery.Kcv*data.fluid.rho*(flow.Q2/data.delivery.A)*abs(flow.Q2/data.delivery.A);
pressure.P3=pressure.P2-0.5*data.delivery.f23*data.fluid.rho*data.delivery.L23*(flow.Q3/data.delivery.A)*abs(flow.Q3/data.delivery.A)/data.delivery.D23;
pressure.P7=data.tank.P_tank0+0.5*data.tank.Kt*data.fluid.rho*(flow.Q7/data.reverse.A)*abs(flow.Q7/data.reverse.A);
pressure.P6=pressure.P7+0.5*data.reverse.f67*data.fluid.rho*data.reverse.L67*(flow.Q6/data.reverse.A)*abs(flow.Q6/data.reverse.A)/data.reverse.Dr;

% Check point for tolerance 
if av<tol
    av=tol;
end

if m~=0
    pressure.P5=pressure.P6+0.5*data.distributor.Kd*data.fluid.rho*(flow.Q5/av)*abs(flow.Q5/av);
    pressure.P4=pressure.P3-0.5*data.distributor.Kd*data.fluid.rho*(flow.Q4/av)*abs(flow.Q4/av);
end

% Piston position and velocity
dv=(pressure.P4*data.actuator.Ac-pressure.P5*data.actuator.Ae-data.actuator.F0-data.actuator.K*par.x)/data.actuator.m;
data.accumulator.dVacc=-flow.Q_acc;
data.tank.dVtank=flow.Q_T;
dx=par.v;

% Constriants
if par.x>=data.actuator.stroke_max && dv>0
    dv=0;
end

f = [data.accumulator.dVacc;data.tank.dVtank;dx;dv];
output = [pressure.P_acc flow.Q_acc pressure.PA flow.QA pressure.P1 flow.Q1 pressure.P2 flow.Q2 pressure.P3 flow.Q3 pressure.P4 flow.Q4 pressure.P5 flow.Q5 pressure.P6...
    flow.Q6 pressure.P7 flow.Q_T];
end
