function [properties,dyn,con, info] = Moli_heat_exchangers()
%%
if nargout <= 1
    properties.nx  = 2;
    properties.nu = 2;
    properties.ny = 1;
    return;
end
% This code appears to define properties of a system based on the number of
% output arguments expected from a function. If only one output argument is 
% expected, it sets specific properties (nx, nu, ny) and returns. Otherwise,
% it continues with further code execution.

%% Parameters of the system
parameters.V1  = 3;
parameters.V2 = 1;
parameters.F1 = 15;
parameters.F2 = 10;
parameters.q = 0.1;
parameters.rho = 998;
parameters.cp = 4.219;
parameters.alpha1 = 120; 
parameters.alpha2 = 20;  
parameters.thetav = 353;
%% steady state
parameters.thetap1s = 303; % For calculating steady state temperatures
parameters.thetap2s = 293; % For calculating steady state temperatures
parameters.theta1s = (parameters.q*parameters.rho*parameters.cp*parameters.thetav ...
    + parameters.alpha1*parameters.F1*parameters.thetap1s)/ ...
(parameters.alpha1*parameters.F1 + parameters.q*parameters.rho*parameters.cp);...
parameters.theta2s = (parameters.q*parameters.rho*parameters.cp* ...
    parameters.theta1s + parameters.alpha2*parameters.F2*parameters.thetap2s)/...
(parameters.alpha2*parameters.F2 + parameters.q*parameters.rho*parameters.cp);

parameters.xs = [parameters.theta1s; parameters.theta2s];
parameters.us = [parameters.thetap1s;parameters.thetap2s];

%%
data = { 
    string('q'),   string('Flow rate'), 1,   string('m^3/s');
    'F1', 'Cross-sectional area of first heat exchanger', 15, 'm^2';
    'F2', 'Cross-sectional area of second heat exchanger', 10, 'm^2';
    'V1', 'Volume of first heat exchanger', 3, 'm^3';
    'V2', 'Volume of second heat exchanger', 1, 'm^3';
    'ρ', 'Density of liquid', 998, 'kg/m^3';
    'cp', 'Specific heat capacity', 4.219, 'J.kg^-1.K^-1';
    'α1', 'Heat transfer coefficient 1', 120, 'W.m^−2.K^−1';
    'α2', 'Heat transfer coefficient 2', 20, 'W.m^−2.K^−1';
    'ϑ1', 'Temperature in first heat exhanger', NaN, 'K';
    'ϑ2', 'Temperature in second heat exhanger', NaN, 'K';
    'ϑp1', 'Temperature of the heating medium that is injected into first jacket', NaN, 'K';
    'ϑp2', 'Temperature of the heating medium that is injected into second jacket', NaN, 'K'};

T1 = cell2table(data, 'VariableNames', {'Symbol', 'Description', 'Value', 'Unit'});
%disp(T1)

states = {'ϑ1'; 'ϑ2'};  states = string(states);
inputs = {'ϑp1'; 'ϑp2'}; inputs = string(inputs);
outputs = {'ϑ2'}; outputs = string(outputs);

T2 = table(states', inputs', outputs');
T2.Properties.VariableNames = {'States', 'Inputs', 'Outputs'};
%disp(T2)

%% Dynamics
% This code defines the dynamics of a system, specifically a linear
% time-invariant (LTI) system.

% State transition matrix. Describes how the state of the system evolves over time.
dyn.sc.dif.A(1,1) = - (parameters.alpha1*parameters.F1 + parameters.q*parameters.rho*parameters.cp)/(parameters.V1*parameters.rho*parameters.cp);
dyn.sc.dif.A(1,2) = 0;
dyn.sc.dif.A(2,1) =  parameters.q*parameters.rho*parameters.cp/(parameters.V2*parameters.rho*parameters.cp);
dyn.sc.dif.A(2,2) =  -(parameters.alpha2*parameters.F2 + parameters.q*parameters.rho*parameters.cp)/(parameters.V2*parameters.rho*parameters.cp);
% Input matrix. Describes how the inputs affect the state evolution.
dyn.sc.dif.B(1,2) = 0;
dyn.sc.dif.B(1,1) = parameters.alpha1*parameters.F1/(parameters.V1*parameters.rho*parameters.cp);
dyn.sc.dif.B(2,1) = 0;
dyn.sc.dif.B(2,2) = parameters.alpha2*parameters.F2/(parameters.V2*parameters.rho*parameters.cp);
% Output matrix. Describes how the states are mapped to the outputs.
dyn.sc.dif.C = [0 1];
% Feedthrough matrix. Describes direct feedthrough from inputs to outputs.
dyn.sc.dif.D = [];

% This is an optional parameter representing any external disturbances 
% acting on the system.
dyn.sc.dif.f=[];
% Another optional parameter representing any external disturbances acting
% on the output.
dyn.sc.dif.e=[];
% Similarly, dyn.sd.dif.f and dyn.sd.dif.e represent the same parameters 
% for the system after it has been converted to discrete-time(sd).

% Sampling time 
dyn.Ts= [1.4218];

% The continuous-time state-space model defined by matrices dyn.sc.dif.A,
% dyn.sc.dif.B, dyn.sc.dif.C, and dyn.sc.dif.D is converted to discrete-time
% using the function c2d.
dyn.sysc=ss(dyn.sc.dif.A,dyn.sc.dif.B,dyn.sc.dif.C,dyn.sc.dif.D);
dyn.sysd=c2d(dyn.sysc,dyn.Ts );

% The resulting discrete-time state-space model is stored in the matrices
% dyn.sd.dif.A, dyn.sd.dif.B, dyn.sd.dif.C, and dyn.sd.dif.D. 
dyn.sd.dif.A=dyn.sysd.A;
dyn.sd.dif.B=dyn.sysd.B;
dyn.sd.dif.C=dyn.sysd.C;
dyn.sd.dif.D=dyn.sysd.D;

dyn.sd.dif.f=[];
dyn.sd.dif.e=[];

% dyn.Q, dyn.R, dyn.S: These are matrices used in the calculation of the 
% cost function for optimal control. They define the importance of minimizing
% the state, input, and output deviations from desired values, respectively.
dyn.Q=[];
dyn.R=[];
dyn.S=[];

% dyn.gu, dyn.gy: These are optional parameters representing any additional 
% terms in the cost function related to inputs and outputs.
dyn.gu=[];
dyn.gy=[];

dyn.params = parameters;
dyn.paramsT1 = T1;
dyn.paramsT2 = T2;

%% Constraints
% Constraints on the states to the system.
con.x.max=[450; 450];
con.x.min=[0;0];
% Constraints on the inputs to the system.
con.u.max=[450; 450];
con.u.min=[0;0];
% Constraints on the outputs of the system.
con.y.min=[];
con.y.max=[];

%% Properties
% Check observability
obscon = rank(obsv(dyn.sc.dif.A, dyn.sc.dif.C)) == size(dyn.sc.dif.A, 1);
if obscon
    fprintf('The system is fully observable!\n');
    properties.observability = true;
else
    fprintf('The system is NOT fully observable!\n');
    properties.observability = false;
end

% Check stability
lambda_sys = eig(dyn.sc.dif.A);
stabcon = all(lambda_sys < 0);
if stabcon
    fprintf('The system is stable!\n');
    properties.stability = true;
else
    fprintf('The system is NOT stable!\n');
    properties.stability = false;
end

% Check controllability
C_0 = ctrb(dyn.sc.dif.A, dyn.sc.dif.B);
nx_ctrb = rank(C_0);
if nx_ctrb == size(dyn.sc.dif.A, 2)
    fprintf('The system is fully controllable!\n');
    properties.controllability = true;
else
    fprintf('The system is NOT fully controllable!\n');
    properties.controllability = false;
end

[properties.nx, properties.nu] = size(dyn.sc.dif.B);
properties.ny = size(dyn.sc.dif.C,1);

%% Info
info.source= [''];
info.text = [
    'Investigating a series configuration of two jacketed heat exchangers, ' ...
    'the objective is to regulate the temperature ϑ2(t) in the second ' ...
    'heat exchanger through adjustments in the temperatures of the heating ' ...
    'mediums introduced into the jackets ϑp1(t) and ϑp2(t). ' ...
    'The model was created by Bc. Branislav Daráš'];

temp = which('Moli_heat_exchangers');
temp = split(temp,'Moli_heat_exchangers.m');
PathModels = temp{1};
last_backslash_index = find(PathModels == '\', 1, 'last');
PathModels = PathModels(1:last_backslash_index-1);

image_path = [PathModels , filesep , 'image', filesep , 'exchangers.jpg'];
info.image_data = imread(image_path);
info.image= imshow(info.image_data);
end