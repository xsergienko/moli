function [properties, dyn,con, info] = Moli_4_tanks()
%%
if nargout <= 1
    properties.nx  = 4;
    properties.nu = 2;
    properties.ny = 2;
    return;
end
% This code appears to define properties of a system based on the number of
% output arguments expected from a function. If only one output argument is 
% expected, it sets specific properties (nx, nu, ny) and returns. Otherwise,
% it continues with further code execution.

%% Parameters of the system
parameters.F1 = 1.2; parameters.F2 = 1.4; parameters.F3 = 1.2; parameters.F4 = 1.3;
parameters.k11 =  0.8; parameters.k22 = 0.8; parameters.k33 =  0.8; parameters.k44 =  0.8;
parameters.q01s = 1; parameters.q02s = 0.9; 
parameters.h2s = ((parameters.q01s+parameters.q02s)/parameters.k22)^2;
parameters.h1s=parameters.h2s+(parameters.q01s/parameters.k11)^2;
parameters.h3s=((parameters.k22*sqrt(parameters.h2s))/(2*parameters.k33))^2;
parameters.h4s=( ( ((parameters.k22*sqrt(parameters.h2s))/2)+(parameters.k33* ...
    sqrt(parameters.h3s)) )/parameters.k44)^2;

parameters.K1 = parameters.k11/(2*sqrt(parameters.h1s-parameters.h2s)); 
parameters.K2 = parameters.k22/(2*sqrt(parameters.h2s));
parameters.K3 = parameters.k33/(2*sqrt(parameters.h3s)); 
parameters.K4 = parameters.k44/(2*sqrt(parameters.h4s));
parameters.xs = [parameters.h1s;parameters.h2s;parameters.h3s;parameters.h4s];
parameters.us = [parameters.q01s;parameters.q02s];

data = {
    string('q01'),   string('Set flow rate for tank 1'), 1,   string('m^3/s');
    'q02', 'Set flow rate for tank 2', 0.9, 'm^3/s';
    'F1', 'Cross-sectional area of tank 1', 1.2, 'm^2';
    'F2', 'Cross-sectional area of tank 2', 1.4, 'm^2';
    'F3', 'Cross-sectional area of tank 3', 1.2, 'm^2';
    'F4', 'Cross-sectional area of tank 4', 1.3, 'm^2';
    'k11', 'Valve constant for tank 1', 0.8, 'm^2.5/s';
    'k22', 'Valve constant for tank 2', 0.8, 'm^2.5/s';
    'k33', 'Valve constant for tank 3', 0.8, 'm^2.5/s';
    'k44', 'Valve constant for tank 4', 0.8, 'm^2.5/s'};
T1 = cell2table(data, 'VariableNames', {'Symbol', 'Description', 'Value', 'Unit'});
%disp(T1)

states = {'h1'; 'h2'; 'h3'; 'h4'};  states = string(states);
inputs = {'q01';'q02'}; inputs = string(inputs);
outputs = {'h4'}; outputs = string(outputs);

T2 = table(states', inputs', outputs');
T2.Properties.VariableNames = {'States', 'Inputs', 'Outputs'};
%disp(T2)

%% Dynamics
% This code defines the dynamics of a system, specifically a linear
% time-invariant (LTI) system.

% State transition matrix. Describes how the state of the system evolves over time.
dyn.sc.dif.A(1,1)=-parameters.K1/parameters.F1; dyn.sc.dif.A(1,2)=parameters.K2/parameters.F1;
dyn.sc.dif.A(1,3)=0; dyn.sc.dif.A(1,4)=0;
dyn.sc.dif.A(2,1)=parameters.K1/parameters.F2; dyn.sc.dif.A(2,2)=-(parameters.K1+parameters.K2)/parameters.F2;
dyn.sc.dif.A(2,3)=0; dyn.sc.dif.A(2,4)=0;
dyn.sc.dif.A(3,1)=0; dyn.sc.dif.A(3,2)=parameters.K2/(2*parameters.F3); 
dyn.sc.dif.A(3,3)=-parameters.K3/parameters.F3; dyn.sc.dif.A(3,4)=0;
dyn.sc.dif.A(4,1)=0; dyn.sc.dif.A(4,2)=parameters.K2/(2*parameters.F4);
dyn.sc.dif.A(4,3)=parameters.K3/parameters.F4; dyn.sc.dif.A(4,4)=-parameters.K4/parameters.F4;
% Input matrix. Describes how the inputs affect the state evolution.
dyn.sc.dif.B(1,1)= 1/parameters.F1; dyn.sc.dif.B(1,2)=0;
dyn.sc.dif.B(2,1)=0; dyn.sc.dif.B(2,2)= 1/parameters.F2;
dyn.sc.dif.B(3,1)=0;dyn.sc.dif.B(3,2)=0;
dyn.sc.dif.B(4,1)=0;dyn.sc.dif.B(4,2)=0;
% Output matrix. Describes how the states are mapped to the outputs.
dyn.sc.dif.C = [0 0 0 1];
% Feedthrough matrix. Describes direct feedthrough from inputs to outputs.
dyn.sc.dif.D = 0;

% This is an optional parameter representing any external disturbances 
% acting on the system.
dyn.sc.dif.f=[];
% Another optional parameter representing any external disturbances acting
% on the output.
dyn.sc.dif.e=[];
% Similarly, dyn.sd.dif.f and dyn.sd.dif.e represent the same parameters 
% for the system after it has been converted to discrete-time(sd).

% Sampling time 
dyn.Ts=[0.5086];

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
con.x.max=[10;10;10;10];
con.x.min=[0.5;0.5;0.5;0.5];
% Constraints on the inputs to the system.
con.u.min=[0.1;0.1];
con.u.max=[5;5];
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
info.text = ['A system consisting of four sequentially connected tanks with' ...
    ' interdependencies is under investigation. Liquid flows to the first' ...
    ' tank with an adjustable flow-rate q01 and to the second tank with an' ...
    ' adjustable flow-rate q02It is presumed that the density of the liquid' ...
    ' remains uniform across the entirety of the system. Measurement of ' ...
    ' the liquid level is solely feasible in the fourth tank. ' ...
    ' The model was created by Bc. Sofiia Serhiienko'];

temp = which('Moli_4_tanks');
temp = split(temp,'Moli_4_tanks.m');
PathModels = temp{1};
last_backslash_index = find(PathModels == '\', 1, 'last');
PathModels = PathModels(1:last_backslash_index-1);

image_path = [PathModels , filesep , 'image', filesep , 'tanks4j.jpg'];
info.image_data = imread(image_path);
info.image= imshow(info.image_data);
end