function [properties, dyn,con, info] = Moli_3_tanks()
%%
if nargout <= 1
    properties.nx  = 3;
    properties.nu = 2;
    properties.ny = 2;
    return;
end
% This code appears to define properties of a system based on the number of
% output arguments expected from a function. If only one output argument is 
% expected, it sets specific properties (nx, nu, ny) and returns. Otherwise,
% it continues with further code execution.

%% Parameters of the system
parameters.F1 = 1.62; parameters.F2 = 1.85; parameters.F3 = 1.57; 
parameters.k11 =  0.1980; parameters.k22 = 0.3820; parameters.k33 =  0.2950; 
parameters.q01s = 0.3650; parameters.q02s = 0.2730; 
parameters.h1s = 0.4856; parameters.h2s = 0.4613;parameters.h3s = 0.7735;
parameters.vs1 = 0.3780; parameters.vs2 = 0.4450;

gradient = [
   -0.0877         0         0    0.2253         0;
    0.0768   -0.1520         0         0    0.1476;
         0    0.1791   -0.1068         0         0];

parameters.xs = [parameters.h1s;parameters.h2s;parameters.h3s];
parameters.us = [parameters.q01s;parameters.q02s];

data = {
    string('q01'),   string('Set flow rate for tank 1'), 0.3650,   string('m^3/s');
    'q02', 'Set flow rate for tank 2', 0.2730, 'm^3/s';
    'F1', 'Cross-sectional area of tank 1', 1.6200, 'm^2';
    'F2', 'Cross-sectional area of tank 2', 1.8500, 'm^2';
    'F3', 'Cross-sectional area of tank 3', 1.5700, 'm^2';
    'k11', 'Valve constant for tank 1', 0.1980, 'm^2.5/s';
    'k22', 'Valve constant for tank 2', 0.3820, 'm^2.5/s';
    'k33', 'Valve constant for tank 3', 0.2950, 'm^2.5/s';
    'vs1', 'Velocity of water in pipe 1', 0.3780, '-'
    'vs2', 'Velocity of water in pipe 2', 0.4450, '-'};
T1 = cell2table(data, 'VariableNames', {'Symbol', 'Description', 'Value', 'Unit'});
%disp(T1)

states = {'h1'; 'h2'; 'h3'};  states = string(states);
inputs = {'q01';'q02'}; inputs = string(inputs);
outputs = {'h1'; 'h3'}; outputs = string(outputs);

T2 = table(states', inputs', outputs');
T2.Properties.VariableNames = {'States', 'Inputs', 'Outputs'};
%disp(T2)

%% Dynamics
% This code defines the dynamics of a system, specifically a linear
% time-invariant (LTI) system.

% State transition matrix. Describes how the state of the system evolves over time.
dyn.sc.dif.A = gradient(:,1:nx);
% Input matrix. Describes how the inputs affect the state evolution.
dyn.sc.dif.B = gradient(:,nx+1:end);
% Output matrix. Describes how the states are mapped to the outputs.
dyn.sc.dif.C = [1,0,0;0,0,1];
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

% The continuous-time state-space model defined by matrices dyn.sc.dif.A,
% dyn.sc.dif.B, dyn.sc.dif.C, and dyn.sc.dif.D is converted to discrete-time
% using the function c2d.
dyn.sysc=ss(dyn.sc.dif.A,dyn.sc.dif.B,dyn.sc.dif.C,dyn.sc.dif.D);

% Sampling time 
lambda= eig(dyn.sysc.A);
dominant_lambda = max(abs(lambda));
T = 1/dominant_lambda;
dyn.Ts=T/4; % Ts =[T/5 T/2] 

dyn.sysd=c2d(dyn.sysc,dyn.Ts);

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
con.x.max=[10;10;10];
con.x.min=[0.5;0.5;0.5];
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
info.text = ['We study the system of three interconnected tanks. We consider' ...
    ' ideal tanks, with constant base F1,2,3, with controlled liquid' ...
    ' levels h1,3, an unknown (unmeasured) level h2, maximum inlet flows' ...
    ' q0,1, q0,2, and manipulated variables v1,2. Note, that the valve' ...
    ' openings v1,2 are constrained to [0, 1].' ...
    ' The model was created by doc. Ing. MSc. Martin Klaučo, PhD.'];

temp = which('Moli_3_tanks');
temp = split(temp,'Moli_3_tanks.m');
PathModels = temp{1};
last_backslash_index = find(PathModels == '\', 1, 'last');
PathModels = PathModels(1:last_backslash_index-1);

image_path = [PathModels , filesep , 'image', filesep , 'tanks3j.jpg'];
info.image_data = imread(image_path);
info.image= imshow(info.image_data);
end