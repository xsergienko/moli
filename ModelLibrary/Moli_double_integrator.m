function [properties, dyn, con, info] = Moli_double_integrator()
%%
if nargout <= 1
    properties.nx  = 2;
    properties.nu = 1;
    properties.ny = 1;
    return;
end
% This code appears to define properties of a system based on the number of
% output arguments expected from a function. If only one output argument is 
% expected, it sets specific properties (nx, nu, ny) and returns. Otherwise,
% it continues with further code execution.

%% Parameters of the system
parameters = [ ];
%% The states, inputs, and outputs are defined as follows:
states = {'Position'; 'Speed'};  states = string(states);
inputs = {'Acceleration'}; inputs = string(inputs);
outputs = {'Position'}; outputs = string(outputs);

T1 = table(states', inputs', outputs');
T1.Properties.VariableNames = {'States', 'Inputs', 'Outputs'};
%disp(T1)

%% Dynamics
% This code defines the dynamics of a system, specifically a linear
% time-invariant (LTI) system.

% State transition matrix. Describes how the state of the system evolves over time.
dyn.sc.dif.A= [1 1; 0 1];
% Input matrix. Describes how the inputs affect the state evolution.
dyn.sc.dif.B= [1; 0.5];
% Output matrix. Describes how the states are mapped to the outputs.
dyn.sc.dif.C= [1 0];
% Feedthrough matrix. Describes direct feedthrough from inputs to outputs.
dyn.sc.dif.D= [0];

%%
% This is an optional parameter representing any external disturbances 
% acting on the system.
dyn.sc.dif.f=[];
% Another optional parameter representing any external disturbances acting
% on the output.
dyn.sc.dif.e=[];
% Similarly, dyn.sd.dif.f and dyn.sd.dif.e represent the same parameters 
% for the system after it has been converted to discrete-time(sd).

% Sampling time 
dyn.Ts=[0.2500];

% The continuous-time state-space model defined by matrices dyn.sc.dif.A,
% dyn.sc.dif.B, dyn.sc.dif.C, and dyn.sc.dif.D is converted to discrete-time
% using the function c2d.
dyn.sysc=ss(dyn.sc.dif.A,dyn.sc.dif.B,dyn.sc.dif.C,dyn.sc.dif.D);
dyn.sysd=c2d(dyn.sysc,dyn.Ts,'tustin');

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

%% Constraints
% Constraints on the states to the system.
con.x.max=[5; 5];
con.x.min=[-5; -5];
% Constraints on the inputs to the system.
con.u.min=[-1];
con.u.max=[1];
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
info.source= ['https://en.wikipedia.org/wiki/Double_integrator'];
info.text = ['In systems and control theory, the double integrator stands ' ...
    'as a fundamental example of a second-order control system. It represents ' ...
    'the dynamics of a basic mass in one-dimensional space subjected to ' ...
    'the influence of a time-varying force input u.'];

temp = which('Moli_double_integrator');
temp = split(temp,'Moli_double_integrator.m');
PathModels = temp{1};
last_backslash_index = find(PathModels == '\', 1, 'last');
PathModels = PathModels(1:last_backslash_index-1);


image_path = [PathModels , filesep , 'image', filesep , 'dv_intj.jpg'];
info.image_data = imread(image_path);
info.image= imshow(info.image_data);
end