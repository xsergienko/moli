function [properties, dyn, con, info] = Moli_inverted_pendulum() 
%%
if nargout <= 1
    properties.nx  = 4;
    properties.nu = 1;
    properties.ny = 2;
    return;
end
% This code appears to define properties of a system based on the number of
% output arguments expected from a function. If only one output argument is 
% expected, it sets specific properties (nx, nu, ny) and returns. Otherwise,
% it continues with further code execution.

%% Parameters of the system
parameters.M = 0.5;
parameters.m = 0.2;
parameters.b = 0.1;
parameters.I = 0.006;
parameters.g = 9.8;
parameters.l = 0.3;
parameters.p = parameters.I*(parameters.M+parameters.m)+parameters.M*parameters.m*parameters.l^2;
% p is a denominator for the A and B matrices

symbols = {'M', 'm', 'b', 'I', 'g','l','p'}; %! ma byt 1 pismenko
symbols= string(symbols);
variableNames = {{'Mass of the cart'}, {'Mass of the pendulum'}, ...
{'Coefficient of friction for cart'}, {'Mass moment of inertia of the pendulum'},...
    {'Acceleration due to gravity'}, {'Length to pendulum center of mass'},...
    {'Denominator for the A and B matrices'}};
variableNames=string(variableNames);
values = [parameters.M, parameters.m , parameters.b, parameters.I, parameters.g,...
    parameters.l, parameters.p];
units = {{'kg'}, {'kg'}, {'N/m/sec'},{'kg.m^2'}, {'m/s^2'},{'m'},{'kg^2.m^2'}};
units=string(units);
T1 = table(symbols', variableNames', values', units');
T1.Properties.VariableNames = {'Symbol','Description','Value','Unit'};
%disp(T1)

%% Dynamics
% This code defines the dynamics of a system, specifically a linear
% time-invariant (LTI) system.

% State transition matrix. Describes how the state of the system evolves over time.
dyn.sc.dif.A=[0   1   0    0;
     0 -(parameters.I+parameters.m*parameters.l^2)*parameters.b/parameters.p  (parameters.m^2*parameters.g*parameters.l^2)/parameters.p   0;
     0  0    0   1;
     0 -(parameters.m*parameters.l*parameters.b)/parameters.p  parameters.m*parameters.g*parameters.l*(parameters.M+parameters.m)/parameters.p  0];
% Input matrix. Describes how the inputs affect the state evolution.
dyn.sc.dif.B=[  0;(parameters.I+parameters.m*parameters.l^2)/parameters.p;
          0; parameters.m*parameters.l/parameters.p];
% Output matrix. Describes how the states are mapped to the outputs.
dyn.sc.dif.C=[1 0 0 0;
     0 0 1 0];
% Feedthrough matrix. Describes direct feedthrough from inputs to outputs.
dyn.sc.dif.D=[0;
     0];
% This is an optional parameter representing any external disturbances 
% acting on the system.
dyn.sc.dif.f=[];
% Another optional parameter representing any external disturbances acting
% on the output.
dyn.sc.dif.e=[];
% Similarly, dyn.sd.dif.f and dyn.sd.dif.e represent the same parameters 
% for the system after it has been converted to discrete-time(sd).

% Sampling time 
dyn.Ts=[0.0446];

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

%% Constraints
% Constraints on the states to the system.
con.x.max=[1.5; 1.5; 0.35; 1.5]; %ohranicit vsetky stavy, idealne nedavat inf
con.x.min=[-1.5; -1.5; -0.35; -1.5];
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
info.source= ['https://ctms.engin.umich.edu/CTMS/?example=InvertedPendulum&section=SystemModeling'];
info.text = ['The system comprises a pendulum that is  affixed to a cart' ...
    ' driven by a motor. The aim of the control system is to maintain ' ...
    'the equilibrium of the inverted pendulum by exerting force on the cart ' ...
    'to which the pendulum is connected. A practical scenario resembling ' ...
    'this inverted pendulum system is the control of the orientation of ' ...
    'a booster rocket during its launch.'];

temp = which('Moli_inverted_pendulum');
temp = split(temp,'Moli_inverted_pendulum.m');
PathModels = temp{1};
last_backslash_index = find(PathModels == '\', 1, 'last');
PathModels = PathModels(1:last_backslash_index-1);

image_path = [PathModels , filesep , 'image', filesep , 'inv_pendulumj.jpg'];
info.image_data = imread(image_path);
info.image= imshow(info.image_data);
end
