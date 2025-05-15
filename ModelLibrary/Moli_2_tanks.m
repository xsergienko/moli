function [properties, dyn,con, info] = Moli_2_tanks()
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
parameters.F1 = 1.2; parameters.F2 = 0.1; 
parameters.k11 =  1.3; parameters.k22 = 1.0; 
parameters.q01s = 1; 
parameters.h1s=(parameters.q01s/parameters.k11)^2;
parameters.h2s = ((2*parameters.q01s)/parameters.k22)^2;
parameters.K1 = parameters.k11/(2*sqrt(parameters.h1s)); 
parameters.K2 = parameters.k22/(2*sqrt(parameters.h2s));

parameters.xs = [parameters.h1s;parameters.h2s];
parameters.us = [parameters.q01s];

data = {
    string('q01'),   string('Set flow rate for tank 1'), 1,   string('m^3/s');
    'F1', 'Cross-sectional area of tank 1', 1.2, 'm^2';
    'F2', 'Cross-sectional area of tank 2', 0.1, 'm^2';
    'k11', 'Valve constant for tank 1', 1.3, 'm^2.5/s';
    'k22', 'Valve constant for tank 2', 1.0, 'm^2.5/s';};
T1 = cell2table(data, 'VariableNames', {'Symbol', 'Description', 'Value', 'Unit'});
%disp(T1)

states = {'h1'; 'h2'};  states = string(states);
inputs = {'q01'}; inputs = string(inputs);
outputs = {'h2'}; outputs = string(outputs);

T2 = table(states', inputs', outputs');
T2.Properties.VariableNames = {'States', 'Inputs', 'Outputs'};
%disp(T2)

%% Dynamics
% This code defines the dynamics of a system, specifically a linear
% time-invariant (LTI) system.

% State transition matrix. Describes how the state of the system evolves over time.
dyn.sc.dif.A(1,1)=-parameters.K1/parameters.F1; dyn.sc.dif.A(1,2)=0;
dyn.sc.dif.A(2,1)=parameters.K1/parameters.F2; dyn.sc.dif.A(2,2)=-parameters.K2/parameters.F2;
% Input matrix. Describes how the inputs affect the state evolution.
dyn.sc.dif.B(1,1)= 1/parameters.F1; 
dyn.sc.dif.B(2,1)= 1/parameters.F2; 
% Output matrix. Describes how the states are mapped to the outputs.
dyn.sc.dif.C = [0 1];
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
con.x.max=[10;10];
con.x.min=[0.5;0.5];
% Constraints on the inputs to the system.
con.u.min=[0.1];
con.u.max=[5];
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
info.text = ['We study a system of two tanks with straight vertical walls that hold ' ...
    'a liquid and are connected in series. The liquid enters a tank on ' ...
    'its top. The flow-rate of this stream q0,1(t) is freely adjustable. ' ...
    'An out-flowing stream leaves the ith tank at its bottom with ' ...
    'the flow-rate that is driven by gravity, i.e., Torricelli''s law ' ...
    'qi(t) = kii*sqrt(hi(t)), where kii is a valve constant and hi(t) is ' ...
    'the height of the liquid in the tank. Two streams enter the second tank, ' ...
    'q0,1(t) and q1(t). The cross-sectional area of the ith tank is ' ...
    'denoted as Fi. We can assume that the density of the liquid (ρ) ' ...
    'is constant everywhere. We can only measure the level in the second tank. ' ...
    'The model was created by Ing. Lenka Galčíková, PhD.'];

temp = which('Moli_2_tanks');
temp = split(temp,'Moli_2_tanks.m');
PathModels = temp{1};
last_backslash_index = find(PathModels == '\', 1, 'last');
PathModels = PathModels(1:last_backslash_index-1);

image_path = [PathModels , filesep , 'image', filesep , 'tanks2j.jpg'];
info.image_data = imread(image_path);
info.image= imshow(info.image_data);
end