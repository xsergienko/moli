clear all, close all, clc
%%
parameters.F1 = 1.2; parameters.F2 = 1.4; 
parameters.F3 = 1.2; parameters.F4 = 1.3;
parameters.k11 =  0.8; parameters.k22 = 0.8;
parameters.k33 =  0.8; parameters.k44 =  0.8;
parameters.q01s = 1; parameters.q02s = 0.9; 
parameters.h2s = ((parameters.q01s+parameters.q02s)/parameters.k22)^2;
parameters.h1s=parameters.h2s+(parameters.q01s/parameters.k11)^2;
parameters.h3s=((parameters.k22*sqrt(parameters.h2s))/(2*parameters.k33))^2;
parameters.h4s=((((parameters.k22*sqrt(parameters.h2s))/2)+(parameters.k33* ...
    sqrt(parameters.h3s)) )/parameters.k44)^2;

parameters.K1 = parameters.k11/(2*sqrt(parameters.h1s-parameters.h2s)); 
parameters.K2 = parameters.k22/(2*sqrt(parameters.h2s));
parameters.K3 = parameters.k33/(2*sqrt(parameters.h3s)); 
parameters.K4 = parameters.k44/(2*sqrt(parameters.h4s));
parameters.xs = [parameters.h1s;parameters.h2s;parameters.h3s;parameters.h4s];
parameters.us = [parameters.q01s;parameters.q02s];

dyn.sc.dif.A(1,1)=-parameters.K1/parameters.F1; 
dyn.sc.dif.A(1,2)=parameters.K2/parameters.F1;
dyn.sc.dif.A(1,3)=0; dyn.sc.dif.A(1,4)=0;
dyn.sc.dif.A(2,1)=parameters.K1/parameters.F2; 
dyn.sc.dif.A(2,2)=-(parameters.K1+parameters.K2)/parameters.F2;
dyn.sc.dif.A(2,3)=0; dyn.sc.dif.A(2,4)=0;
dyn.sc.dif.A(3,1)=0; dyn.sc.dif.A(3,2)=parameters.K2/(2*parameters.F3); 
dyn.sc.dif.A(3,3)=-parameters.K3/parameters.F3; dyn.sc.dif.A(3,4)=0;
dyn.sc.dif.A(4,1)=0; dyn.sc.dif.A(4,2)=parameters.K2/(2*parameters.F4);
dyn.sc.dif.A(4,3)=parameters.K3/parameters.F4;
dyn.sc.dif.A(4,4)=-parameters.K4/parameters.F4;
dyn.sc.dif.B(1,1)= 1/parameters.F1; dyn.sc.dif.B(1,2)=0;
dyn.sc.dif.B(2,1)=0; dyn.sc.dif.B(2,2)= 1/parameters.F2;
dyn.sc.dif.B(3,1)=0;dyn.sc.dif.B(3,2)=0;
dyn.sc.dif.B(4,1)=0;dyn.sc.dif.B(4,2)=0;
dyn.sc.dif.C = [0 0 0 1];
dyn.sc.dif.D = 0;

dyn.sysc=ss(dyn.sc.dif.A,dyn.sc.dif.B,dyn.sc.dif.C,dyn.sc.dif.D);
lambda= eig(dyn.sysc.A);
dominant_lambda = max(abs(lambda));
T = 1/dominant_lambda;
dyn.Ts=T/4;% Ts =[T/5 T/2] 
dyn.sysd=c2d(dyn.sysc,dyn.Ts );
dyn.sd.dif.A=dyn.sysd.A;
dyn.sd.dif.B=dyn.sysd.B;
dyn.sd.dif.C=dyn.sysd.C;
dyn.sd.dif.D=dyn.sysd.D;

properties.nx  = 4;
properties.nu = 2;

model= LTISystem('A',dyn.sd.dif.A,'B',dyn.sd.dif.B,'C',dyn.sd.dif.C,'Ts',dyn.Ts);

model.u.min = [0.1;0.1]-parameters.us;
model.u.max = [5;5]-parameters.us;
model.x.max=[10;10;10;10]-parameters.xs;
model.x.min=[0.5;0.5;0.5;0.5]-parameters.xs;

model.u.penalty = QuadFunction(eye(properties.nu));
model.x.penalty = QuadFunction(eye(properties.nx));
%% 
ctrl = MPCController(model, 6)
x0 = parameters.xs.*0;
Nsim = 101;
data = ctrl.simulate([1;1;1;1], 101);
%%
t=0:1:Nsim-1;
figure
subplot(2,1,1), hold on, grid on, box on
xx = (data.X + parameters.xs)';

plot(t,xx(1:end-1,1), 'LineWidth',2, 'color',"#0072BD");
plot(t,xx(1:end-1,2), 'LineWidth',2, 'color',"#D95319");
plot(t,xx(1:end-1,3), 'LineWidth',2, 'color',"#EDB120");
plot(t,xx(1:end-1,4), 'LineWidth',2, 'color',"#7E2F8E");

for i = 1:4
    stairs(t, parameters.xs(i) * ones(size(t)), '--', 'LineWidth',1.5)
end
legend('$x_1$', '$x_2$', '$x_3$', '$x_4$', '$x_1^s$', '$x_2^s$', ...
    '$x_3^s$', '$x_4^s$', 'Interpreter', 'latex', 'Location','east')
title('States')
xlim([0, Nsim-1]);
xlabel('$t$ [s]','Interpreter', 'latex')
ylabel('$x$ [m]','Interpreter', 'latex')
set(gca, 'fontsize', 14.5, 'ticklabelinterpreter', 'latex')

subplot(2,1,2),  hold on, grid on, box on
uu = (data.U + parameters.us)';
stairs(t,uu(:,1), 'LineWidth',2, 'color',"#0072BD");
stairs(t,uu(:,2), 'LineWidth',2, 'color',"#80B3FF");

for i = 1:2
    plot(t, parameters.us(i) * ones(size(t)), '--', 'LineWidth',1.5)
end
legend('$u_1$', '$u_2$', '$u_1^s$', '$u_2^s$', 'Interpreter', 'latex',  'Location','best')
title('Inputs')
xlabel('$t$ [s]','Interpreter', 'latex')
ylabel('$u$ [m$^3$/s]','Interpreter', 'latex')
ylim([0, 1.2]);
set(gca, 'fontsize', 14.5, 'ticklabelinterpreter', 'latex')




