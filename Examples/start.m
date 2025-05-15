clear all, close all, clc

M = Moli()
M.getListOfModels();
M.barListOfModels;
%%
modelName = M.getName
%%
M.getListOfModels(1)
%%
T=M.getListOfModels();
disp(T(T.Number_of_states_nx <=3, :));
%%
M.getModel('Moli_aircraft_pitch')
figure, M.ShowPicture()
%%
M = Moli();
M.getListOfModels();
M.getModel('Moli_4_tanks');
modelLTI = M.getMPTModel();
%%
ctrl = MPCController(modelLTI,5);
data = ctrl.simulate([1;1;1;1], 101);

Nsim = 101;t=0:1:Nsim-1;
figure
subplot(2,1,1), hold on, grid on, box on
xx = (data.X + M.dynamics.params.xs)';

plot(t,xx(1:end-1,1), 'LineWidth',2, 'color',"#0072BD");
plot(t,xx(1:end-1,2), 'LineWidth',2, 'color',"#D95319");
plot(t,xx(1:end-1,3), 'LineWidth',2, 'color',"#EDB120");
plot(t,xx(1:end-1,4), 'LineWidth',2, 'color',"#7E2F8E");

for i = 1:4
    stairs(t, M.dynamics.params.xs(i) * ones(size(t)), '--', 'LineWidth',1.5)
end
legend('$x_1$', '$x_2$', '$x_3$', '$x_4$', '$x_1^s$', '$x_2^s$', ...
    '$x_3^s$', '$x_4^s$', 'Interpreter', 'latex', 'Location','east')
title('States')
xlim([0, Nsim-1]);
xlabel('$t$ [s]','Interpreter', 'latex')
ylabel('$x$ [m]','Interpreter', 'latex')
set(gca, 'fontsize', 14.5, 'ticklabelinterpreter', 'latex')


subplot(2,1,2),  hold on, grid on, box on
uu = (data.U + M.dynamics.params.us)';
stairs(t,uu(:,1), 'LineWidth',2, 'color',"#0072BD");
stairs(t,uu(:,2), 'LineWidth',2, 'color',"#80B3FF");

for i = 1:2
    plot(t, M.dynamics.params.us(i) * ones(size(t)), '--', 'LineWidth',1.5)
end
legend('$u_1$', '$u_2$', '$u_1^s$', '$u_2^s$', 'Interpreter', 'latex',  'Location','best')
title('Inputs')
xlabel('$t$ [s]','Interpreter', 'latex')
ylabel('$u$ [m$^3$/s]','Interpreter', 'latex')
ylim([0, 1.2]);
set(gca, 'fontsize', 14.5, 'ticklabelinterpreter', 'latex')


