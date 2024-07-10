
%%%% To use this script, place three 'To Workspace' blocks in your Simulink
%%%% model called:
%%%%%% - out.ref : connected to the references
%%%%%% - out.state: connected to the state of the system
%%%%%% - out.time: connected to a clock source
%%%% All variables must be in this order: x, y, z, phi, theta, psi, u, v, w, p, q, r


close all
%% 3D trajectory tracking plot
figure()
plot3(out.ref(:,1),out.ref(:,2),-out.ref(:,3))
hold on
plot3(out.state(:,1),out.state(:,2),-out.state(:,3))
grid on
xlim([out.ref(1,1)-0.5,out.ref(end,1)+0.5])
ylim([out.ref(1,2)-0.5,out.ref(end,2)+0.5])
zlim([out.ref(1,2)-0,out.ref(end,2)+3])
title('3D trajectory tracking')
legend('Reference','Real')



%% 2D trajectory tracking plot

figure()
plot(out.ref(:,1),out.ref(:,2))
hold on
grid on
plot(out.state(:,1),out.state(:,2))
xlabel('X [m]')
ylabel('Y [m]')
xlim([out.ref(1,1)-0.5,out.ref(end,1)+0.5])
ylim([out.ref(1,2)-0.01,out.ref(end,2)+0.01])
title('2D trajectory tracking')
legend('Reference','Real')

%% State plus reference in subplot structure

%%% Figure for Real position + references
figure()
subplot(3,1,1)
plot(out.time(:,1),out.ref(:,1),'lineWidth',1.5)
hold on
plot(out.time(:,1),out.state(:,1),'lineWidth',1.5)
grid on
ylabel(['X [m]'])
t = title('Position');
set(t, 'horizontalAlignment', 'center')
set(t, 'units', 'normalized')
%     legend('Reference', 'X_{real}')
subplot(3,1,2)
plot(out.time(:,1),out.ref(:,2),'lineWidth',1.5)
hold on
plot(out.time(:,1),out.state(:,2),'lineWidth',1.5)
grid on
ylabel(['Y [m]'])
legend('Reference', 'Real')
subplot(3,1,3)
plot(out.time(:,1),-out.ref(:,3),'lineWidth',1.5)
hold on
plot(out.time(:,1),-out.state(:,3),'lineWidth',1.5)
grid on
ylabel(['Z [m]'])
%     legend('Z_{ref}', 'Z_{real}')

%     legend(text1,text2,['RPM/10^3 ' text1],['RPM/10^3 ' text2])
%     xlabel('Time [s]')
%     ylabel('F_z','FontSize', text_size)
%     xlim([0 time2(end)])

%%% Figure for Real attitude + references
figure()
subplot(3,1,1)
plot(out.time(:,1),out.ref(:,4),'lineWidth',1.5)
hold on
plot(out.time(:,1),out.state(:,4),'lineWidth',1.5)
grid on
ylabel(['\phi [rad]'])
t = title('Attitude');
set(t, 'horizontalAlignment', 'center')
set(t, 'units', 'normalized')
subplot(3,1,2)
plot(out.time(:,1),out.ref(:,5),'lineWidth',1.5)
hold on
plot(out.time(:,1),out.state(:,5),'lineWidth',1.5)
grid on
ylabel(['\theta [rad]'])
legend('Reference', 'Real')
subplot(3,1,3)
plot(out.time(:,1),out.ref(:,6),'lineWidth',1.5)
hold on
plot(out.time(:,1),out.state(:,6),'lineWidth',1.5)
grid on
ylim([-10^-3, 10^-3])
ylabel(['\psi [rad]'])

%%% Figure for Real velocities + references
figure()
subplot(3,1,1)
plot(out.time(:,1),out.ref(:,7),'lineWidth',1.5)
hold on
plot(out.time(:,1),out.state(:,7),'lineWidth',1.5)
grid on
ylabel(['u [m/s]'])
t = title('Velocity');
set(t, 'horizontalAlignment', 'center')
set(t, 'units', 'normalized')
subplot(3,1,2)
plot(out.time(:,1),out.ref(:,8),'lineWidth',1.5)
hold on
plot(out.time(:,1),out.state(:,8),'lineWidth',1.5)
grid on
ylabel(['v [m/s]'])
legend('Reference', 'Real')
subplot(3,1,3)
plot(out.time(:,1),out.ref(:,9),'lineWidth',1.5)
hold on
plot(out.time(:,1),out.state(:,9),'lineWidth',1.5)
grid on
ylabel(['\w [m/s]'])

