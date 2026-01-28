clc; clear; close all;
set(groot, 'defaultTextInterpreter','latex')
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultLineLineWidth', 1.2);
set(groot, 'defaultLegendAutoUpdate','off');

%% Parameters
% r1 = 5;
% r2 = -3;
% vd = 3;

r1 = 5;
r2 = -3;
vd = 3;

m = 1.725;
g = 9.81;
J =  diag([0.029125,0.029125,0.055225])*3;
Jinv = inv(J);
%% Initilaization
t = 0:0.01:30;
initial_conditions_0;
%%
gains;
%% Solve
tic;
[T,Z]=ode89(@(t,z)UAV_quasi(t, z, m,g,J,Jinv,K1,K2,K3,K4,r1,r2,vd),t,IC);
toc;


%% Getting input and Rd signal
for i = 1 :length(T)
    [~,u__t(i,1),tau__1(i,1),tau__2(i,1),tau__3(i,1),h1(i,1),h2(i,1),h3(i,1),h4(i,1),ch(i,1),ch_dt(i,1),tmp(i,1)] = UAV_quasi(T(i),Z(i,:),m,g,J,Jinv,K1,K2,K3,K4,r1,r2,vd);
end
%% Fetching data
% Translational
p1 = Z(:,1); p2 = Z(:,2); p3 = Z(:,3);
v1 = Z(:,4); v2 = Z(:,5); v3 = Z(:,6);

%SO(3)
R11 = Z(:,7);  R12 = Z(:,8);  R13 = Z(:,9);
R21 = Z(:,10);  R22 = Z(:,11); R23 = Z(:,12);
R31 = Z(:,13); R32 = Z(:,14); R33 = Z(:,15);
for i=1:length(T)
    [UU,S,VV] = svd([R11(i), R12(i), R13(i);
        R21(i), R22(i), R23(i);
        R31(i), R32(i), R33(i)]);
    R_s(:,:,i) =  UU*VV.';
    rr31(i) = R_s(3,1,i);
    rr32(i) = R_s(3,2,i);
    rr11(i) = R_s(1,1,i);
    p_s(:,i)=  [Z(i,1); Z(i,2); Z(i,3)];

    eta = rotm2eul(R_s(:,:,i),'YXZ');

    tht(i,1) = eta(1);
    phi(i,1) = eta(2);
    psi(i,1) = eta(3);

end
% Omega
OM1 = Z(:, 16);
OM2 = Z(:, 17);
OM3 = Z(:, 18);

%% Positions
figure;
subplot(2,2,1); hold on; grid on;
plot(T,p1);
xlabel('Time(s)')
ylabel('$p_1$(m)')

subplot(2,2,2); hold on; grid on;
plot(T,p2);
xlabel('Time(s)')
ylabel('$p_2$(m)')

subplot(2,2,3); hold on; grid on;
plot(T,p3);
xlabel('Time(s)')
ylabel('$p_3$(m)')

subplot(2,2,4); hold on; grid on;
plot(T,psi);
xlabel('Time(s)')
ylabel('$\psi$(rad)')


%% phi, theta
figure;
subplot(1,2,1); hold on; grid on;
plot(T,phi);
xlabel('Time(s)')
ylabel('$\phi$(rad)')

subplot(1,2,2); hold on; grid on;
plot(T,tht);
xlabel('Time(s)')
ylabel('$\theta$(rad)')

%% Output
figure;
subplot(2,2,1); hold on; grid on;
plot(T,h1);
xlabel('Time(s)')
ylabel('$h_1$')

subplot(2,2,2); hold on; grid on;
plot(T,h2);
xlabel('Time(s)')
ylabel('$h_2$')

subplot(2,2,3); hold on; grid on;
plot(T,h3);
xlabel('Time(s)')
ylabel('$h_3$')

subplot(2,2,4); hold on; grid on;
plot(T,h4);
xlabel('Time(s)')
ylabel('$h_4$')

%% Input
figure;
subplot(2,2,1); hold on; grid on;
plot(T,u__t);
xlabel('Time(s)')
ylabel('$u_t$(N)')

subplot(2,2,2); hold on; grid on;
plot(T,tau__1);
xlabel('Time(s)')
ylabel('$\tau_1$(N.m)')

subplot(2,2,3); hold on; grid on;
plot(T,tau__2);
xlabel('Time(s)')
ylabel('$\tau_2$(N.m)')

subplot(2,2,4); hold on; grid on;
plot(T,tau__3);
xlabel('Time(s)')
ylabel('$\tau_3$(N.m)')

%% Checking

figure; hold on; grid on;

plot(T,ch)
xlabel('Time(s)')
title('Checkig signals')


figure; hold on; grid on;

plot(T,ch_dt)
plot(T(2:end),diff(ch)./diff(T),'--')
xlabel('Time(s)')
legend('$\frac{d}{dt}$ch','diff(ch)/diff(T)')
title('Checkig derivatives')

figure; hold on; grid on;

plot(T,tmp)
xlabel('Time(s)')
title('Checkig signals')

%% Animate
d_arm = 1;
d_prop = d_arm/2;
THT = -45;


snapshots_UAV(T,p_s,R_s, d_arm, d_prop, THT,r1,r2)
% animate_UAV(0,'UAV_example_inv',T,p_s,R_s, d_arm, d_prop, THT,r1,r2,vd)