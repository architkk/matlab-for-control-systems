%% MATLAB BASICS FOR CONTROL SYSTEMS:
% This is to introduce the students of EE250 to MATLAB
%% Compulsary statements:
clc; clear all; close all;
%% Solving DE:
syms y(t);
Dy = diff(y,t);
ode = diff(y,t,2)-2*diff(y,t)+4*y==0
cond1 = y(0)==1
cond2 = Dy(0)==2
conds = [cond1 cond2];
ysol(t) = dsolve(ode, conds)
%% Step response computation:
s = tf('s');
sys = (10*(s+1))/(s*(s+4)*(s+6))
figure(1)
stepplot(sys);
grid on;
%% Eulers Method:
x(1) = 0.5;
h = 0.1; t = 0; time(1)=0;
for i=1:100
    x(i+1) = x(i)+h*(-x(i)+x(i)*x(i));
    t = t+h;
    time(i+1) = t;
end
x1 = x;
figure(2);
plot(time,x);
grid on;
title('Eulers Method');
xlabel('Time');
ylabel('Magnitude');
%% Range-kutta method:
x(1) = 0.5;
h = 0.1; t = 0; time(1) = 0;
for i = 1:100
    m0 = -x(i)+x(i)^2;
    m1 = -(x(i)+m0*h/2)+(x(i)+m0*h/2)^2;
    m2 = -(x(i)+m1*h/2)+(x(i)+m1*h/2)^2;
    m3 = -(x(i)+m2*h)+(x(i)+m2*h)^2;
    x(i+1) = x(i)+(h/6)*(m0+2*m1+2*m2+m3);
    t = t+h;
    time(i+1) = t;
end
x2 = x;
figure(3);
plot(time,x);
grid on;
title('RK Method');
xlabel('Time');
ylabel('Magnitude');
%% ODE 45:
tspan = [0 10];
x0 = 0.5;
[t,x] = ode45(@(t,x) -x+x^2, tspan, x0);
x3 = x;
figure(4);
plot(t,x);
grid on;
title('ODE45 Method');
xlabel('Time');
ylabel('Magnitude');
%% Joint Plot:
figure(5)
plot(time,x1,'-');
hold on;
plot(time,x2,'o');
hold on
plot(t,x3,'r');
legend('EM','RKM','ODE45');
grid on;
title('Comparison Between Methods');
xlabel('Time');
ylabel('Magnitude');
%% Second Order System:
s = tf('s');
G = 1/(s*(s+1))
D1 = (s+2)/2
sys1 = feedback(G*D1,1)
S1 = stepinfo(sys1)
figure(6);
stepplot(sys1);
grid on;
title('Step Response of Overall System');
xlabel('Time');
ylabel('Magnitude');
%% Ramp Response:
t=0:0.1:10;
ramp = t;
[x, t] = lsim(sys1,ramp,t);
figure(6);
plot(t,x);
hold on;
plot(t,t);
grid on;
title('Ramp Response of the system');
xlabel('Time');
ylabel('Magnitude');





