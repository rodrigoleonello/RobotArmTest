close all
%% Check control efforts
T = readtable('out.csv');
time = T.t;
ux = T.ux;
uy = T.uy;
uz = T.uz;

%Plot ux
figure(1)
plot(time, ux)
xlabel('Time [s]')
ylabel('ux [rad/s]')
grid on
%Plot uy
figure(2)
plot(time, uy)
xlabel('Time [s]')
ylabel('uy [rad/s]')
grid on
%Plot uz
figure(3)
plot(time, uz)
xlabel('Time [s]')
ylabel('uz [rad/s]')
grid on
%% Check angles
t1 = T.t1;
t2 = T.t2;
t3 = T.t3;

%Plot t1
figure(4)
plot(time, t1)
xlabel('Time [s]')
ylabel('t1 [rad]')
grid on
%Plot t2
figure(5)
plot(time, t2)
xlabel('Time [s]')
ylabel('t2 [rad]')
hold on
%Plot uz
figure(6)
plot(time, t3)
xlabel('Time [s]')
ylabel('t3 [rad]')
hold on
%% Check position 3D
x = T.x;
y = T.y;
z = T.z;

%Plot t1
figure(7)
plot3(x,y,z)
xlabel('x')
ylabel('y')
zlabel('z')
grid on