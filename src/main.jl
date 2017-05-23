clear all;
clc;
%units:m,kg,s£¬deg
disp('resluts include; actual releasing angle, x-axis releasing velocity, y-axisreleasing velocity')
disp('combined releasing velocity, releasing time')
disp('the maximum height of ball flying, the distance of ball flying, the landing time of ball')
global params;
params.flag1=0;
params.flag2=0;
params.flag3=0;
params.flag4=0;
params.g=9.81; 
params.m1=45; %mass of counter weight
params.m2=5; %mass of arm
params.m3=20; %mass of frame
params.m4=0.15; %mass of ball
params.l1=-0.75; %length of pivot to arm CG
params.l2=0.5; %length of short arm
params.l3=0.6; %length of counter weight
params.l4=2; %length of long arm
params.l5=2; %length of string
params.I=2.6; %inertia of arm
params.theta0=48; %start angle
params.Release_angle=45;
params.max_height=0;
params.Distance=0;


tout=[0:0.0005:7];
x0=[params.theta0*(pi/180);0;0;params.l5-cos(params.theta0*(pi/180))*params.l4;-params.l4*sin(params.theta0*(pi/180));0;0;0;0;0];
xout=ode4(@trebuchet,tout,x0);


subplot(2,2,1);
plot(tout,xout(:,1)*(180/pi));
title('funnction plot of arm angle and time');
xlabel('time(s)');
ylabel('arm angle(deg)');
grid on;
subplot(2,2,2);
plot(tout,xout(:,2)*(180/pi));
title('funnction plot of counter weight angle and time');
xlabel('time(s)');
ylabel('counter weight angle(deg)');
grid on;
subplot(2,2,3);
plot(tout,xout(:,3));
title('funnction plot of frame location and time');
xlabel('time(s)');
ylabel('frame location(m)');
grid on;
subplot(2,2,4);
plot(xout(:,4),xout(:,5));
title('funnction plot of ball x location and ball y locaiton');
xlabel('x location(m)');
ylabel('y location(m)');
axis([-params.l4-params.l5-2 params.Distance+5 -params.l4*sin(params.theta0*(pi/180))-2 params.max_height+5]);
grid on;


max_height=params.max_height
Distance=params.Distance

