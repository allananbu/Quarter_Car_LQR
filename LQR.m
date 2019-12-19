clc;
close all;
clear all;
%build a state space model
A=[0 0 1 0;0 0 0 1;-60.3 60.3 -1.52 1.52;366.7 -1250.3 9.25 423.25];
B=[0;0;0;3116.5];
C=[1 0 0 0;0 1 0 0;0 0 0 1;0 0 0 1];
D=0;
sys=ss(A,B,C,D);

% open loop analysis
openpoles=eig(A);
t=0:0.1:25;
u=zeros(size(t));
x0=[0.2 0 0.34 0.56];
[y,t,x]=lsim(sys,u,t,x0);

co=rank(ctrb(A,B));
ob=rank(obsv(A,C));


%pole placement controller with desired pole location with settling time
%ts<0.5s & overshoot<5%

p1=-10+10i;
p2=-10-10i;
p3=-90;p4=-95;

%pole placement ctrller design
K=place(A,B,[p1 p2 p3 p4]);
sys_cl=ss(A-B*K,B,C,D);

%closed loop impulse response

u1=(t>=0.5);
x01=[0.2 0 0.34 0.56];
[yp,tp,xp]=lsim(sys_cl,u1,t,x01);
hold on;
plot(t,yp);
hold off;
plot(t,y);


%LQR

Q=C'*C;
R=0.5;
K1=lqr(A,B,Q,R);
sys_cl2=ss(A-B*K1,B,C,D);

%CL impulse response

u2=(t>=0.5);
x02=[0.2 0 0.34 0.56];
[yp2,tp2,xp2]=lsim(sys_cl2,u2,t,x02);

plot(t,yp2);
ci2=-K1*xp2'; %control action for lqr
ci=-K*xp'; %control action for pole placement


%LQG controller + observer
ob1=-8+8i
ob2=-8-8i;
ob3=-60;ob4=-10;    
L=place(A',C',[ob1 ob2 ob3 ob4])';
At1=[A-B*K1  B*K1 
    zeros(size(A)) A-L*C];
Bt1=[B 
    zeros(size(B))];
Ct1=[C zeros(size(C))];
obsys=ss(At1,Bt1,Ct1,0);
[yp3,t,xp3]=lsim(obsys,u2,t,[x02 x02]);

plot(t,yp3);

%plot true and estimated state variables
n=4;
e=xp3(:,n+1:end);
x0=xp3();:,1:n
x0_est=x0-e;

s1=x0(:,1);
s1_est=x0_est(:,1);
plot(t,s1,'-g',t,s1_est,'.r');

s2=x0(:,2);
s2_est=x0_est(:,2);
plot(t,s2,'-g',t,s2_est,'.r');

s3=x0(:,3);
s3_est=x0_est(:,3);
plot(t,s3,'-g',t,s3_est,'.r');

s4=x0(:,4);
s4_est=x0_est(:,4);
plot(t,s4,'-g',t,s4_est,'.r');




