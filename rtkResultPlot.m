%A=load("oem719-202203031500-2.bin.err");
%A=load("oem719-202203170900-2.bin.err");
% A=load("实时.err");
% A=load("8h.err");
A=load("rt3.err");
% A=load("realtime.err");
[m,n]=size(A);
k=0;
T=A(:,2);

%经纬高
B =A(:,3);
L =A(:,4);
H =A(:,5);

%ECEF
X =A(:,6);
Y =A(:,7);
Z =A(:,8);

%ENU
E=A(:,22);
N=A(:,23);
U=A(:,24);

%基线向量
dX=A(:,9);
dY=A(:,10);
dZ=A(:,11);

%SigmaPos
SigmaPos=A(:,12);

%mxyz
mx=A(:,26);
my=A(:,27);
mz=A(:,28);

%衰减因子
PDOP=A(:,13);
HDOP=A(:,14);
VDOP=A(:,15);

%ResAmb
ResAmb1=A(:,17);
ResAmb2=A(:,18);

%Raatio
Ratio=A(:,16);

%Sats
GPSSats=A(:,19);
BDSSats=A(:,20);
Sats=A(:,21);

%beFied
isFixed=A(:,25);
%mean
MEAN=[mean(dX),mean(dY),mean(dZ)];
%rms
RMS=[rms(dX),rms(dY),rms(dZ)];
RMSenu=[rms(E),rms(N),rms(U)];
%std
STD=[std(dX),std(dY),std(dZ)];
STDenu=[std(E),std(N),std(U)];
%var
VAR=[var(dX),var(dY),var(dZ)];


%经纬高
figure(1);
subplot(3,1,1);
plot(T,B);
xlabel("time/s");
ylabel("B/°");
grid on;
subplot(3,1,2);
plot(T,L);
xlabel("time/s");
ylabel("L/°");
grid on;
subplot(3,1,3);
plot(T,H);
xlabel("time/s");
ylabel("H/m");
grid on

%ECEF
figure(2);
subplot(3,1,1);
plot(T,X);
xlabel("time/s");
ylabel("X/m");
grid on;
subplot(3,1,2);
plot(T,Y);
xlabel("time/s");
ylabel("Y/m");
grid on;
subplot(3,1,3);
plot(T,Z);
xlabel("time/s");
ylabel("Z/m");
grid on

%基线向量（误差）
figure(3);
subplot(3,1,1);
plot(T,dX,[T(1),T(m)],[MEAN(1,1),MEAN(1,1)],'LineWidth',1.0);
legend("dX","mean dX");
xlabel("time/s");
ylabel("dX/m");
grid on;
subplot(3,1,2);
plot(T,dY,[T(1),T(m)],[MEAN(1,2),MEAN(1,2)],'LineWidth',1.0);
legend("dY","mean dY");
xlabel("time/s");
ylabel("dY/m");
grid on;
subplot(3,1,3);
plot(T,dZ,[T(1),T(m)],[MEAN(1,3),MEAN(1,3)],'LineWidth',1.0);
legend("dZ","mean dZ");
xlabel("time/s");
ylabel("dZ/m");
grid on

%ENU
figure(4);
subplot(3,1,1);
plot(T,E);
xlabel("time/s");
ylabel("E/m");
grid on;
subplot(3,1,2);
plot(T,N);
xlabel("time/s");
ylabel("N/m");
grid on;
subplot(3,1,3);
plot(T,U);
xlabel("time/s");
ylabel("U/m");
grid on

%Sigma及DOP
figure(5);
subplot(3,1,1);
plot(T,PDOP,T,HDOP,T,VDOP,'LineWidth',2)
xlabel("time/s");
ylabel("DOP");
legend("PDOP","HDOP","VDOP");
grid on;
subplot(3,1,2);
plot(T,SigmaPos);
xlabel("time/s");
ylabel("SigmaPos/m");
grid on;
subplot(3,1,3);
plot(T,isFixed,'*');
xlabel("time/s");
ylabel("isFixed");
grid on;


%ratio
figure(6);
subplot(3,1,1);
plot(T,ResAmb1);
xlabel("time/s");
ylabel("ResAmb1");
grid on;
subplot(3,1,2);
plot(T,ResAmb2);
xlabel("time/s");
ylabel("ResAmb2");
grid on;
subplot(3,1,3);
plot(T,Ratio);
xlabel("time/s");
ylabel("Ratio");
grid on;


%卫星数
figure(7);
subplot(3,1,1);
plot(T,GPSSats,"*",'MarkerFaceColor','red');
xlabel("time/s");
ylabel("GPSSatNum");
grid on;
subplot(3,1,2);
plot(T,BDSSats,"*",'MarkerFaceColor','green');
xlabel("time/s");
ylabel("BDSSatNum");
grid on;
subplot(3,1,3);
plot(T,Sats,"*",'MarkerFaceColor','black');
xlabel("time/s");
ylabel("SatNum");
grid on;

%dXYZ-meanXYZ
%基线向量（误差）
figure(8);
subplot(3,1,1);
plot(T,dX-MEAN(1,1),'LineWidth',1.0);
legend("dX-mean dX");
xlabel("time/s");
ylabel("dX-mean dX/m");
grid on;
subplot(3,1,2);
plot(T,dY-MEAN(1,2),'LineWidth',1.0);
legend("dY-mean dY");
xlabel("time/s");
ylabel("dY-mean dY/m");
grid on;
subplot(3,1,3);
plot(T,dZ-MEAN(1,3),'LineWidth',1.0);
legend("dZ-mean dZ");
xlabel("time/s");
ylabel("dZ-mean dZ/m");
grid on;

%ENU
figure(9);
subplot(3,1,1);
plot(T,E-mean(E));
xlabel("time/s");
ylabel("E/m");
grid on;
subplot(3,1,2);
plot(T,N-mean(N));
xlabel("time/s");
ylabel("N/m");
grid on;
subplot(3,1,3);
plot(T,U-mean(U));
xlabel("time/s");
ylabel("U/m");
grid on;


%ENU
figure(10);
subplot(3,1,1);
plot(T,mx);
xlabel("time/s");
ylabel("mx/m");
grid on;
subplot(3,1,2);
plot(T,my);
xlabel("time/s");
ylabel("my/m");
grid on;
subplot(3,1,3);
plot(T,mz);
xlabel("time/s");
ylabel("mz/m");
grid on