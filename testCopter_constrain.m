close all
clear all
clc

%----------------------状態変数の初期値，目標値-----------------------------
x=zeros(16,1); %states initial values
x(2)=0.5;
x(4:6)=[0,0,0];
x(13)=0;
x(15)=0;
num=length(x);
ref=zeros(length(x),1);
ref(15)=-pi;
%--------------------------拘束条件--------------------------------------
Jc=zeros(2,8);
% Jc(1,1)=1;
% Jc(2,2)=1;
% Jc(3,3)=1;
% Jc(4,4)=1;
% Jc(5,5)=1;
% Jc(6,6)=1;

Jc(1,7)=1;
Jc(2,8)=1;
Jc_dot=zeros(2,8);
%[n nandemo]=size(Jc);
%num=num+2*n;
%-------------------------------------------------------------------------
Copter=Copter_constrain(x,Jc,Jc_dot);
%Copter=Copter(x);
Cont_attitude=cascadePIController_z_PHI();
Cont_arm=PIDController_ETA();
Cont_position=PIDController_xyz();
%-----------------------Simulation Condition-------------------------------
dt=0.0001;%Dynamics friqency
time=1;
sample=time/dt;
T=0:dt:time;
X=zeros(1,num);
dtc=0.01;%controller friqency
samplec=time/dtc;
U=zeros(1,5);
INPUT=zeros(1,5);
Uf=zeros(1,5);
um=zeros(5,1);
Ref=zeros(1,5);
u=zeros(5,1);
%--------------------------------------------------------------------------

%===============================Main roop==================================
for i=1:sample
    %----------------------------------------------------------------------
    if rem(i,sample/samplec)==1
        %Cont_attitude.Calculate(ref,x,Copter.D.D);
        %Cont_arm.Calculate(ref,x,Copter.D.D);
        
        %Cont_attitude.Calculate(ref,x,Copter.D);
        %Cont_arm.Calculate(ref,x,Copter.D);
        
        Cont_position.Calculate(ref,x,Copter.D.D);
        u(1:4)=Cont_position.u;
        u(5:6)=zeros(2,1);%Cont_arm.u;
    end
Copter.Move(u,dt,2);
%--------------------------------------------------------------------------
x=Copter.x;
X(i,1:num)=Copter.x';
%X(i,1:10)=xp';
Uf(i,1:10)=Copter.f;
U(i,1:6)=[Cont_attitude.F;Cont_arm.tau]';
INPUT(i,1:6)=u';
%x=xp;
%x=Copter.x;
Ref(i,1:num)=Cont_position.ref';%Controller.refInner';
end
%==========================================================================

%%------------------------------後処理--------------------------------------
%変数わかりやすいように置き換え
xr=X(:,1);
yr=X(:,2);
zr=X(:,3);
ph=X(:,4);
th=X(:,5);
ps=X(:,6);
et1=X(:,13);
et2=X(:,15);

%--------------------------------------------------------------------------
plot(T(1:end-1),X(:,1:6))
hold on
plot(T(1:end-1),X(:,13))
plot(T(1:end-1),X(:,15))
% plot(T(1:end-1),X(:,7:12));
% hold on
plot(T(1:end-1),Ref(:,4:6));
hold off
%--------------------------------------------------------------------------