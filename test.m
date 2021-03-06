close all
clear variables
clc

%----------------------状態変数の初期値，目標値-----------------------------
x=zeros(16,1); %states initial values
x(3)=0;
x(4:6)=[0,pi/12,0];
x(13)=0;
x(15)=0;
num=length(x);
ref=zeros(length(x),1);
ref(1)=0;
ref(2)=0;
ref(3)=0;
ref(15)=0;
%--------------------------拘束条件--------------------------------------
[Jc,Jc_dot]=constrain_point(x);
%-------------------------------------------------------------------------
Copter=Copter_constrain(x,Jc,Jc_dot);
%Copter=Copter(x);
Cont=IBcontroller_ver2();
%-----------------------Simulation Condition-------------------------------
dt=0.0001;%Dynamics friqency
time=1;
sample=time/dt;
T=0:dt:time;
X=zeros(1,num);
dtc=Cont.dt;%controller friqency
samplec=time/dtc;
U=zeros(1,5);
INPUT=zeros(1,5);
Uf=zeros(1,5);
um=zeros(5,1);
Ref=zeros(1,5);
u=zeros(5,1);
x(3)=-Copter.D.D.L1*cos(x(5));
x(1)=-Copter.D.D.L1*sin(x(5));
Copter=Copter_constrain(x,Jc,Jc_dot);
[Jc,Jc_dot]=constrain_point(x);
Copter.D.Jc=Jc;
Copter.D.Jc_dot=Jc_dot;
Copter.D.D.calculateMatrix(x,zeros(10,1),0);
ref(3)=-Copter.D.D.L1;
[n,m]=size(Jc);
%===============================Main roop==================================
for i=1:sample
    %----------------------------------------------------------------------
    if rem(i,sample/samplec)==1
       
        Cont.prepare(ref,x,Copter.D.D)
        Cont.attitude(x);
        %Cont.position_b(ref,x);
        %Cont.position_endeffecter_ver2(p_ed);
        %Cont.arm_angle();
        u(1:4)=Cont.u;
        %u=zeros(6,1);
        %u=Cont.u;
        u(5:6)=zeros(2,1);
    end
[Jc,Jc_dot]=constrain_point(x);
Copter.D.Jc=Jc;
Copter.D.Jc_dot=Jc_dot;
Copter.Move(u,dt,0);
%--------------------------------------------------------------------------
x=Copter.x;
X(i,1:num)=Copter.x';
Uf(i,1:10)=Copter.f;
U(i,1:6)=[Cont.F;0;0]';
INPUT(i,1:6)=u';
CF(i,1:n)=Copter.cforce';
%x=xp;
%x=Copter.x;
Ref(i,1:num)=ref';%Controller.refInner';

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
figure1 = figure;

% axes を作成
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% plot への行列入力を使って複数のラインを作成
plot(T(1:end-1),CF,'DisplayName','CF 対 T(1:end-1)','Parent',axes1);

box(axes1,'on');

% plot(T(1:end-1),X(:,7:12));
% hold on
%plot(T(1:end-1),Ref(:,4:6));
%--------------------------------------------------------------------------