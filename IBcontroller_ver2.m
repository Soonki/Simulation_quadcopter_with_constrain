classdef IBcontroller_ver2 < handle
 
 properties(Constant)
    
    D=Dynamics(zeros(12:0));
    Rt=Rotor();
    dt=0.01;
    S=Servo();
    Kp=[1 0 0;0 1 0;0 0 1];
    Kd=[1 0 0;0 1 0;0 0 1];
 end

 properties(GetAccess = public, SetAccess = private)
     u;
     int_e;
     M;
     H;
     e;    
     F;
     Q;
     C;
     ram;
     x_dot;
     x;
     Mq;
     Ux;Uy;
     p_e
 end
 
 methods
     %---------------------インスタンス---------------------------------
     function Cont=IBcontroller_ver2()
         Cont.x=zeros(16,1);
         Cont.x_dot=zeros(16,1);
         Cont.u=zeros(4,1);
         Cont.F=zeros(4,1);
         Cont.int_e=zeros(16,1);
         Cont.e=zeros(16,1);
         Cont.Q=zeros(4);
         Cont.Q(1,1:4)=ones(1,4).*Cont.Rt.KT;
         Cont.Q(2,1:4)=[-1 1 -1 1].*Cont.Rt.Ktau;
         Cont.Q(3,1:4)=[1 -1 -1 1].*Cont.Rt.KT*sqrt(2)/4*Cont.D.L;
         Cont.Q(4,1:4)=[-1 -1 1 1].*Cont.Rt.KT*sqrt(2)/4*Cont.D.L;
         Cont.C=[2 3 3.5 10 55 10 0 0;0.5 0.5 1.5 2 4 6 0 0];
         Cont.ram=[0.5 0.5 0.01 0.01 0.01 0.01 0 0];
     end
     %-------------------------セッター---------------------------------
     function init(Cont,newu)
         Cont.u=newu;
     end
     
     function Mixing(Cont)
         Cont.u=Cont.Q\Cont.F;
         
         for j=1:4
            if Cont.u(j)<0
                Cont.u(j)=0;
            end
         Cont.u(j)=sqrt(Cont.u(j));
         end
     end
     
     
     function prepare(Cont,ref,x,Dynamics)  
         Cont.M=Dynamics.Mq+[zeros(4,6);zeros(2,4),eye(2,2)*Cont.D.m0*Cont.D.L1^2];
         Cont.H=Dynamics.Cq+Dynamics.Gq;
         Cont.e=ref-x;
         Cont.int_e=Cont.int_e+Cont.e*Cont.dt;
         Cont.x_dot=(x-Cont.x)/Cont.dt;
         Cont.x=x;
         Cont.Mq=Cont.M*[Cont.x_dot(7:12,1)];
         Cont.p_e=Cont.x(1:3)+Rotation(x(4),x(5),x(6))*[Cont.D.L1*sin(x(7))-Cont.D.L2*sin(x(7)+x(8)-pi/2);0;Cont.D.L1*cos(x(7))-Cont.D.L2*cos(x(7)+x(8)-pi/2)];
     end
     
     function altitude(Cont,x)
         K13=1+Cont.C(1,3)*Cont.C(2,3)+Cont.ram(1,3);
         K23=Cont.C(1,3)+Cont.C(2,3);
         K33=Cont.ram(1,3)*Cont.C(2,3);
         
         Cont.F(1)=-(Cont.M(3,3)*(K13*Cont.e(3)+K23*Cont.e(9)+K33*Cont.int_e(3))...
             +Cont.H(3)+Cont.Mq(3)-Cont.M(3,3)*Cont.x_dot(9))/cos(x(5))/cos(x(6));
         
     end
     
     function attitude(Cont,x)
         K14=1+Cont.C(1,4)*Cont.C(2,4)+Cont.ram(1,4);
         K24=Cont.C(1,4)+Cont.C(2,4);
         K34=Cont.ram(1,4)*Cont.C(2,4);
         K15=1+Cont.C(1,5)*Cont.C(2,5)+Cont.ram(1,5);
         K25=Cont.C(1,5)+Cont.C(2,5);
         K35=Cont.ram(1,5)*Cont.C(2,5);
         K16=1+Cont.C(1,6)*Cont.C(2,6)+Cont.ram(1,6);
         K26=Cont.C(1,6)+Cont.C(2,6);
         K36=Cont.ram(1,6)*Cont.C(2,6);
         
         
         Cont.altitude(x);
         
         Cont.F(2)=Cont.M(4,4)*(K14*Cont.e(4)+K24*Cont.e(10)+K34*Cont.int_e(4))+Cont.H(4)+Cont.Mq(4)-Cont.M(4,4)*Cont.x_dot(10);
         Cont.F(3)=Cont.M(5,5)*(K15*Cont.e(5)+K25*Cont.e(11)+K35*Cont.int_e(5))+Cont.H(5)+Cont.Mq(5)-Cont.M(5,5)*Cont.x_dot(11);
         Cont.F(4)=Cont.M(6,6)*(K16*Cont.e(6)+K26*Cont.e(12)+K36*Cont.int_e(6))+Cont.H(6)+Cont.Mq(6)-Cont.M(6,6)*Cont.x_dot(12);
         
         Cont.Mixing();
     end
     
     function position_b(Cont,ref,x)
         
         K11=1+Cont.C(1,1)*Cont.C(2,1)+Cont.ram(1,1);
         K21=Cont.C(1,1)+Cont.C(2,1);
         K31=Cont.ram(1,1)*Cont.C(2,1);
         K12=1+Cont.C(1,2)*Cont.C(2,2)+Cont.ram(1,2);
         K22=Cont.C(1,2)+Cont.C(2,2);
         K32=Cont.ram(1,2)*Cont.C(2,2);
         
         Cont.altitude(x);
         
         Cont.Ux=Cont.M(1,1)/Cont.F(1)*(K11*Cont.e(1)+K21*Cont.e(7)+K31*Cont.int_e(1))+(Cont.H(1)+Cont.Mq(1)-Cont.M(1,1)*Cont.x_dot(7))/Cont.F(1);
         Cont.Uy=Cont.M(2,2)/Cont.F(1)*(K12*Cont.e(2)+K22*Cont.e(8)+K32*Cont.int_e(2))+(Cont.H(2)+Cont.Mq(2)-Cont.M(2,2)*Cont.x_dot(8))/Cont.F(1);
         
         ref_ps=asin(-Cont.Ux*sin(ref(4))+Cont.Uy*cos(ref(4)));
         ref_th=asin(-Cont.Ux*cos(ref(4))-Cont.Uy*sin(ref(4))/cos(ref_ps));
         
         Cont.e(5:6)=[ref_th;ref_ps]-x(5:6);
         
         Cont.attitude(x);
     end
     
     function arm_angle(Cont)
         K17=1+Cont.C(1,7)*Cont.C(2,7)+Cont.ram(1,7);
         K27=Cont.C(1,7)+Cont.C(2,7);
         K37=Cont.ram(1,7)*Cont.C(2,7);
         K18=1+Cont.C(1,8)*Cont.C(2,8)+Cont.ram(1,8);
         K28=Cont.C(1,8)+Cont.C(2,8);
         K38=Cont.ram(1,8)*Cont.C(2,8);
         
         %Cont.u(5)=Cont.M(7,7)*(K17*Cont.e(13)+K27*Cont.e(14)+K37*Cont.int_e(13))+Cont.H(7)+Cont.Mq(7)-Cont.M(7,7)*Cont.x_dot(14);
         %Cont.u(6)=Cont.M(8,8)*(K18*Cont.e(15)+K28*Cont.e(16)+K38*Cont.int_e(15))+Cont.H(8)+Cont.Mq(8)-Cont.M(8,8)*Cont.x_dot(16);
         %Cont.u(5)=Cont.S.torque2angle(Cont.u(5),Cont.x(13),Cont.x(14));
         %Cont.u(6)=Cont.S.torque2angle(Cont.u(6),Cont.x(15),Cont.x(16));
         U=Cont.M*([Cont.x_dot(7:12);diag([K17,K18])*[Cont.e(13);Cont.e(15)]+diag([K27,K28])*[Cont.e(14);Cont.e(16)]+diag([K37,K38])*[Cont.int_e(13);Cont.int_e(15)]])+Cont.H;
         Cont.u(5:6)=U(7:8);
     end
     
     function et_d=Inverce_kinematicsP(Cont,xd,zd)
         et_d(1,1)=atan2((Cont.D.L1^2-Cont.D.L2^2+xd^2+zd^2)/2/Cont.D.L1,-sqrt(xd^2+zd^2-((Cont.D.L1^2-Cont.D.L2^2+xd^2+zd^2)/2/Cont.D.L1)^2))-atan2(zd,xd);
         et_d(2,1)=atan2(Cont.D.L1/Cont.D.L2*(xd*cos(et_d(1))-zd*sin(et_d(1))),sqrt(xd^2+zd^2-(Cont.D.L1/Cont.D.L2*(xd*cos(et_d(1))-zd*sin(et_d(1))))^2))-atan2(zd,xd)-et_d(1);
     end
     
     function et_d=Inverce_kinematicsM(Cont,xd,zd)
         et_d(1,1)=atan2((Cont.D.L1^2-Cont.D.L2^2+xd^2+zd^2)/2/Cont.D.L1,sqrt(xd^2+zd^2-((Cont.D.L1^2-COnt.D.L2^2+xd^2+zd^2)/2/Cont.D.L1)^2))-atan2(zd,xd);
         et_d(2,1)=atan2(Cont.D.L1/Cont.D.L2*(xd*cos(et_d(1))-zd*sin(et_d(1))),sqrt(xd^2+zd^2-(Cont.D.L1/Cont.D.L2*(xd*cos(et_d(1))-zd*sin(et_d(1))))^2))-atan2(zd,xd)-et_d(1);
     end
     
     function position_endeffecter(Cont,p_ed)
         pb_ed=Rotation(Cont.x(4),Cont.x(5),Cont.x(6))'*(p_ed-[Cont.x(1);Cont.x(2);Cont.x(3)]);
         if pb_ed(1,1)^2+pb_ed(3,1)^2>(Cont.D.L1+Cont.D.L2)^2
             pb_ed(1,1)=0;
             pb_ed(2,1)=0;
             pb_ed(3,1)=0.05;
         end
         et_d=Cont.Inverce_kinematicsP(pb_ed(1),pb_ed(3));
         Cont.e(13)=et_d(1)-Cont.x(13,1);
         Cont.e(15)=et_d(2)-Cont.x(15,1);
         Cont.arm_angle();
     end
     
     function position_endeffecter_ver2(Cont,p_ed)
         %---------------------ヤコビアンの計算------------------------------------------------------------------
         m=50;
         Cph=round(cos(Cont.x(4)),m);Sph=round(sin(Cont.x(4)),m);
         Cth=round(cos(Cont.x(5)),m);Sth=round(sin(Cont.x(5)),m);
         Cps=round(cos(Cont.x(6)),m);Sps=round(sin(Cont.x(6)),m);
         C1=round(cos(Cont.x(13)),m);S1=round(sin(Cont.x(13)),m);
         C12=round(cos(Cont.x(13)+Cont.x(15)-pi/2),m);S12=round(sin(Cont.x(13)+Cont.x(15)-pi/2),m);
    
         ph_dot=Cont.x(10); th_dot=Cont.x(11); ps_dot=Cont.x(12); et1_dot=Cont.x(14);et2_dot=Cont.x(16);et12_dot=Cont.x(14)+Cont.x(16);
    
         pb=[Cont.D.L1*S1-Cont.D.L2*S12;
             0;
             Cont.D.L1*C1-Cont.D.L2*C12];
    
         pb_dot=[pb(3)*et1_dot-Cont.D.L2*C12*et2_dot;
                 0;
                 -pb(1)*et1_dot+Cont.D.L2*S12*et2_dot];
    
         R=round(Rotation(Cont.x(4),Cont.x(5),Cont.x(6)),m);
         R_dot=Rotation_dot(Cont.x(4),Cont.x(5),Cont.x(6),Cont.x_dot(4),Cont.x_dot(5),Cont.x_dot(6));
         Jc=[1 0 0 Sph*Cth*pb(1)-R(2,3)*pb(3) -Cph*Sth*pb(1)+Cph*Cth*Cps*pb(3) -R(1,2)*pb(3)  Cph*Cth*pb(3)-R(1,3)*pb(1) -Cph*Cth*Cont.D.L2*C12+R(1,3)*Cont.D.L2*S12;
         0 0 1 0                          -Cth*pb(1)-Sth*Cps*pb(3)         -Cth*Sps*pb(3) -Sth*pb(3)-Cth*Cps*pb(1)   Sth*Cont.D.L2*C12+Cth*Cps*Cont.D.L2*S12];
         %----------------------------------------------------------------------------------------------------------------
         p_e_dot=Cont.x_dot(1:3)+R*pb_dot+R_dot*pb;
         
         Fe=Cont.Kp*(p_ed-Cont.p_e)-Cont.Kd*p_e_dot;
         Cont.u(5:6)=Jc(:,7:8)\([Fe(1);Fe(3)]-Jc(:,1:6)*[Cont.Ux;Cont.Uy;Cont.F]);         
      end
     
     
 end
end