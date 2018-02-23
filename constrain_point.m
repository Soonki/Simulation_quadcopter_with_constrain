function [Jc,Jc_dot]=constrain_point(x)
    D=Dynamics(zeros(16,1));

    
    m=50;
    Cph=round(cos(x(4)),m);Sph=round(sin(x(4)),m);
    Cth=round(cos(x(5)),m);Sth=round(sin(x(5)),m);
    Cps=round(cos(x(6)),m);Sps=round(sin(x(6)),m);
%    C1=round(cos(x(13)),m);S1=round(sin(x(13)),m);
 %   C12=round(cos(x(13)+x(15)-pi/2),m);S12=round(sin(x(13)+x(15)-pi/2),m);
    
    ph_dot=x(10); th_dot=x(11); ps_dot=x(12); %et1_dot=x(14);et2_dot=x(16);et12_dot=x(14)+x(16);
    
%     pb=[D.L1*S1-D.L2*S12;
%         0;
%         D.L1*C1-D.L2*C12];
%     
%     pb_dot=[pb(3)*et1_dot-D.L2*C12*et2_dot;
%             0;
%             -pb(1)*et1_dot+D.L2*S12*et2_dot];
%     
     R=round(Rotation(x(4),x(5),x(6)),m);
%     
     Rdot=[-ph_dot*Sph*Cth-th_dot*Cph*Sth -ph_dot*R(2,2)+th_dot*Cph*Cth*Sps+ps_dot*R(1,3) -ph_dot*R(2,3)+th_dot*Cph*Cth*Cps-ps_dot*R(1,2);
                   ph_dot*Cph*Cth-th_dot*Sph*Sth  ph_dot*R(1,2)+th_dot*Sph*Cth*Sps+ps_dot*R(2,3)  ph_dot*R(1,3)+th_dot*Sph*Cth*Cps-ps_dot*R(2,2);
                   -th_dot*Cth                    0                                               -th_dot*Sth*Cps-ps_dot*Cth*Sps                  ];
%     
    
     Jc=[1/D.L1 0 0 -R(2,3)  Cph*Cth*Cps -R(1,2)  0 0;
         0 1/D.L1 0 R(1,3)   Sph*Cth*Cps -R(2,2)  0 0;
         0 0 1/D.L1 0        -Sth*Cps    -Cth*Sps 0 0].*D.L1;
     
     Jc_dot=[0 0 0 -Rdot(2,3) -ph_dot*Sph*Cth*Cps-th_dot*Cph*Sth*Cps-ps_dot*Cph*Cth*Sps -Rdot(1,2)                     0 0;
             0 0 0 Rdot(1,3)  ph_dot*Cph*Cth*Cps-th_dot*Sph*Sth*Cps-ps_dot*Sph*Cth*Sps  -Rdot(2,2)                     0 0;
             0 0 0 0          -th_dot*Cth*Cps+ps_dot*Sth*Sps                            th_dot*Sth*Sps-ps_dot*Cth*Cps  0 0].*D.L1;
end
     