classdef Copter_constrain< handle
 
 properties(GetAccess = public, SetAccess = public)
    Rotor1;Rotor2;Rotor3;Rotor4;
    Servo1;Servo2;
    D;
    f;
    x;
 end
 
 methods
     %---------------------インスタンス---------------------------------
     function copter=Copter_constrain(x0,Jc,Jc_dot)
         copter.x=x0;
         copter.Rotor1=Rotor();
         copter.Rotor2=Rotor();
         copter.Rotor3=Rotor();
         copter.Rotor4=Rotor();
         copter.D=Dynamics_constrain(x0,Jc,Jc_dot);
         copter.Servo1=Servo();
         copter.Servo2=Servo();
     end
     %------------------------------------------------------------------
     function Move(copter,u,dt,LEVEL)
         [n m]=size(copter.D.Jc);
         copter.Rotor1.Drive(u(1),dt);
         copter.Rotor2.Drive(u(2),dt);
         copter.Rotor3.Drive(u(3),dt);
         copter.Rotor4.Drive(u(4),dt);
         copter.Servo1.getTorque(u(5),copter.x(13),copter.x(14),copter.D.D.Gq(7));
         copter.Servo2.getTorque(u(6),copter.x(15),copter.x(16),copter.D.D.Gq(8));
         copter.f=[copter.Rotor1.thrust,copter.Rotor2.thrust,copter.Rotor3.thrust,copter.Rotor4.thrust,...
             copter.Rotor1.tau_c,copter.Rotor2.tau_c,copter.Rotor3.tau_c,copter.Rotor4.tau_c,0,0];%copter.Servo1.tau,copter.Servo2.tau]';
         %copter.f=[1.3*9.8/4 1.3*9.8/4 1.3*9.8/4 1.3*9.8/4 0 0 0 0 copter.Servo1.tau copter.Servo2.tau]';
         copter.D.Move(copter.f,dt,LEVEL);         
         copter.x(1:6,1)=copter.D.x(1:6,1);
         copter.x(7:12,1)=copter.D.x(9+n:14+n,1);
         copter.x(13)=copter.D.x(7);
         copter.x(14)=copter.D.x(15+n);
         copter.x(15)=copter.D.x(8);
         copter.x(16)=copter.D.x(16+n);         
     end
    %------------------------------------------------------------------
 end
end