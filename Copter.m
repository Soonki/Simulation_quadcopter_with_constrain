classdef Copter< handle
 
 properties(GetAccess = public, SetAccess = public)
    Rotor1;Rotor2;Rotor3;Rotor4;
    Servo1;Servo2;
    D;
    f;
    x;
 end
 
 methods
     %---------------------インスタンス---------------------------------
     function copter=Copter(x0)
         copter.x=x0;
         copter.Rotor1=Rotor();
         copter.Rotor2=Rotor();
         copter.Rotor3=Rotor();
         copter.Rotor4=Rotor();
         copter.D=Dynamics(x0);
         copter.Servo1=Servo();
         copter.Servo2=Servo();
         copter.Rotor1.init((copter.D.m0+copter.D.m1+copter.D.m2)*9.8/4);
         copter.Rotor2.init((copter.D.m0+copter.D.m1+copter.D.m2)*9.8/4);
         copter.Rotor3.init((copter.D.m0+copter.D.m1+copter.D.m2)*9.8/4);
         copter.Rotor4.init((copter.D.m0+copter.D.m1+copter.D.m2)*9.8/4);
         %copter.Rotor1.thrust=(copter.D.m0+copter.D.m1+copter.D.m2)/3.5;
         %copter.Rotor2.thrust=(copter.D.m0+copter.D.m1+copter.D.m2)/3.5;
         %copter.Rotor3.thrust=(copter.D.m0+copter.D.m1+copter.D.m2)/3.5;
         %copter.Rotor4.thrust=(copter.D.m0+copter.D.m1+copter.D.m2)/3.5;
     end
     %------------------------------------------------------------------
     function Move(copter,u,dt,LEVEL)
         copter.Rotor1.Drive(u(1),dt);
         copter.Rotor2.Drive(u(2),dt);
         copter.Rotor3.Drive(u(3),dt);
         copter.Rotor4.Drive(u(4),dt);
         %copter.Servo1.getTorque(u(5),copter.D.x(13),copter.D.x(14),copter.D.Gq(7));
         %copter.Servo2.getTorque(u(6),copter.D.x(15),copter.D.x(16),copter.D.Gq(8));
         copter.f=[copter.Rotor1.thrust,copter.Rotor2.thrust,copter.Rotor3.thrust,copter.Rotor4.thrust,...
             copter.Rotor1.tau_c,copter.Rotor2.tau_c,copter.Rotor3.tau_c,copter.Rotor4.tau_c,u(5),u(6)]';%copter.Servo1.tau,copter.Servo2.tau]';
         % copter.f=[3.7*9.8/4 3.7*9.8/4 3.7*9.8/4 3.7*9.8/4 0 0 0 0 copter.Servo1.tau copter.Servo2.tau]';
         copter.D.Move(copter.f,dt,LEVEL);
         copter.x=copter.D.x;
     end
    %------------------------------------------------------------------
 end
end