classdef Rotor < handle
    
 properties(Constant)
    K=0.936; tau=0.178;
    Ktau=5.039*10^(-6); KT=3.026*10^(-7);
 end
 
 properties(GetAccess = public, SetAccess = public)
    rot;
    thrust;
    tau_c;
 end
 
 methods
     %---------------------インスタンス---------------------------------
     function R=Rotor()
         R.rot=0;
         R.thrust=0;
         R.tau_c=0;
     end
     %-------------------------セッター---------------------------------
     function init(R,thrust)
         R.rot=sqrt(thrust/R.KT);
         R.thrust=thrust;
         R.tau_c=R.Ktau*R.rot*R.rot;
     end
     %------------------------------------------------------------------
     function Drive(R,rot_ref,dt)
         R.rot=(R.rot+rot_ref.*dt./R.tau)/(1+dt/R.tau);
         R.tau_c=R.Ktau*R.rot*R.rot;
         R.thrust=R.KT*R.rot*R.rot;         
     end
    %------------------------------------------------------------------
 end
end